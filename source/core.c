// SPDX-License-Identifier: GPL-2.0+
/*
 * aspeed-vhub -- Driver for Aspeed SoC "vHub" USB gadget
 *
 * core.c - Top level support
 *
 * Copyright 2017 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/prefetch.h>
#include <linux/clk.h>
#include <linux/usb/gadget.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/dma-mapping.h>
#include <linux/random.h>
#include <linux/crc8.h>

#include "vhub.h"

DECLARE_CRC8_TABLE(vbus_crc_table);

void pr_hex_mark(const char* mem, int count, int mark);

void pr_hex(const char* mem, int count)
{
  pr_hex_mark(mem, count, 0);
}

#define PR_WRITE 1
#define PR_READ  2
#define PR_ERROR 4

void pr_hex_mark(const char* mem, int count, int mark)
{
  char hexbyte[11] = "";
  char hexline[126] = "";
  int i, k = 0;
  for (i = 0; i < count && count < sizeof(hexline); i++)
  {
    sprintf(hexbyte, "%02X|", mem[i]);
    strcat(hexline, hexbyte);
    // print line every 16 bytes or if this is the last for-loop
    if (((i + 1) % 20 == 0) && (i != 0) || (i + 1 == count)) {
      k++;
      switch (mark) {
      case PR_READ:
        printk(KERN_INFO " r %02x: %s\n", k, hexline); // print line to console
        break;
      case PR_WRITE:
        printk(KERN_INFO " w %02x: %s\n", k, hexline); // print line to console
        break;
      default:
        if (mark & PR_WRITE) {
          printk(KERN_ERR " w %02x: %s\n", k, hexline); // print line to console
        }
        else {
          printk(KERN_ERR " w %02x: %s\n", k, hexline); // print line to console
        }
        break;
      }
      //syslog(LOG_INFO, "l%d: %s",k , hexline); // print line to syslog
      memset(&hexline[0], 0, sizeof(hexline)); // clear hexline array
    }
  }
}

static ssize_t spidev_sync(struct ast_vhub* vhub, struct spi_message* message)
{
  int status;
  struct spi_device* spi;

  spin_lock_irq(&vhub->lock);
  spi = vhub->spi;
  spin_unlock_irq(&vhub->lock);

  if (spi == NULL)
    status = -ESHUTDOWN;
  else
    status = spi_sync(spi, message);

  if (status == 0)
    status = message->actual_length;

  return status;
}

static inline ssize_t spidev_sync_write(struct ast_vhub* vhub, size_t len)
{
  struct spi_transfer	t = {
      .tx_buf = vhub->transfer,
      .len = len,
      .speed_hz = vhub->spi->max_speed_hz,
  };
  struct spi_message	m;

  spi_message_init(&m);
  spi_message_add_tail(&t, &m);
  return spidev_sync(vhub, &m);
}

void ast_vhub_done(struct ast_vhub_ep* ep, struct ast_vhub_req* req,
  int status)
{
  bool internal = req->internal;

  EPVDBG(ep, "completing request @%p, status %d\n", req, status);

  list_del_init(&req->queue);

  if (req->req.status == -EINPROGRESS)
    req->req.status = status;

  if (req->req.dma) {
    if (!WARN_ON(!ep->dev))
      usb_gadget_unmap_request(&ep->dev->gadget,
        &req->req, ep->epn.is_in);
    req->req.dma = 0;
  }

  /*
   * If this isn't an internal EP0 request, call the core
   * to call the gadget completion.
   */
  if (!internal) {
    spin_unlock(&ep->vhub->lock);
    usb_gadget_giveback_request(&ep->ep, &req->req);
    spin_lock(&ep->vhub->lock);
  }
}

void ast_vhub_nuke(struct ast_vhub_ep* ep, int status)
{
  struct ast_vhub_req* req;
  int count = 0;

  /* Beware, lock will be dropped & req-acquired by done() */
  while (!list_empty(&ep->queue)) {
    req = list_first_entry(&ep->queue, struct ast_vhub_req, queue);
    ast_vhub_done(ep, req, status);
    count++;
  }
  if (count)
    EPDBG(ep, "Nuked %d request(s)\n", count);
}

struct usb_request* ast_vhub_alloc_request(struct usb_ep* u_ep,
  gfp_t gfp_flags)
{
  struct ast_vhub_req* req;

  struct ast_vhub_ep* ep = to_ast_ep(u_ep);
  struct ast_vhub* vhub = ep->vhub;
  dev_info(&vhub->spi->dev, "ast_vhub_alloc_request\n");

  req = kzalloc(sizeof(*req), gfp_flags);
  if (!req)
    return NULL;
  return &req->req;
}

void ast_vhub_free_request(struct usb_ep* u_ep, struct usb_request* u_req)
{
  struct ast_vhub_req* req = to_ast_req(u_req);

  kfree(req);
}

// read the data from the SIE
static int vusb_read_buffer(struct ast_vhub* vhub, u8* buffer, size_t length)
{
  struct spi_device* spi = vhub->spi;
  struct spi_transfer	tr;
  struct spi_message	m;

  spi_message_init(&m);

  memset(&tr, 0, sizeof(tr));

  // header 2 bytes
  tr.rx_buf = buffer;
  tr.len = 3; //sizeof(spi_cmd_t);

  spi_message_add_tail(&tr, &m);
  spi_sync(spi, &m);

  spi_cmd_t* cmd = (spi_cmd_t*)buffer;

  // check data
  if (cmd->reg.bit.read || cmd->reg.bit.write)
  {
    memset(&tr, 0, sizeof(tr));
    spi_message_init(&m);
    // data
    tr.rx_buf = &buffer[3];
    tr.len = (u8)buffer[1];
    spi_message_add_tail(&tr, &m);

    if (!spi_sync(spi, &m) && crc8(vbus_crc_table, tr.rx_buf, tr.len, 0) == buffer[2] )
      return 1;
  } 
  return 0;
}

//static u16 counts = 0;

static int vusb_write_buffer(struct ast_vhub* vhub, u8 reg, void* buffer, u8 length)
{
  struct spi_device* spi = vhub->spi;
  struct spi_transfer transfer;
  struct spi_message msg;

  memset(&transfer, 0, sizeof(transfer));
  spi_message_init(&msg);

  // copy the out data
  memmove(&vhub->transfer[3], buffer, length);

  // header reg, length, crc
  vhub->transfer[0] = reg;
  vhub->transfer[1] = length;
  vhub->transfer[2] = crc8(vbus_crc_table, &vhub->transfer[3], length, 0);

  transfer.tx_buf = vhub->transfer;
  transfer.len = 3 + length;

  spi_message_add_tail(&transfer, &msg);

  return !spi_sync(spi, &msg);
}

#define GPIO_CLIENT_GPIO_IRQ 17

static irqreturn_t ast_vhub_irq(int irq, void* data)
{
  struct ast_vhub* vhub = data;
  irqreturn_t iret = IRQ_NONE;
  u32 i, istat;
  unsigned long flags;
  // UDCVDBG(vhub, "ast_vhub_irq\n");

  /* Stale interrupt while tearing down */
  if (vhub->irq != irq)
    return IRQ_NONE;

  iret = IRQ_HANDLED;
  if (gpio_get_value(GPIO_CLIENT_GPIO_IRQ) == 1)
  {
    UDCVDBG(vhub, "ast_vhub_irq GPIO rising status irq:%d, value*:%d \n", irq, 0);
  }
  else {

    mutex_lock(&vhub->spi_bus_mutex);

  #define DEVICE_PIPE_0   0x10
  #define WRITE_CMD_WRITE 0x80
  #define WRITE_CMD_READ  0x40

    memset(vhub->transfer, 0, 512);

    //if (vusb_write_buffer(vhub, WRITE_CMD_WRITE | DEVICE_PIPE_0, vhub->transfer, varsize)) {
    //  pr_hex_mark(vhub->transfer, min(10, (u8)vhub->transfer[3]), PR_WRITE);
    //


    //}

#define DEBUG_SPI_CECK
#define MAX_PRINT_COLUMN 64

#ifdef DEBUG_SPI_CECK
    // read
    if (vusb_read_buffer(vhub, vhub->transfer, 512)) {

      // print the whole buffer
      pr_hex_mark(vhub->transfer, min(MAX_PRINT_COLUMN, (u8)vhub->transfer[3]), PR_READ);

      static u8 varsize = 1;
      get_random_bytes(&varsize, 1);
      get_random_bytes(vhub->transfer, varsize);

      // write
      if (vusb_write_buffer(vhub, WRITE_CMD_WRITE | DEVICE_PIPE_0, vhub->transfer, varsize)) {
        pr_hex_mark(vhub->transfer, min(MAX_PRINT_COLUMN, (u8)vhub->transfer[3]), PR_WRITE);
      }
      else {
        pr_hex_mark(vhub->transfer, min(MAX_PRINT_COLUMN, (u8)vhub->transfer[3]), PR_WRITE | PR_ERROR);
      }
    }
    else {
      UDCVDBG(vhub, "ast_vhub_irq error with irq:%d, value*:%d \n", irq, 0);
    }

#endif // DEBUG

    mutex_unlock(&vhub->spi_bus_mutex);
  }

bail:
  return iret;
}

void ast_vhub_init_hw(struct ast_vhub* vhub)
{
  u32 ctrl, port_mask, epn_mask;

  UDCDBG(vhub, "(Re)Starting HW ...\n");

}

static int ast_vhub_remove(struct spi_device* spi)
{
  struct ast_vhub* vhub = spi_get_drvdata(spi);

  unsigned long flags;
  int i;

  if (!vhub)
    return 0;

  dev_info(&spi->dev, "Remove of v-hub.\n");

  /* disable the slave IRQ line */
  disable_irq(vhub->irq);

  /* Remove devices */
  for (i = 0; i < vhub->max_ports; i++) {
    dev_info(&spi->dev, "Device remove: %s.\n", dev_name(vhub->ports[i].dev.port_dev));
    ast_vhub_del_dev(&vhub->ports[i].dev);
  }

  spin_lock_irqsave(&vhub->lock, flags);
  spin_unlock_irqrestore(&vhub->lock, flags);

  return 0;
}

static int ast_vhub_probe(struct spi_device* spi)
{
  enum usb_device_speed max_speed;
  struct ast_vhub* vhub;
  struct resource* res;
  int i, rc = 0;

  const struct device_node* np = spi->dev.of_node;

  vhub = devm_kzalloc(&spi->dev, sizeof(*vhub), GFP_KERNEL);
  if (!vhub)
    return -ENOMEM;

  // change to device tree later
  rc = of_property_read_u32(np, "max-ports", &vhub->max_ports);
  if (rc < 0) {
    dev_err(&spi->dev, "Unable to allocate hub downstreams ports.\n");
    return -ENOMEM;
  }

  vhub->ports = devm_kcalloc(&spi->dev, vhub->max_ports,
    sizeof(*vhub->ports), GFP_KERNEL);
  if (!vhub->ports)
    return -ENOMEM;

  dev_info(&spi->dev, "Hub device has initiated %d hub ports.\n", vhub->max_ports);

  rc = of_property_read_u32(np, "hubshield,eps", &vhub->max_epns);
  vhub->epns = devm_kcalloc(&spi->dev, vhub->max_epns,
    sizeof(*vhub->epns), GFP_KERNEL);
  if (!vhub->epns)
  {
    dev_err(&spi->dev, "Unable to allocate Hub endpoints.\n");
    return -ENOMEM;
  }

  vhub->transfer = devm_kcalloc(&spi->dev, 1024,
    sizeof(*vhub->transfer), GFP_KERNEL);
  if (!vhub->transfer)
  {
    dev_err(&spi->dev, "Unable to allocate Hub transfer buffer.\n");
    return -ENOMEM;
  }

  // init crc8
  crc8_populate_msb(vbus_crc_table, 0x7);

  vhub->spi = spi;
  spi_set_drvdata(spi, vhub);

  vhub->irq = spi->irq;
  if (vhub->irq < 0)
  {
    dev_err(&vhub->spi->dev, "Irq missing in platform data");
    return -ENODEV;
  }

  // spi defs
  vhub->spi->mode = SPI_MODE_0;
  spi->bits_per_word = 8;
  if (spi_setup(vhub->spi) < 0) {
    dev_err(&vhub->spi->dev, "Unable to setup SPI bus");
    return -EFAULT;
  }

  dev_info(&vhub->spi->dev, "Spi clock set at %u KHz.\n",
    (vhub->spi->max_speed_hz + 500) / 1000);

  spin_lock_init(&vhub->lock);
  mutex_init(&vhub->spi_bus_mutex);

  vhub->port_irq_mask = GENMASK(VHUB_IRQ_DEV1_BIT + vhub->max_ports - 1,
    VHUB_IRQ_DEV1_BIT);

  //rc = devm_gpiod_get(&vhub->spi->dev, "reset-gpios", GPIOD_IN);
  rc = of_property_read_u32(np, "reset-gpios", &vhub->reset_gpio);
  dev_info(&vhub->spi->dev, "Reset gpio is defined as gpio:%d\n", rc);

  // set the irq handler
  rc = devm_request_threaded_irq(&vhub->spi->dev, vhub->irq,
    NULL, ast_vhub_irq, IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, "vusbsoc", vhub);
  if (rc)
  {
    dev_err(&vhub->spi->dev, "Failed to request interrupt\n");
    rc = -ENOMEM;
    goto err;
  }

  vhub->ep0_bufs = kzalloc(AST_VHUB_EPn_MAX_PACKET *
    (vhub->max_ports + 1),
    GFP_KERNEL);
  if (!vhub->ep0_bufs)
  {
    dev_err(&spi->dev, "Failed to allocate EP0 DMA buffers\n");
    rc = -ENOMEM;
    goto err;
  }

  // Init vHub EP0
  ast_vhub_init_ep0(vhub, &vhub->ep0, NULL);

  // Init devices
  for (i = 0; i < vhub->max_ports && rc == 0; i++)
  {
    rc = ast_vhub_init_dev(vhub, i);
  }
  if (rc)
    goto err;

   //  Init hub emulation
   rc = ast_vhub_init_hub(vhub);
   if (rc)
   	goto err;

  //  Initialize HW
  ast_vhub_init_hw(vhub);

  dev_info(&spi->dev, "Initialized virtual hub in USB%d mode\n",
    vhub->force_usb1 ? 1 : 2);

  return 0;
err:
  ast_vhub_remove(spi);
  dev_err(&spi->dev, "Failed to initialize vhub.\n");
  return rc;
}

static const struct of_device_id ast_vhub_dt_ids[] = {
  {.compatible = "hubshield,v-hub", },
  { }
};
MODULE_DEVICE_TABLE(of, ast_vhub_dt_ids);

static struct spi_driver ast_vhub_driver = {
  .probe = ast_vhub_probe,
  .remove = ast_vhub_remove,
  .driver = {
    .name = KBUILD_MODNAME,
    .of_match_table = ast_vhub_dt_ids,
  },
};
//module_platform_driver(ast_vhub_driver);
module_spi_driver(ast_vhub_driver);

MODULE_DESCRIPTION("Hubshield virtual hub udc driver");
MODULE_AUTHOR("Manfred Kubica <manfredkubica@web.de>");
MODULE_LICENSE("GPL");
