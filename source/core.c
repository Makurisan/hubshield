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
  char hexbyte[64] = "";
  char hexline[256] = "";
  int i, k = 0;
  for (i = 0; i < count && count < sizeof(hexline); i++)
  {
    if (i == 3)
      sprintf(hexbyte, "%02X] length: %d\n   ", mem[i], (u16)mem[2]);
    else
      sprintf(hexbyte, "%02X ", mem[i]);
    strcat(hexline, hexbyte);

    // print line every 16 bytes or if this is the last for-loop
    if (((i + 1) % 24 == 0) && (i != 0) || (i + 1 == count)) {
      k++;
      switch (mark) {
      case PR_READ:
        if (k == 1)
          printk(KERN_INFO "r [%s\n", hexline); // print line to console
        else
          printk(KERN_INFO "   %s\n", hexline); // print line to console
        break;
      case PR_WRITE:
        if (k == 1)
          printk(KERN_INFO "w [%s\n", hexline); // print line to console
        else
          printk(KERN_INFO "   %s\n", hexline); // print line to console
        break;
      default:
        if (mark & PR_WRITE) {
          printk(KERN_ERR " w %02x: %s\n", k, hexline); // print line to console
        }
        else {
          printk(KERN_ERR " r %02x: %s\n", k, hexline); // print line to console
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

static DECLARE_WAIT_QUEUE_HEAD(wq);
static int flag = 0;
static int value= 0;

// read the data from the SIE
static int vusb_read_buffer(struct ast_vhub* vhub, u8* buffer, u16 length)
{
  struct spi_device* spi = vhub->spi;
  struct spi_transfer	tr;
  struct spi_message	m;

  spi_message_init(&m);

  memset(&tr, 0, sizeof(tr));

  tr.rx_buf = buffer;
  // header 4 bytes
  tr.len = offsetof(spi_cmd_t, data);

  spi_message_add_tail(&tr, &m);
  spi_sync(spi, &m);

  spi_cmd_t* cmd = (spi_cmd_t*)buffer;

  // check data
  if (cmd->reg.bit.read || cmd->reg.bit.write)
  {
    memset(&tr, 0, sizeof(tr));
    spi_message_init(&m);
    // data
    tr.rx_buf = &buffer[offsetof(spi_cmd_t, data)];
    if (cmd->length < 1024)
    {
      tr.len = cmd->length;
      spi_message_add_tail(&tr, &m);
      if (!spi_sync(spi, &m) && crc8(vbus_crc_table, tr.rx_buf, tr.len, 0) == cmd->crc8) {
      //  UDCVDBG(vhub, "vusb_read_buffer back from event wait: %02d, flag: %d, value:%d\n", cmd->length, flag, value);
        flag = 1; 
        wake_up_interruptible(&wq);
        return 1;
      }
    }
    else {
      UDCVDBG(vhub, "vusb_read_buffer spi buffer exceeds maximal size length: %02x\n", cmd->length);
    }
  }
  return 0;
}

static int vusb_write_buffer(struct ast_vhub* vhub, u8 reg, u8* buffer, u16 length)
{
  struct spi_device* spi = vhub->spi;
  struct spi_transfer t;
  struct spi_message msg;

  memset(&t, 0, sizeof(t));
  spi_message_init(&msg);

  // header reg, length, crc
  spi_cmd_t* cmd = (spi_cmd_t*)vhub->transfer;

  // copy the out data
  //memmove(vhub->transfer + 4, buffer, length);

  cmd->reg.val = reg;
  cmd->crc8 = crc8(vbus_crc_table, cmd->data, length, 0);
  cmd->length = length;

  t.tx_buf = vhub->transfer;
  t.len = offsetof(spi_cmd_t, data) + length; //
  //t.delay_usecs = 200;
  //t.cs_change_delay.unit = 0;
  //t.cs_change_delay.value = 100;

  pr_hex_mark(vhub->transfer, min(64, t.len), PR_WRITE);

  spi_message_add_tail(&t, &msg);

  UDCVDBG(vhub, "ast_vhub_irq spi write count: %d, hex:%04x, t.len:%d\n", length, cmd->length, t.len);

  if (cmd->reg.bit.read) {
    spi_sync(spi, &msg);
    flag = 0;
    wait_event_interruptible_timeout(wq, flag != 0, 800);
    flag = 0;
    return 1;
  }
  return !spi_sync(spi, &msg);
}

#define GPIO_CLIENT_GPIO_IRQ 17

#define VUSB_DEVICE_RESET    0x11
#define VUSB_DEVICE_PIPE_0   0x10
#define WRITE_CMD_WRITE 0x80
#define WRITE_CMD_READ  0x40

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

    memset(vhub->transfer, 0, 1024);

#define DEBUG_SPI_CECK
#define MAX_PRINT_COLUMN (u16)64
#define MAX_OUTPUT 512
#define HEADER offsetof(spi_cmd_t, data)

    // read
    if (vusb_read_buffer(vhub, vhub->transfer, 1024)) {

      spi_cmd_t* cmd = (spi_cmd_t*)vhub->transfer;
      // print the whole buffer
      pr_hex_mark(vhub->transfer, min(MAX_PRINT_COLUMN, cmd->length + HEADER), PR_READ);
    }
    else {
      UDCVDBG(vhub, "ast_vhub_irq spi error with read buffer:%d, value :%d \n", 0, irq);
    }


#ifdef _DEBUG_SPI_CECK
    //// read
    //if (vusb_read_buffer(vhub, vhub->transfer, 1024)) {
    //
    //  spi_cmd_t* cmd = (spi_cmd_t*)vhub->transfer;
    //// print the whole buffer
    //  pr_hex_mark(vhub->transfer, min(MAX_PRINT_COLUMN, cmd->length + HEADER), PR_READ);

    //  // create random write buffer
    //  u16 nlocCount = 0;
    //  get_random_bytes(&nlocCount, 2);
    //  u16 nCount = nlocCount % MAX_OUTPUT;
    //  //nCount = min(MAX_OUTPUT, nCount);
    //  get_random_bytes(&vhub->transfer[4], nCount);
    //  // write
    //  //UDCVDBG(vhub, "ast_vhub_irq spi write count: %d, hex:%04x, t.len:%d\n", nCount, nlocCount, 0);
    //  if (vusb_write_buffer(vhub, WRITE_CMD_WRITE | VUSB_DEVICE_PIPE_0, vhub->transfer, nCount)) {
    //    pr_hex_mark(vhub->transfer, min(MAX_PRINT_COLUMN, nCount + HEADER), PR_WRITE);

    //    u8 length = 1; u8 i;
    //    for (i = 0; i < length; i++)
    //    {
    //      // // blocking read
    //      // if (vusb_write_buffer(vhub, WRITE_CMD_READ | DEVICE_PIPE_0, vhub->transfer, nCount)) {
    //        // pr_hex_mark(vhub->transfer, min(MAX_PRINT_COLUMN, nCount + HEADER), PR_WRITE);
    //      // }
    //    }
    //  }
    //  else {
    //    pr_hex_mark(vhub->transfer, min(MAX_PRINT_COLUMN, nCount + HEADER), PR_WRITE | PR_ERROR);
    //  }
    //}
    //else {
    //  UDCVDBG(vhub, "ast_vhub_irq spi error with read buffer:%d, value*:%d \n", 0, irq);
    //}

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

  vhub->spi->master->cs_hold.unit = 0;
  vhub->spi->master->cs_hold.value = 700;
  //vhub->spi->master->cs_setup.unit = 0;
  //vhub->spi->master->cs_setup.value = 700;

  //vhub->spi->word_delay.unit = 1;
  //vhub->spi->word_delay.value = 800;

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

  u16 nlocCount;
  u32 nIndex = 0;
//#define DEBUG

#ifdef DEBUG
  for (nIndex = 0; nIndex < 60000; nIndex++)
  {
    get_random_bytes(&nlocCount, 1); 
    u16 nCount = 8; //nlocCount % MAX_OUTPUT;
    get_random_bytes(vhub->transfer, nCount);
    *((u16*)&vhub->transfer[4]) = nIndex;
    // blocking read
     if (!vusb_write_buffer(vhub, WRITE_CMD_READ | VUSB_DEVICE_PIPE_0, vhub->transfer, nCount)) {
      break;
     }
  }
#else
    memset(vhub->transfer, 0x22, 32);
    memset(vhub->transfer+4, 0x11, 3);
    memset(vhub->transfer+(32-4), 0x33, 4);
    for (nIndex = 1; nIndex < 45000; nIndex++)
    {
    //get_random_bytes(&nlocCount, 2);
    //u16 nCount = nlocCount % MAX_OUTPUT; // MAX_OUTPUT
    //get_random_bytes(vhub->transfer, nCount);
   //strcpy(vhub->transfer, "12345678901234567890");
    //*((u16*)&vhub->transfer[4]) = nIndex;
    // blocking read
    if (!vusb_write_buffer(vhub, WRITE_CMD_WRITE | VUSB_DEVICE_PIPE_0, vhub->transfer, 38)) {
      break;
    }
    mdelay(3);
  }
#endif

  dev_info(&spi->dev, "Succesfully written to client.\n");

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
