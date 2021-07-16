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
#include <linux/fs.h>
#include <linux/irq.h>
#include "vhub.h"

DECLARE_CRC8_TABLE(vbus_crc_table);

#define PR_WRITE 1
#define PR_READ  2
#define PR_ERROR 4

#define WRITE_CMD_READ        0x40
#define VUSB_DEVICE_IRQ       0x1a

void pr_hex_mark(const char* mem, int count, int mark)
{
  char hexbyte[64];
  char headbyte[30];
  char hexline[512];

  memset(hexbyte, 0, sizeof(hexbyte));
  memset(hexline, 0, sizeof(hexline));

  u16 length = *(u16*)&mem[2];

  int i, k = 0;
  for (i = 0; i < count /*&& count < sizeof(hexline)*/; i++)
  {
    if (i == 3) {
    //  sprintf(hexbyte, "%02X] length: %d, count: %d", mem[i], (uint16_t)mem[2], count);
      if (length)
        snprintf(hexbyte, 64, "%02X] length: %04x / %d", mem[i], count, count);
      else
        sprintf(hexbyte, "%02X] ", mem[i]);
      if (count > 4) {
        snprintf(headbyte, 30, "\n   %02X %02X %02X %02X ", mem[0], mem[1], mem[2], mem[3]);
        strcat(hexbyte, headbyte);
      }
    }
    else {
      if ((i + 1) % 8 == 0)
        snprintf(hexbyte, 64, "%02X  ", mem[i]);
      else
        snprintf(hexbyte, 64, "%02X ", mem[i]);
    }
    strcat(hexline, hexbyte);

    // print line every 16 bytes or if this is the last for-loop
    if (((i + 1) % 24 == 0) && (i != 0) || (i + 1 == count)) {
      k++;
      switch (mark) {
      case PR_READ:
        if (k == 1) {
          if (length)
            printk(KERN_INFO " r [%s\n", hexline); // print line to console
          else
            printk(KERN_ERR " r [%s mcu read error\n", hexline); // print line to console
        }
        else
          printk(KERN_INFO "   %s\n", hexline); // print line to console
        break;
      case PR_WRITE:
        if (k == 1)
          printk(KERN_INFO "w [%s\n", hexline); // print line to console
        else {
          printk(KERN_INFO "   %s\n", hexline); // print line to console
        }
        break;
      default:
        break;
      }
      //syslog(LOG_INFO, "l%d: %s",k , hexline); // print line to syslog
      memset(hexline, 0, sizeof(hexline)); // clear hexline array
    }
  }
}

void ast_vhub_done(struct ast_vhub_ep* ep, struct ast_vhub_req* req,  int status)
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

struct usb_request* ast_vhub_alloc_request(struct usb_ep* u_ep,  gfp_t gfp_flags)
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

static int value= 0;

// read the data from the MCU
int vusb_read_buffer(struct ast_vhub* vhub, u8 reg, u8* buffer, u16 length)
{
  struct spi_device* spi = vhub->spi;
  struct spi_transfer	tr;
  struct spi_message	m;
  u8 cmd_reg;

  spi_message_init(&m);
  memset(&tr, 0, sizeof(tr));

  tr.rx_buf = buffer;
  tr.len = VHUB_SPI_HEADER;
  spi_message_add_tail(&tr, &m);
  // read the four bytes header
  if (!spi_sync(spi, &m))
  {
    spi_cmd_t* cmd = (spi_cmd_t*)buffer;
    cmd_reg = cmd->reg.val;
    // UDCVDBG(vhub, "mcu read length:%d\n", cmd->length);
    pr_hex_mark(buffer, VHUB_SPI_HEADER, PR_READ);
    // check data
    if (cmd->length)
    {
     memset(&tr, 0, sizeof(tr));
      spi_message_init(&m);
      // data
      tr.rx_buf = &buffer[offsetof(spi_cmd_t, data)];
      if (cmd->length < VHUB_SPI_BUFFER_LENGTH)
      {
        tr.len = cmd->length;
        spi_message_add_tail(&tr, &m);
        if (!spi_sync(spi, &m) && crc8(vbus_crc_table, tr.rx_buf, cmd->length, 0) == cmd->crc8) {
          // set the reg back to the header, the other fields are correct  
          cmd->reg.val = cmd_reg;
         return cmd->length;
        }
        else {
          UDCVDBG(vhub, "mcu read spi_sync error!\n");
        }
      }
      else {
        UDCVDBG(vhub, "mcu read spi buffer exceeds maximal size length: %02x\n", cmd->length);
      }
    }
  }
  return 0;
}

int vusb_write_buffer(struct ast_vhub* vhub, u8 reg, u8* buffer, u16 length)
{
  struct spi_transfer t;
  struct spi_message msg;

  mutex_lock_interruptible(&vhub->spi_read_mutex);

  memset(&t, 0, sizeof(t));
  spi_message_init(&msg);

  // header reg, length, crc
  spi_cmd_t* cmd = (spi_cmd_t*)vhub->transfer;

  // overlapping copy
  memmove(cmd->data, buffer, length);

  // prepare the header
  cmd->reg.val = reg;
  // crc over data
  cmd->crc8 = crc8(vbus_crc_table, cmd->data, length, 0);
  cmd->length = length;

  t.tx_buf = vhub->transfer;
  t.len = cmd->length + VHUB_SPI_HEADER;
  t.delay_usecs = 0;
  t.cs_change_delay.unit = 0;
  t.cs_change_delay.value = 100;

  pr_hex_mark(vhub->transfer, t.len, PR_WRITE);
  spi_message_add_tail(&t, &msg);
  
  int status = spi_sync(vhub->spi, &msg);
  if (cmd->reg.bit.read) {
    wait_event_interruptible_timeout(vhub->spi_read_queue,
            gpio_get_value(GPIO_DATRDY_IRQ_PIN), 800);
  }

  mutex_unlock(&vhub->spi_read_mutex);

  return !status;
}

static irqreturn_t ast_vhub_irq_primary_handler(int irq, void* dev_id)
{
  struct ast_vhub* vhub = dev_id;

/* Stale interrupt while tearing down */
  if (vhub->irq_datrdy == irq || vhub->irq_listen == irq)
    return IRQ_WAKE_THREAD;
  return IRQ_NONE;
}
static uint32_t irq_called = 0;

static struct workqueue_struct* irq_workerqueue;

struct work_data {
  struct work_struct work;
  struct ast_vhub* vhub;
  int irq; // actual irq
  int irqs_unhandled;
};


static irqreturn_t ast_vhub_irq_listen(int irq, void* dev_id)
{
  struct ast_vhub* vhub = dev_id;
  irqreturn_t iret = IRQ_HANDLED;

  struct irq_desc* desc = irq_to_desc(irq);
  struct irq_data* data = irq_desc_get_irq_data(desc);
  if (desc && data && desc->irq_data.hwirq == GPIO_LISTEN_IRQ_PIN)
  {
    struct irq_chip* chip = irq_desc_get_chip(desc);
    if (chip)
    {
      UDCDBG(vhub, "mcu ast write/read: VUSB_DEVICE_IRQ\n");
      trace_printk("irq/desc:%d, irqs/unhandled:%d, irq/count:%d\n",
        desc->irq_data.hwirq, desc->irqs_unhandled, desc->irq_count);
      vusb_write_buffer(vhub, WRITE_CMD_READ|VUSB_DEVICE_IRQ, vhub->transfer, 0);
    }
  }
  return iret;
}

static void irq_worker(struct work_struct* work)
{
  struct work_data* data = (struct work_data*)work;
  struct ast_vhub* vhub = data->vhub;

  trace_printk("irq/desc:%d, irqs/unhandled:%d, irq/count:%d\n",
    data->irq, data->irqs_unhandled, 0);
  //clear and read
  spi_cmd_t* cmd = (spi_cmd_t*)vhub->transfer;
  cmd->length = VHUB_SPI_BUFFER_LENGTH >> 1;
  memset(vhub->transfer, 0, cmd->length);
  if (vusb_read_buffer(vhub, WRITE_CMD_READ, vhub->transfer, cmd->length)) {
    cmd = (spi_cmd_t*)vhub->transfer;
    pr_hex_mark(vhub->transfer, cmd->length + VHUB_SPI_HEADER, PR_READ);
  }
  wake_up_interruptible(&vhub->spi_read_queue);
  kfree(data);
}

static irqreturn_t ast_vhub_irq_dtrdy(int irq, void* dev_id)
{
  struct ast_vhub* vhub = dev_id;
  irqreturn_t iret = IRQ_HANDLED;

  struct irq_desc* desc = irq_to_desc(irq);
  struct irq_data* data = irq_desc_get_irq_data(desc);
  if (desc && data && desc->irq_data.hwirq == GPIO_DATRDY_IRQ_PIN)
  {
    struct irq_chip* chip = irq_desc_get_chip(desc);
    if (chip)
    {
      chip->irq_ack(data);
      struct work_data* wd;
      wd = kmalloc(sizeof(struct work_data), GFP_KERNEL);
      wd->vhub = vhub;
      wd->irq = desc->irq_data.hwirq;
      // diagnostic
      wd->irqs_unhandled = desc->irqs_unhandled;
      INIT_WORK(&wd->work, irq_worker);
      queue_work(irq_workerqueue, &wd->work);
    }
  }
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

  if (!vhub) {
    return 0;
  }

  dev_info(&spi->dev, "Remove of v-hub.\n");

  /* disable the slave IRQ line */
  disable_irq(vhub->irq_listen);
  disable_irq(vhub->irq_datrdy);

  /* IRQ worker queue */
  flush_workqueue(irq_workerqueue);
  destroy_workqueue(irq_workerqueue);

  /* Remove devices */
  for (i = 0; i < vhub->max_ports; i++) {
    dev_info(&spi->dev, "Device remove: %s.\n", dev_name(vhub->ports[i].dev.port_dev));
    ast_vhub_del_dev(&vhub->ports[i].dev);
  }

  // remove the char device
  device_destroy(vhub->chardev_class, MKDEV(vhub->crdev_major, 1));
  class_destroy(vhub->chardev_class);

  //class_unregister(vhub->chardev_class);
  //dev_info(&spi->dev, "- 4 v-hub.\n");

  dev_t dev_id = MKDEV(vhub->crdev_major, 0);
  cdev_del(&vhub->cdev);
  unregister_chrdev_region(dev_id, VUSB_MAX_CHAR_DEVICES);

  dev_info(&spi->dev, "Char device from v-hub removed.\n");

  spin_lock_irqsave(&vhub->lock, flags);
  spin_unlock_irqrestore(&vhub->lock, flags);

  return 0;
}

extern const struct file_operations vusb_ops;
int vusbchardev_uevent(struct device* dev, struct kobj_uevent_env* env);

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

  // create worker queue for irq and irdtrdy
  irq_workerqueue = create_workqueue("irq_workerqueue");

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

  rc = of_property_read_u32(np, "endpoints", &vhub->max_epns);
  vhub->epns = devm_kcalloc(&spi->dev, vhub->max_epns,
    sizeof(*vhub->epns), GFP_KERNEL);
  if (!vhub->epns)
  {
    dev_err(&spi->dev, "Unable to allocate Hub endpoints.\n");
    return -ENOMEM;
  }
  dev_info(&spi->dev, "Hub device has %d eps.\n", vhub->max_epns);

  vhub->transfer = devm_kcalloc(&spi->dev, VHUB_SPI_BUFFER_LENGTH,
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

  if (spi->irq < 0)
  {
    dev_err(&vhub->spi->dev, "Irq missing in platform data");
    return -ENODEV;
  }

  dev_info(&vhub->spi->dev, "SPI irq is defined as :%d\n", vhub->spi->irq);

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

  /* INTERRUPT spi read queue */
  init_waitqueue_head(&vhub->spi_read_queue);
  mutex_init(&vhub->spi_read_mutex);

  /* GPIO for mcu chip reset */
  vhub->gpiod_reset = devm_gpiod_get(&vhub->spi->dev, "reset", GPIOD_OUT_HIGH);
  dev_info(&vhub->spi->dev, "Reset gpio is defined as gpio:%x\n", vhub->gpiod_reset);
  gpiod_set_value(vhub->gpiod_reset, 1);

  vhub->irq_datrdy = gpio_to_irq(GPIO_DATRDY_IRQ_PIN);
  dev_info(&vhub->spi->dev, "GPIO for mcu dtrdy hwirq %d is irq %d.\n", GPIO_DATRDY_IRQ_PIN, vhub->irq_datrdy);
  // set the irq handler: list with "cat /proc/interrupts"
  rc = devm_request_threaded_irq(&vhub->spi->dev, vhub->irq_datrdy,
    ast_vhub_irq_primary_handler, ast_vhub_irq_dtrdy, IRQF_TRIGGER_FALLING | IRQF_ONESHOT  | IRQF_SHARED | IRQF_NO_SUSPEND, "vusbdtrdy", vhub);
  if (rc)
  {
    dev_err(&vhub->spi->dev, "Failed to request dtrdy hwirq interrupt\n");
    rc = -ENOMEM;
    goto err;
  }

  vhub->irq_listen = gpio_to_irq(GPIO_LISTEN_IRQ_PIN);
  dev_info(&vhub->spi->dev, "GPIO for mcu listen hwirq %d is irq %d.\n", GPIO_LISTEN_IRQ_PIN, vhub->irq_listen);
  rc = devm_request_threaded_irq(&vhub->spi->dev, vhub->irq_listen, ast_vhub_irq_primary_handler,
       ast_vhub_irq_listen, IRQF_SHARED | IRQF_ONESHOT | IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING, "vusbirq", vhub);
  if (rc)
  {
    dev_err(&vhub->spi->dev, "Failed to request listen hwirq interrupt\n");
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

  // char device
  dev_t usrdev;
  alloc_chrdev_region(&usrdev, 0, VUSB_MAX_CHAR_DEVICES, "vusb");
  vhub->crdev_major = MAJOR(usrdev);
  cdev_init(&vhub->cdev, &vusb_ops);
 
  rc = cdev_add(&vhub->cdev, usrdev, VUSB_MAX_CHAR_DEVICES);
  if (rc < 0) {
    pr_warn("Couldn't cdev_add\n");
    goto err;
  }
  vhub->chardev_class = class_create(THIS_MODULE, "vusb");
  vhub->chardev_class->dev_uevent = vusbchardev_uevent;

  device_create(vhub->chardev_class, NULL, MKDEV(vhub->crdev_major, 1), NULL, "vusb-%d", 1);
 
  trace_printk("Succesfully initialized vhub.\n");
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