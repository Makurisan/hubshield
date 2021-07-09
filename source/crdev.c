// SPDX-License-Identifier: GPL-2.0+
/*
 * vhub -- Driver for usbshield SoC "vHub" USB gadget
 *
 * crdev.c - Character device handling
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
#include <linux/regmap.h>
#include <linux/dma-mapping.h>

#include "vhub.h"

#define VUSB_DEVICE_PING      0x11
#define VUSB_DEVICE_RESET     0x12
#define VUSB_DEVICE_ATTACH    0x13
#define VUSB_DEVICE_DETACH    0x14
#define VUSB_DEVICE_MEMORY    0x15
#define VUSB_DEVICE_HWATTACH  0x16
#define VUSB_DEVICE_DATA      0x17
#define VUSB_DEVICE_HEADER    0x18 // the header with the length field
#define VUSB_DEVICE_CLEARSCRN 0x19

#define WRITE_CMD_WRITE 0x80
#define WRITE_CMD_READ  0x40

typedef struct vusb_send {
  uint8_t chr[4]; 
  uint16_t cmd;
  uint8_t cmdst[0x40];
  uint8_t port;
  uint8_t length;
}vusb_send_t;

static long vusb_ioctl(struct file* file, unsigned int cmd, unsigned long arg);
static ssize_t vusb_read(struct file* file, char __user* buf, size_t count, loff_t* offset);
static int vusb_open(struct inode* inode, struct file* file);
static int vusb_release(struct inode* inode, struct file* file);
static ssize_t vusb_write(struct file* file, const char __user* buf, size_t count, loff_t* offset);

int vusb_write_buffer(struct ast_vhub* vhub, u8 reg, u8* buffer, u16 length);

const struct file_operations vusb_ops = {
  .owner = THIS_MODULE,
  .open = vusb_open,
  .write = vusb_write,
  .read = vusb_read,
};

const vusb_send_t vusb_send_tab[] = {
    { "p",   /*cmd*/ WRITE_CMD_WRITE | VUSB_DEVICE_PING,     "VUSB_DEVICE_PING",    /*port*/0,  4},
    { "a",   /*cmd*/ WRITE_CMD_WRITE | VUSB_DEVICE_ATTACH,   "VUSB_DEVICE_ATTACH",  /*port*/1,  4},
    { "m",   /*cmd*/ WRITE_CMD_WRITE | VUSB_DEVICE_MEMORY,   "VUSB_DEVICE_MEMORY",  /*port*/1,  8},
    { "b",   /*cmd*/ WRITE_CMD_WRITE | VUSB_DEVICE_HWATTACH, "VUSB_DEVICE_HWATTACH",/*hub*/ 0,  4},
    { "r",   /*cmd*/ WRITE_CMD_WRITE | VUSB_DEVICE_RESET,    "VUSB_DEVICE_RESET",   /*hub*/ 0,  4},
    { "d",   /*cmd*/ WRITE_CMD_WRITE | VUSB_DEVICE_DETACH,   "VUSB_DEVICE_DETACH",  /*port*/1,  4},
    { "c",   /*cmd*/ WRITE_CMD_READ | VUSB_DEVICE_CLEARSCRN, "VUSB_DEVICE_CLEARSCR",/*port*/0, 12},
    { "i",   /*cmd*/ WRITE_CMD_READ | VUSB_DEVICE_DATA,      "VUSB_DEVICE_DATA",    /*port*/0,  8},
};

static int vusb_open(struct inode* inode, struct file* file)
{
  struct ast_vhub* vhub;
  //printk("vusb: Device open with %d possible cmds\n", sizeof(vusb_send_tab) / sizeof(vusb_send_t));
  vhub = container_of(inode->i_cdev, struct ast_vhub, cdev);
  file->private_data = vhub;
  return 0;
}

static int vusb_release(struct inode* inode, struct file* file)
{
  return 0;
}

static long vusb_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
  printk("vusb: Device ioctl");
  return 0;
}

static ssize_t vusb_read(struct file* file, char __user* buf, size_t count, loff_t* offset)
{
  printk("vusb: Device read");
  return 0;
}

static ssize_t vusb_write(struct file* file, const char __user* buf, size_t count, loff_t* offset)
{
  struct ast_vhub* vhub;
  vhub = file->private_data;
  const size_t maxdatalen = 120;
  uint8_t *data = kmalloc(maxdatalen, GFP_KERNEL);
  memset(data, 0, maxdatalen);
  size_t ncopied;
  ncopied = copy_from_user(data, buf, maxdatalen);

  printk("cmd: %s", data);
  size_t i, j;
  for (i=0; i < count; i++) {
    for (j = 0; j < (sizeof(vusb_send_tab) / sizeof(vusb_send_t)); j++) {
      if (strncasecmp(&data[i], &vusb_send_tab[j].chr[0], 1) == 0)  {
        printk("char device write: %s, idx:%d", vusb_send_tab[j].cmdst, i);
        size_t k, l=5;
        for (k = 0; k < 32; k++)
         vhub->transfer[k]= ++l;
        //memset(vhub->transfer, 0x2, 32);
        vusb_write_buffer(vhub, vusb_send_tab[j].cmd, vhub->transfer, vusb_send_tab[j].length);
        msleep(400);
        break;
      }
    }
  }
  kfree(data);
  return count;
}

int vusbchardev_uevent(struct device* dev, struct kobj_uevent_env* env)
{
  add_uevent_var(env, "DEVMODE=%#o", 0666);
  return 0;
}
