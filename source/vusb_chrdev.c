// SPDX-License-Identifier: GPL-2.0+
/*
 * udc -- Driver for usbshield SoC "udc" USB gadget
 *
 * vusb_chrdev.c - Character device handling
 *
 * Copyright 2021 IBM Corporation
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
#include <linux/random.h>
#include "vusb_udc.h"

#define isdigit(c)	('0' <= (c) && (c) <= '9')

static long vusb_ioctl(struct file* file, unsigned int cmd, unsigned long arg);
static ssize_t vusb_chrdev_read(struct file* file, char __user* buf, size_t count, loff_t* offset);
static int vusb_chrdev_open(struct inode* inode, struct file* file);
static int vusb_release(struct inode* inode, struct file* file);
static ssize_t vusb_chrdev_write(struct file* file, const char __user* buf, size_t count, loff_t* offset);

const struct file_operations vusb_ops = {
  .owner = THIS_MODULE,
  .open = vusb_chrdev_open,
  .write = vusb_chrdev_write,
  .read = vusb_chrdev_read,
};

typedef struct vusb_send {
  uint8_t chr[4];
  uint16_t cmd;
  uint8_t cmdst[0x40];
  uint8_t port;
  uint8_t length;
}vusb_send_t;
const vusb_send_t vusb_send_tab[] = {
    { "r",   /*cmd*/ VUSB_SPI_CMD_WRITE | VUSB_DEVICE_RESET,    "VUSB_DEVICE_RESET",   /*port*/ 0, 1},
    { "a",   /*cmd*/ VUSB_SPI_CMD_WRITE | VUSB_DEVICE_HWATTACH, "VUSB_DEVICE_HWATTACH",/*port*/ 0, 1},
    { "d",   /*cmd*/ VUSB_SPI_CMD_WRITE | VUSB_DEVICE_HWDETACH, "VUSB_DEVICE_HWDETACH",/*port*/ 1, 1},
    { "+",   /*cmd*/ VUSB_SPI_CMD_WRITE | VUSB_DEVICE_ATTACH,   "VUSB_DEVICE_ATTACH",  /*port*/ 2, 1},
};

static int vusb_chrdev_open(struct inode* inode, struct file* file)
{
  struct vusb_udc* udc;
  //printk("vusb: Device open with %d possible cmds\n", sizeof(vusb_send_tab) / sizeof(vusb_send_t));
  udc = container_of(inode->i_cdev, struct vusb_udc, cdev);
  file->private_data = udc;
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

static ssize_t vusb_chrdev_read(struct file* file, char __user* buf, size_t count, loff_t* offset)
{
  printk("vusb: Device read");
  return 0;
}

static ssize_t vusb_chrdev_write(struct file* file, const char __user* buf, size_t count, loff_t* offset)
{
  struct vusb_udc* udc;
  udc = file->private_data;
  const size_t maxdatalen = 120;
  uint8_t *data = kmalloc(maxdatalen, GFP_KERNEL);
  memset(data, 0, maxdatalen);
  size_t ncopied;
  ncopied = copy_from_user(data, buf, maxdatalen);
  size_t i, j;
  for (i=0; i < count; i++) {
    for (j = 0; j < (sizeof(vusb_send_tab) / sizeof(vusb_send_t)); j++) {
      if (strncasecmp(&data[i], &vusb_send_tab[j].chr[0], 1) == 0)  {
        memset(udc->transfer, 0, VUSB_SPI_BUFFER_LENGTH);
        *udc->transfer = vusb_send_tab[j].port;
        vusb_write_buffer(udc, vusb_send_tab[j].cmd, udc->transfer, vusb_send_tab[j].length);
        msleep(200);
        break;
      }
    }
  }
  kfree(data);
  return count;
}

int vusb_chardev_uevent(struct device* dev, struct kobj_uevent_env* env)
{
  add_uevent_var(env, "DEVMODE=%#o", 0666);
  return 0;
}