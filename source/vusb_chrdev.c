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
    { "r",   /*cmd*/ VUSB_REG_RESET,      "REG_RESET",   /*port*/ 0, 1},
    { "a",   /*cmd*/ VUSB_REG_HWATTACH,   "REG_HWATTACH",/*port*/ 0, 1},
    { "d",   /*cmd*/ VUSB_REG_HWDETACH,   "REG_HWDETACH",/*port*/ 1, 1},
    { "+",  /*cmd*/ VUSB_REG_PORT_ATTACH, "REG_PORT_ATTACH",  /*port*/ 0, 2},
    { "-",  /*cmd*/ VUSB_REG_PORT_DETACH, "REG_PORT_DETACH",  /*port*/ 0, 2},
   // debug 
    { "p",   /*cmd*/ VUSB_REG_MEMORY,   "REG_MEMORY",  /*port*/ 0, 1},
    { "s",   /*cmd*/ VUSB_REG_PRINTF,   "REG_PRINTF",  /*port*/ 0, 1},
    { "t",   /*cmd*/ VUSB_REG_PRINTF1,  "REG_PRINTF",  /*port*/ 0, 1},
    { "u",   /*cmd*/ VUSB_REG_PRINTF2,  "REG_PRINTF",  /*port*/ 0, 1},
    { "v",   /*cmd*/ VUSB_REG_PRINTF3,  "REG_PRINTF",  /*port*/ 0, 1},
    { "w",   /*cmd*/ VUSB_REG_PRINTF4,  "REG_PRINTF",  /*port*/ 0, 1},
    { "x",   /*cmd*/ VUSB_REG_PRINTF99, "REG_PRINTF",  /*port*/ 0, 1},
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
      if (data[i] == 'x') {
        gpiod_set_value(udc->mcu_gpreset, 0);
        msleep_interruptible(100);
        gpiod_set_value(udc->mcu_gpreset, 1);
        break;
      }
      if (strncasecmp(&data[i], &vusb_send_tab[j].chr[0], 1) == 0)  {
        memset(udc->transfer, 0, VUSB_SPI_BUFFER_LENGTH);
        udc->transfer[0] = vusb_send_tab[j].port;
        if (vusb_send_tab[j].length > 1) {
          udc->transfer[1] = '1';
          if (isdigit(data[i + 1]))
            udc->transfer[1] = data[i+1];
        }
        vusb_write_buffer(udc, vusb_send_tab[j].cmd, udc->transfer, vusb_send_tab[j].length);
        //msleep_interruptible(100);
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
