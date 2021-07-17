// SPDX-License-Identifier: GPL-2.0+
/*
 * VUSB Device Controller driver for USB.
 *
 * Author: Manfred Kubica <ManfredKubica@web.de>
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/bitfield.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/prefetch.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/crc8.h>
#include <linux/irq.h>
#include "vusb_udc.h"

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
      case PRINTF_READ:
        if (k == 1) {
          if (length)
            printk(KERN_INFO " r [%s\n", hexline); // print line to console
          else
            printk(KERN_ERR " r [%s mcu read error\n", hexline); // print line to console
        }
        else
          printk(KERN_INFO "   %s\n", hexline); // print line to console
        break;
      case PRINTF_WRITE:
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
