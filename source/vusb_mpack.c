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
#include "mpack.h"

int vusb_mpack_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length)
{
  int16_t hResult = 0;

  mpack_reader_t reader;
  //mpack_tree_t  tree = { 0 };
  size_t _length = 12;
  uint32_t count = 0;

#define OCTOPUS_MAX_SEND_ELEMENTS 5

  mpack_reader_init_data(&reader, buffer, _length);
  if (!mpack_expect_map_max_or_nil(&reader, OCTOPUS_MAX_SEND_ELEMENTS, &count)) {
    mpack_reader_destroy(&reader);
    return hResult;
  }
  mpack_reader_destroy(&reader);
  return hResult;
}


