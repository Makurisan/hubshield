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


 // read the data from the MCU
int vusb_read_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length)
{
  struct spi_device* spi = udc->spi;
  struct spi_transfer	tr;
  struct spi_message	m;
  u8 cmd_reg;

  spi_message_init(&m);
  memset(&tr, 0, sizeof(tr));

  tr.rx_buf = buffer;
  tr.len = VUSB_SPI_HEADER;
  spi_message_add_tail(&tr, &m);
  // read the four bytes header
  if (!spi_sync(spi, &m))
  {
    spi_cmd_t* cmd = (spi_cmd_t*)buffer;
    cmd_reg = cmd->reg.val;
    // UDCVDBG(vhub, "mcu read length:%d\n", cmd->length);
    pr_hex_mark(buffer, VUSB_SPI_HEADER, PR_READ);
    // check data
    if (cmd->length)
    {
      memset(&tr, 0, sizeof(tr));
      spi_message_init(&m);
      // data
      tr.rx_buf = &buffer[offsetof(spi_cmd_t, data)];
      if (cmd->length < VUSB_SPI_BUFFER_LENGTH)
      {
        tr.len = cmd->length;
        spi_message_add_tail(&tr, &m);
        if (!spi_sync(spi, &m) && crc8(udc->crc_table, tr.rx_buf, cmd->length, 0) == cmd->crc8) {
          // set the reg back to the header, the other fields are correct  
          cmd->reg.val = cmd_reg;
          return cmd->length;
        }
        else {
          UDCVDBG(udc, "mcu read spi_sync error!\n");
        }
      }
      else {
        UDCVDBG(udc, "mcu read spi buffer exceeds maximal size length: %02x\n", cmd->length);
      }
    }
  }
  return 0;
}

int vusb_write_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length)
{
  struct spi_transfer t;
  struct spi_message msg;

  mutex_lock_interruptible(&udc->spi_read_mutex);

  memset(&t, 0, sizeof(t));
  spi_message_init(&msg);

  // header reg, length, crc
  spi_cmd_t* cmd = (spi_cmd_t*)udc->transfer;

  // overlapping copy
  memmove(cmd->data, buffer, length);

  // prepare the header
  cmd->reg.val = reg;
  // crc over data
  cmd->crc8 = crc8(udc->crc_table, cmd->data, length, 0);
  cmd->length = length;

  t.tx_buf = udc->transfer;
  t.len = cmd->length + VUSB_SPI_HEADER;
  t.delay_usecs = 0;
  t.cs_change_delay.unit = 0;
  t.cs_change_delay.value = 100;

  pr_hex_mark(udc->transfer, t.len, PR_WRITE);
  spi_message_add_tail(&t, &msg);

  int status = spi_sync(udc->spi, &msg);
  if (cmd->reg.bit.read) {
    wait_event_interruptible_timeout(udc->spi_read_queue,
      gpio_get_value(GPIO_DATRDY_IRQ_PIN), 800);
  }

  mutex_unlock(&udc->spi_read_mutex);

  return !status;
}
