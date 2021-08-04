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

#define WAIT_UNTIL_GPIO_ASSERTED	msecs_to_jiffies(700)

static int wakeup_flag = 0;

irqreturn_t vusb_spi_dtrdy(int irq, void* dev_id)
{
  struct vusb_udc* udc = dev_id;
  irqreturn_t iret = IRQ_HANDLED;

  struct irq_desc* desc = irq_to_desc(irq);
  struct irq_data* data = irq_desc_get_irq_data(desc);
  if (desc && data && desc->irq_data.hwirq == GPIO_DATRDY_IRQ_PIN)
  {
    struct irq_chip* chip = irq_desc_get_chip(desc);
    if (chip)
    {
      //spin_lock_irq(&udc->lock);
      wakeup_flag = 1;
      wake_up_interruptible(&udc->spi_read_queue);
      //spin_unlock_irq(&udc->lock);
    }
  }
  return iret;

}

static int _internal_read_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length)
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
    ////UDCVDBG(udc, "Mcu read length:%d\n", cmd->length);
    //pr_hex_mark(buffer, VUSB_SPI_HEADER, PRINTF_READ);
    // check data
    if (cmd->length)
    {
      memset(&tr, 0, sizeof(tr));
      spi_message_init(&m);
      // data
      tr.rx_buf = &buffer[offsetof(spi_cmd_t, data)];
      if (cmd->length < VUSB_SPI_BUFFER_LENGTH)
      {
        //UDCVDBG(udc, "Mcu read length:%d\n", cmd->length);
        tr.len = cmd->length;
        spi_message_add_tail(&tr, &m);
        if (!spi_sync(spi, &m)) {
          if (crc8(udc->crc_table, tr.rx_buf, cmd->length, 0) == cmd->crc8) {
            //pr_hex_mark(buffer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ);
            //pr_hex_mark(udc->spitransfer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ);
            // set the reg back to the header, the other fields are correct  
            cmd->reg.val = cmd_reg;
            return cmd->length;
          }
          else {
            pr_hex_mark(udc->spitransfer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ);
            //UDCVDBG(udc, "mcu read crc8 %d error!\n", cmd->length);
          }
        }
        else {
          UDCVDBG(udc, "mcu read spi_sync error!\n");
        }
      }
      else {
        UDCVDBG(udc, "mcu read spi buffer exceeds maximal size length: %02x\n", cmd->length);
      }
    }
    else {
      UDCVDBG(udc, "mcu read spi no length returned: %02x\n", cmd->length);
    }
  }
  return 0;
}

int vusb_read_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length)
{
  struct spi_transfer t;
  struct spi_message msg;
  int rc = 0;

  mutex_lock_interruptible(&udc->spi_read_mutex);

  memset(&t, 0, sizeof(t));
  spi_message_init(&msg);

  // header reg, length, crc
  spi_cmd_t* cmd = (spi_cmd_t*)udc->spitransfer;
  // overlapping copy and length automatically checked
  memmove(cmd->data, buffer, length);
  // prepare the header
  cmd->reg.val = VUSB_SPI_CMD_READ | reg;
  // crc over data
  cmd->crc8 = crc8(udc->crc_table, cmd->data, length, 0);
  cmd->length = length;

  t.tx_buf = buffer;
  t.len = cmd->length + VUSB_SPI_HEADER;
  t.delay_usecs = 0;
  t.cs_change_delay.unit = 0;
  t.cs_change_delay.value = 100;

  //pr_hex_mark(udc->spitransfer, t.len, PRINTF_WRITE);
  spi_message_add_tail(&t, &msg);

  if (!spi_sync(udc->spi, &msg)) {
    wakeup_flag = 0;
    rc = wait_event_interruptible_timeout(udc->spi_read_queue, wakeup_flag, WAIT_UNTIL_GPIO_ASSERTED);
    if (rc) {
      rc = _internal_read_buffer(udc, VUSB_SPI_CMD_READ | reg, udc->spitransfer, length);
    } else {
      UDCVDBG(udc, "*Mcu error wait vusb_read_buffer\n");
    }
  }
  mutex_unlock(&udc->spi_read_mutex);

  return rc;
}


int vusb_write_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length)
{
  struct spi_transfer t;
  struct spi_message msg;

  memset(&t, 0, sizeof(t));
  spi_message_init(&msg);

  // header reg, length, crc
  spi_cmd_t* cmd = (spi_cmd_t*)udc->spitransfer;

  // overlapping copy and length automatically checked
  memmove(cmd->data, buffer, length);

  // prepare the header
  cmd->reg.val = VUSB_SPI_CMD_WRITE | reg;
  // crc over data
  cmd->crc8 = crc8(udc->crc_table, cmd->data, length, 0);
  cmd->length = length;

  t.tx_buf = udc->spitransfer;
  t.len = cmd->length + VUSB_SPI_HEADER;
  t.delay_usecs = 0;
  t.cs_change_delay.unit = 0;
  t.cs_change_delay.value = 100;

  //pr_hex_mark(udc->spitransfer, t.len, PRINTF_WRITE);
  spi_message_add_tail(&t, &msg);

  int status = spi_sync(udc->spi, &msg);

  return !status;
}
