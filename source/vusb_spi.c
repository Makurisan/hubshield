// SPDX-License-Identifier: GPL-2.0+
/*
 * VUSB Device Controller driver for USB.
3d printing quadcopter *
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

#define WAIT_UNTIL_GPIO_ASSERTED	msecs_to_jiffies(100)

static int wakeup_flag = 0;
static ktime_t wait_endtime;

void vusb_spi_pipe_attach(struct vusb_udc* udc, u8 port)
{
  udc->spitransfer[0] = '1';
  udc->spitransfer[1] = udc->spitransfer[0];
  vusb_write_buffer(udc, VUSB_REG_PORT_ATTACH, udc->spitransfer, sizeof(u8)*2);
}

void vusb_spi_pipe_ack(struct vusb_udc* udc, u8 irq)
{
  udc->spitransfer[0] = REG_PIPEIRQ;
  *(u32*)&udc->spitransfer[1] = htonl(BIT(irq)); // take one bit
  vusb_write_buffer(udc, VUSB_REG_ACK, udc->spitransfer, sizeof(u8) + sizeof(u32));
}

int vusb_spi_clear_pipe_irq(struct vusb_udc* udc, u8 irq)
{
  udc->spitransfer[0] = REG_PIPEIRQ;
  *(u32*)&udc->spitransfer[1] = htonl(BIT(irq)); // take one bit
  if (vusb_write_buffer(udc, VUSB_REG_IRQ_CLEAR, udc->spitransfer, sizeof(u8) + sizeof(u32))) {
    udc->irq_map.PIPIRQ &= ~irq;
    return true;
  }
  return false;
}

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
      wait_endtime = ktime_get();
      //spin_lock_irq(&udc->lock);
      wakeup_flag = 1;
      wake_up_interruptible(&udc->spi_read_queue);
      //spin_unlock_irq(&udc->lock);
    }
  }
  return iret;

}

static int _internal_read_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length, u8 showpart)
{
  struct spi_device* spi = udc->spi;
  struct spi_transfer	tr;
  struct spi_message	m;
  u8 cmd_reg;

  spi_message_init(&m);
  memset(&tr, 0, sizeof(tr));

#ifndef READ_ONE
  tr.tx_buf = NULL;
  tr.rx_buf = buffer;
  tr.len = VUSB_SPI_BUFFER_LENGTH / 4;

  tr.delay_usecs = 0;
  tr.cs_change_delay.unit = 0;
  tr.cs_change_delay.value = 100;

  spi_message_add_tail(&tr, &m);
  // read the four bytes header
  if (!spi_sync(spi, &m))
  {
    spi_cmd_t* cmd = (spi_cmd_t*)buffer;
    if (crc8(udc->crc_table, cmd->data, cmd->length, 0) == cmd->crc8) {
      if (showpart)
        pr_hex_mark(buffer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ, "correct");
      //pr_hex_mark(udc->spitransfer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ, NULL);
      // set the reg back to the header, the other fields are correct  
      return cmd->length;
    }
    else {
      pr_hex_mark(udc->spitransfer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ, "crc8 error");
      //UDCVDBG(udc, "mcu read crc8 %d error!\n", cmd->length);
      return -1;
    }
  }
  return -5;
#else
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
    // check data length, call/return cmd must be identical
    if (cmd->length && reg == cmd_reg)
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
            if(showpart)
              pr_hex_mark(buffer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ, "correct");
            //pr_hex_mark(udc->spitransfer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ, NULL);
            // set the reg back to the header, the other fields are correct  
            cmd->reg.val = cmd_reg;
            return cmd->length;
          }
          else {
            pr_hex_mark(udc->spitransfer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ, "crc8 error");
            //UDCVDBG(udc, "mcu read crc8 %d error!\n", cmd->length);
            return -1;
          }
        }
        else {
          UDCVDBG(udc, "mcu read spi_sync error!\n");
          return -2;
        }
      }
      else {
        UDCVDBG(udc, "mcu read spi buffer exceeds maximal size length: %02x\n", cmd->length);
        pr_hex_mark(udc->spitransfer, VUSB_SPI_HEADER, PRINTF_READ, NULL);
        return -3;
      }
    }
    else {
      UDCVDBG(udc, "mcu read command header error: %02x\n", cmd->length);
      return -4;
    }
  }
  return -5; 
#endif
}

#define TRY_FAILED

static int _vusb_read_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length, u8 showpart);

int vusb_read_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length)
{
  int rc1=0, rc2=0, rc3=0;
  //mutex_lock_interruptible(&udc->spi_read_mutex);

#ifdef TRY_FAILED
  if ((rc1 = _vusb_read_buffer(udc, reg, buffer, length, 0)) <= 0) {
    //UDCVDBG(udc, "mcu first read error: %d, %d\n", rc1);
    pr_hex_mark(buffer, sizeof(u8) * 12, PRINTF_ERROR, NULL);
    if ((rc2 = _vusb_read_buffer(udc, reg, buffer, length, 1)) <= 0) {
      UDCVDBG(udc, "mcu second read error: %d, %d\n", rc1, rc2);
      if ((rc3 = _vusb_read_buffer(udc, reg, buffer, length, 1)) <= 0) {
        UDCVDBG(udc, "mcu third read error: %d, %d, %d\n", rc1, rc2, rc3);
        mutex_unlock(&udc->spi_read_mutex);
        return rc3;
      }
    }
  }
#else
  rc1 = _vusb_read_buffer(udc, reg, buffer, length));
#endif // DEBUG
  //mutex_unlock(&udc->spi_read_mutex);
  return rc1;
}
static int _vusb_write_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length);

static int _vusb_read_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length, u8 showpart)
{
  struct spi_transfer t;
  struct spi_message msg;
  int rc = -7;

  u8 spibuffer[2];
  spibuffer[0] = true;
  //_vusb_write_buffer(udc, VUSB_REG_READ_LOCK, spibuffer, sizeof(u8)*2);

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

  t.tx_buf = buffer;
  cmd->length = length;
  t.len = cmd->length + VUSB_SPI_HEADER;
  t.delay_usecs = 0;
  t.cs_change_delay.unit = 0;
  t.cs_change_delay.value = 100;

  //pr_hex_mark(udc->spitransfer, t.len, PRINTF_WRITE, NULL);
  spi_message_add_tail(&t, &msg);

  if (!spi_sync(udc->spi, &msg)) {
    ktime_t wait_starttime = ktime_get();
    u16 wait_time;
    wakeup_flag = 0;
    rc = wait_event_interruptible_timeout(udc->spi_read_queue, wakeup_flag, WAIT_UNTIL_GPIO_ASSERTED);
    wait_time = ktime_to_us(wait_endtime - wait_starttime);
    if (wait_time > 8000) {
      UDCVDBG(udc, "Mcu wait time(us): %d, rc:%d\n", wait_time, rc);
    }
    if (rc) {
      rc = _internal_read_buffer(udc, VUSB_SPI_CMD_READ | reg, udc->spitransfer, length, showpart);
    } else {
      //UDCVDBG(udc, "Mcu wait event error for spi read\n");
      memset(udc->spitransfer + VUSB_SPI_HEADER, 0, length );
      rc = -6;
    }
  }
  spibuffer[0] = false;
  //_vusb_write_buffer(udc, VUSB_REG_READ_LOCK, spibuffer, sizeof(u8)*2);

  return rc;
}


int vusb_write_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length)
{
  int rc;

  u8 spibuffer[2];
  spibuffer[0] = true;
  //_vusb_write_buffer(udc, VUSB_REG_READ_LOCK, spibuffer, sizeof(u8));

  rc = _vusb_write_buffer(udc, reg, buffer, length);
  
  spibuffer[0] = false;
  //_vusb_write_buffer(udc, VUSB_REG_READ_LOCK, spibuffer, sizeof(u8));

  return rc;
}

static int _vusb_write_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length)
{
  struct spi_transfer t;
  struct spi_message msg;

  memset(&t, 0, sizeof(t));
  spi_message_init(&msg);

  // header reg, length, crc
  spi_cmd_t* cmd = (spi_cmd_t*)udc->spiwritebuffer;

  // overlapping copy and length automatically checked
  memmove(cmd->data, buffer, length);

  // prepare the header
  cmd->reg.val = VUSB_SPI_CMD_WRITE | reg;
  // crc over data
  cmd->crc8 = crc8(udc->crc_table, cmd->data, length, 0);
  cmd->length = length;

  t.tx_buf = udc->spiwritebuffer;
  t.len = cmd->length + VUSB_SPI_HEADER;
  t.delay_usecs = 0;
  t.cs_change_delay.unit = 0;
  t.cs_change_delay.value = 100;

  //pr_hex_mark(udc->spitransfer, t.len, PRINTF_WRITE, NULL);
  spi_message_add_tail(&t, &msg);

  int status = spi_sync(udc->spi, &msg);
  if (status) {
    UDCVDBG(udc, "--> Mcu spi write error: %d\n", status);
  }
  return !status;
}
