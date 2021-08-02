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
      //trace_printk("irq/desc:%d, irqs/unhandled:%d, irq/count:%d\n",
      //  desc->irq_data.hwirq, desc->irqs_unhandled, desc->irq_count);
      //clear and read
      spi_cmd_t *cmd = (spi_cmd_t*)udc->spitransfer;
      cmd->length = 64;
      memset(udc->spitransfer, 0, cmd->length);
      if (vusb_read_buffer(udc, VUSB_SPI_CMD_READ, udc->spitransfer, cmd->length)) {
        //pr_hex_mark(udc->spitransfer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ);
        // IRQ reg data
        if (cmd->reg.bit.reg == VUSB_REG_IRQ_GET && cmd->length == sizeof(vusb_req_map_t)) {
          memmove(&udc->irq_map, udc->spitransfer+VUSB_SPI_HEADER, sizeof(vusb_req_map_t));
          //UDCVDBG(udc, "vusb_spi_dtrdy: VUSB_REG_GET_IRQDATA r1:%x, r2:%x\n",
          //  bswap32(udc->irq_map.PIPIRQ), bswap32(udc->irq_map.PIPIEN));
          //pr_hex_mark(udc->irq_data, cmd->length, PRINTF_READ);
          if ( (udc->irq_map.USBIRQ & udc->irq_map.USBIEN) ||
                 (udc->irq_map.PIPIRQ & udc->irq_map.PIPIEN)) {
            if (udc->thread_task &&
              udc->thread_task->state != TASK_RUNNING)
              wake_up_process(udc->thread_task);
          }
        }
        if (cmd->reg.bit.reg == VUSB_REG_PIPE_SETUP_GET && cmd->length == sizeof(struct usb_ctrlrequest)) {
          struct usb_ctrlrequest setup;
          memmove(&setup, udc->spitransfer + VUSB_SPI_HEADER, sizeof(struct usb_ctrlrequest));
          //pr_hex_mark(udc->spitransfer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ);
          vusb_handle_setup(udc, setup);
        }
      }
      wake_up_interruptible(&udc->spi_read_queue);
    }
  }
  return iret;
}

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
    //UDCVDBG(udc, "Mcu read length:%d\n", cmd->length);
    //pr_hex_mark(buffer, VUSB_SPI_HEADER, PR_READ);
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
        if (!spi_sync(spi, &m)) {
          if(crc8(udc->crc_table, tr.rx_buf, cmd->length, 0) == cmd->crc8) {
           // set the reg back to the header, the other fields are correct  
            cmd->reg.val = cmd_reg;
            return cmd->length;
          }
          else {
            UDCVDBG(udc, "mcu read crc8 %d error!\n", cmd->length);
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

int vusb_write_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length)
{
  struct spi_transfer t;
  struct spi_message msg;

  mutex_lock_interruptible(&udc->spi_read_mutex);

  memset(&t, 0, sizeof(t));
  spi_message_init(&msg);

  // header reg, length, crc
  spi_cmd_t* cmd = (spi_cmd_t*)udc->spitransfer;

  // overlapping copy and length automatically checked
  memmove(cmd->data, buffer, length);

  // prepare the header
  cmd->reg.val = reg;
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
  if (cmd->reg.bit.read) {
    wait_event_interruptible_timeout(udc->spi_read_queue,
      gpio_get_value(GPIO_DATRDY_IRQ_PIN), VUSB_SPI_DATRDY_TIMEOUT);
  }
  mutex_unlock(&udc->spi_read_mutex);

  return !status;
}
