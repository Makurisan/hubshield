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

#define to_vusb_req(r)	container_of((r), struct vusb_req, usb_req)
#define to_vusb_ep(e)	container_of((e), struct vusb_ep, ep_usb)
#define wk_data_to_vusb_ep(e) container_of((e), struct work_struct, wk_data)

static int vusb_ep_set_halt(struct usb_ep* _ep, int stall)
{
  struct vusb_ep* ep = to_vusb_ep(_ep);
  struct vusb_udc* udc = ep->udc;
  unsigned long flags;

  spin_lock_irqsave(&ep->lock, flags);

  ep->todo &= ~STALL_EP;
  if (stall)
    ep->todo |= STALL;
  else
    ep->todo |= UNSTALL;

  spin_unlock_irqrestore(&ep->lock, flags);

  dev_info(udc->dev, "vusb_ep_set_halt, %sStall %s\n", stall ? "" : "Un", ep->name);
  return 0;
}

static int vusb_ep_enable(struct usb_ep* _ep, const struct usb_endpoint_descriptor* desc)
{
  struct vusb_ep* ep = to_vusb_ep(_ep);
  struct vusb_udc* udc = ep->udc;

  unsigned int maxp = usb_endpoint_maxp(desc);
  unsigned long flags;

  spin_lock_irqsave(&ep->lock, flags);
  ep->ep_usb.desc = desc;
  ep->ep_usb.maxpacket = maxp;

  ep->todo &= ~ENABLE_EP;
  ep->todo |= ENABLE;
  spin_unlock_irqrestore(&ep->lock, flags);

  //dev_info(&ep->udc->spi->dev, "vusb_ep_enable name:%s, addr: %x, maxp:%x\n",
  //       _ep->name, desc->bEndpointAddress, maxp);

  // schedule to work
  schedule_work(&ep->wk_status);

  return 0;
}

void vusb_nuke(struct vusb_ep* ep, int status)
{
  struct vusb_req* req, * r;
  unsigned long flags;

  dev_info(&ep->udc->spi->dev, "vusb_nuke ep:%s\n", ep->name);

  spin_lock_irqsave(&ep->lock, flags);

  list_for_each_entry_safe(req, r, &ep->queue, queue) {
    list_del_init(&req->queue);

    spin_unlock_irqrestore(&ep->lock, flags);
    vusb_req_done(req, status);
    spin_lock_irqsave(&ep->lock, flags);
  }

  spin_unlock_irqrestore(&ep->lock, flags);
}

static int vusb_ep_disable(struct usb_ep* _ep)
{
  struct vusb_ep* ep = to_vusb_ep(_ep);
  struct vusb_udc* udc = ep->udc;
  unsigned long flags;

  vusb_nuke(ep, -ESHUTDOWN);

  spin_lock_irqsave(&ep->lock, flags);
  ep->ep_usb.desc = NULL;
  ep->todo &= ~ENABLE_EP;
  ep->todo |= DISABLE;
  spin_unlock_irqrestore(&ep->lock, flags);

  schedule_work(&ep->wk_status);
  //dev_info(ep->udc->dev, "vusb_ep_disable %s\n", ep->name);

  return 0;
}

static struct usb_request* vusb_alloc_request(struct usb_ep* _ep, gfp_t gfp_flags)
{
  struct vusb_ep* ep = to_vusb_ep(_ep);
  struct vusb_req* req;

  req = kzalloc(sizeof(*req), gfp_flags);
  if (!req)
    return NULL;

  req->ep = ep;

  dev_dbg(ep->udc->dev, "vusb_alloc_request %s\n", ep->name);

  return &req->usb_req;
}

static void vusb_free_request(struct usb_ep* _ep, struct usb_request* _req)
{
  kfree(to_vusb_req(_req));
}

static int vusb_ep_queue(struct usb_ep* _ep, struct usb_request* _req, gfp_t ignored)
{
  struct vusb_req* req = to_vusb_req(_req);
  struct vusb_ep* ep = to_vusb_ep(_ep);
  struct vusb_udc* udc = ep->udc;
  unsigned long flags;

  _req->status = -EINPROGRESS;
  _req->actual = 0;

#ifdef _DEBUG
  spin_lock_irqsave(&ep->lock, flags);
  void *buf = req->usb_req.buf + req->usb_req.actual;
  int length = req->usb_req.length - req->usb_req.actual;
  int psz = ep->ep_usb.maxpacket;
  length = min(length, psz);
  pr_hex_mark(buf, length, PRINTF_READ, ep->name);
  spin_unlock_irqrestore(&ep->lock, flags);
#endif // _DEBUG

  spin_lock_irqsave(&ep->lock, flags);
  list_add_tail(&req->queue, &ep->queue);
  schedule_work(&ep->wk_data);
  spin_unlock_irqrestore(&ep->lock, flags);
  //dev_info(udc->dev, "vusb_ep_queue, name: %s pipe: %d\n", ep->name, ep->pipe);

  return 0;
}

static int vusb_ep_dequeue(struct usb_ep* _ep, struct usb_request* _req)
{
  struct vusb_req* t, * req = to_vusb_req(_req);
  struct vusb_ep* ep = to_vusb_ep(_ep);
  unsigned long flags;

  dev_info(ep->udc->dev, "vusb_ep_dequeue %s\n", ep->name);

  spin_lock_irqsave(&ep->lock, flags);

  /* Pluck the descriptor from queue */
  list_for_each_entry(t, &ep->queue, queue)
    if (t == req) {
      list_del_init(&req->queue);
      break;
    }

  spin_unlock_irqrestore(&ep->lock, flags);

  if (t == req)
    vusb_req_done(req, -ECONNRESET);

  return 0;
}

static const struct usb_ep_ops vusb_ep_ops = {
  .enable = vusb_ep_enable,
  .disable = vusb_ep_disable,
  .alloc_request = vusb_alloc_request,
  .free_request = vusb_free_request,
  .queue = vusb_ep_queue,
  .dequeue = vusb_ep_dequeue,
  .set_halt = vusb_ep_set_halt,
};

/* Control endpoint configuration.*/
static const struct usb_endpoint_descriptor ep0_desc = {
  .bEndpointAddress = USB_DIR_OUT,
  .bmAttributes = USB_ENDPOINT_XFER_CONTROL,
  .wMaxPacketSize = cpu_to_le16(VUSB_EP_MAX_PACKET_LIMIT),
};

struct vusb_ep* vusb_get_ep(struct vusb_udc* udc, u8 ep_idx)
{
  int idx;

  for (idx = 0; idx < VUSB_MAX_EPS; idx++) {
    struct vusb_ep* ep = &udc->ep[idx];
    if (ep->pipe == ep_idx) {
      return ep;
    }
  }
  return 0;
}

static void vusb_ep_irq_data(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_irq_data);
  struct vusb_udc* udc = ep->udc;

  u8 transfer[24];
  transfer[0] = ep->port; // octopus port
  transfer[1] = ep->pipe; // octopus pipe
  if (vusb_read_buffer(ep->udc, VUSB_REG_PIPE_GET_DATA, transfer, 1 + sizeof(struct usb_ctrlrequest))) {
    spi_cmd_t* cmd = (spi_cmd_t*)transfer;
    struct vusb_ep* _ep = vusb_get_ep(ep->udc, (u8)cmd->data[0]);
    if (_ep == ep) {
      if (ep->ep_usb.caps.type_control) {
        struct usb_ctrlrequest setup;
        memmove(&setup, &cmd->data[sizeof(u8)], sizeof(struct usb_ctrlrequest));
        // pr_hex_mark((void*)&setup, sizeof(struct usb_ctrlrequest), PRINTF_READ, ep->name);
        vusb_handle_setup(ep->udc, ep, setup);
      }
      else {
        // OUT data from the mcu...
        UDCVDBG(ep->udc, "vusb_ep_irq_data, name: %s, pipe: %d\n", ep->name, ep->pipe);
        pr_hex_mark_debug(transfer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ, ep->name, "irq_data");
      }
    }
    else {
      UDCVDBG(ep->udc, "vusb_ep_irq_data: Error: %s, %s\n", ep->name, _ep->name);
    }
  }
}

static void vusb_ep_data(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_data);

  if (ep->ep_usb.caps.type_control) {
    // process the data from the list
    while (vusb_do_data(ep->udc, ep));
  }
  else
  if (ep->ep_usb.caps.dir_out) {
    dev_info(ep->udc->dev, "vusb_ep_data ep-out: %s, pipe: %d\n", ep->name, ep->pipe);
    //vusb_do_data(ep->udc, ep);
  }
  else
  if (ep->ep_usb.caps.dir_in) {
    dev_info(ep->udc->dev, "vusb_ep_data ep-in: %s, pipe: %d\n", ep->name, ep->pipe);
    //vusb_do_data(ep->udc, ep);
  }

}

// called over worker from enable/disable ....
static void vusb_ep_status(struct work_struct* work)
{
  unsigned long flags;
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_status);
  u8 transfer[24];

  if (ep->todo & ENABLE_EP) {
    spin_lock_irqsave(&ep->lock, flags);
    ep->todo &= ~ENABLE_EP;
    spin_unlock_irqrestore(&ep->lock, flags);

    UDCVDBG(ep->udc, "vusb_ep_state name:%s, pipe: %x, attrib:%x, epaddr:%x\n",
      ep->name, ep->pipe, ep->ep_usb.desc->bmAttributes, ep->ep_usb.desc->bEndpointAddress);

    // set the pipe endpoint address
    transfer[0] = REG_PIPE_EPADDRESS; // reg
    transfer[1] = ep->pipe; // pipe num
    transfer[2] = ep->ep_usb.desc->bEndpointAddress;			// field to set
    vusb_write_buffer(ep->udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);
    // set the pipe interval
    transfer[0] = REG_PIPE_INTERVAL; // reg
    transfer[1] = ep->pipe; // pipe num
    transfer[2] = ep->ep_usb.desc->bInterval;			// field to set
    vusb_write_buffer(ep->udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

    // set the pipe enable
    transfer[0] = REG_PIPE_ENABLED; // reg
    transfer[1] = ep->pipe; // pipe num
    transfer[2] = 1;			  // field to set
    vusb_write_buffer(ep->udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

  } else
    if (ep->todo & DISABLE) {
      UDCVDBG(ep->udc, "vusb_ep_state name:%s, pipe: %x, attrib:%x, epaddr:%x\n",
        ep->name, ep->pipe, ep->ep_usb.desc->bmAttributes, ep->ep_usb.desc->bEndpointAddress);

      // set the pipe enable
      transfer[0] = REG_PIPE_ENABLED; // reg
      transfer[1] = ep->pipe; // pipe num
      transfer[2] = 0;			  // field to set
      vusb_write_buffer(ep->udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

    }    
  else
  if (ep->todo & STALL_EP) {

    spin_lock_irqsave(&ep->lock, flags);
    ep->todo &= ~STALL_EP;
    spin_unlock_irqrestore(&ep->lock, flags);
    UDCVDBG(ep->udc, "vusb_ep_state 'STALL' request, name: %s, pipe:%d\n", ep->name, ep->pipe);
  }
  else {
    UDCVDBG(ep->udc, "vusb_ep_state '!STALL' request, name: %s, pipe: %d\n", ep->name, ep->pipe);
    ep->halted = 0;
  }

}


struct pipe_ch_cfg {
  int				ep_port;
  int				ep_num;
  int				ep_type;
  int				dir;
  int				n_fifo_slots;
  int				max_pkt_fs;
};

#define VUSB_CTRL		0x00
#define VUSB_ISOC		0x01
#define VUSB_BULK		0x02
#define VUSB_INTR		0x03

#define VUSB_OUT		0x00
#define VUSB_IN			0x01

static const struct pipe_ch_cfg pipe_defaults[] = {

  /* 
    
        ep_port       ep_type         n_fifo_slots    
  idx      |  n_pipe    |       dir       |    max_pkt_fs 
   |       |    |       |        |        |      |        */
  [0] = {  1,   2, VUSB_CTRL, VUSB_OUT,  32,    64, },
  [1] = {  1,   3, VUSB_INTR, VUSB_OUT,  32,    64, },
  [2] = {  1,   4, VUSB_INTR, VUSB_IN,  128,   512, },
  [3] = {  2,   5, VUSB_CTRL, VUSB_IN,  128,   512, },
  [4] = {  2,   6, VUSB_INTR, VUSB_OUT,  32,    64, },
  [5] = {  2,   7, VUSB_INTR, VUSB_IN,   32,    64, },
};

void vusb_eps_init(struct vusb_udc* udc)
{
  int idx;

  INIT_LIST_HEAD(&udc->gadget.ep_list);

  for (idx = 0; idx < VUSB_MAX_EPS; idx++) {
    struct vusb_ep* ep = &udc->ep[idx];

    spin_lock_init(&ep->lock);
    INIT_LIST_HEAD(&ep->queue);

    ep->udc = udc;
    ep->id = idx;
    ep->port = 1; // port on the mcu
    ep->halted = 0;
    ep->maxpacket = 0;
    ep->ep_usb.name = ep->name;
    ep->ep_usb.ops = &vusb_ep_ops;
    INIT_WORK(&ep->wk_data, vusb_ep_data);
    INIT_WORK(&ep->wk_status, vusb_ep_status);
    INIT_WORK(&ep->wk_irq_data, vusb_ep_irq_data);
    usb_ep_set_maxpacket_limit(&ep->ep_usb, VUSB_EP_MAX_PACKET_LIMIT);
    ep->pipe = idx + 2; //  _PIPIRQ2	BIT(2), Pipe 2

    if (idx == 0) { /* For EP0 */
 // ep->pipe = portnr - 1;
      ep->ep_usb.desc = &ep0_desc;
      ep->ep_usb.maxpacket = usb_endpoint_maxp(&ep0_desc);
      ep->ep_usb.caps.type_control = true;
      ep->ep_usb.caps.dir_in = true;
      ep->ep_usb.caps.dir_out = true;
      snprintf(ep->name, VUSB_EPNAME_SIZE, "ep%d", idx);
      continue;
    }

    if (idx == 1) { /* EP1 is OUT */
      ep->ep_usb.caps.dir_in = false;
      ep->ep_usb.caps.dir_out = true;
      snprintf(ep->name, VUSB_EPNAME_SIZE, "ep%d-out", idx);
    }

    if (idx > 1) { /* EP2 & EP3 are IN */
      ep->ep_usb.caps.dir_in = true;
      ep->ep_usb.caps.dir_out = false;
      snprintf(ep->name, VUSB_EPNAME_SIZE, "ep%d-in", idx);
    }
    ep->ep_usb.caps.type_iso = false;
    ep->ep_usb.caps.type_int = true;
    ep->ep_usb.caps.type_bulk = false;

    list_add_tail(&ep->ep_usb.ep_list, &udc->gadget.ep_list);
  }

}
