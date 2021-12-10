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

  UDCVDBG(ep->udc, "vusb_ep_set_halt, %sStall %s\n", stall ? "" : "Un", ep->name);
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

  UDCVDBG(ep->udc, "vusb_ep_enable name:%s, ep/idx: %d, addr: %x, maxp:%x\n", ep->name, ep->idx, desc->bEndpointAddress, maxp);

  // schedule to work
  schedule_work(&ep->wk_status);

  return 0;
}

void vusb_nuke(struct vusb_ep* ep, int status)
{
  struct vusb_req* req, * r;
  unsigned long flags;

  UDCVDBG(ep->udc, "vusb_nuke ep:%s\n", ep->name);

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
  UDCVDBG(ep->udc, "vusb_ep_disable %s\n", ep->name);

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

  //UDCVDBG(ep->udc, "vusb_alloc_request %s\n", ep->name);

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

  if (unlikely(!_req || !_req->complete || !_req->buf || !_ep))
    return -EINVAL;

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
  spin_unlock_irqrestore(&ep->lock, flags);

  schedule_work(&ep->wk_data);
  //UDCVDBG(ep->udc, "vusb_ep_queue, name: %s pipe: %d, cnt:%d\n", ep->name, ep->pipe, list_empty(&ep->queue));

  return 0;
}

static int vusb_ep_dequeue(struct usb_ep* _ep, struct usb_request* _req)
{
  struct vusb_req* t, * req = to_vusb_req(_req);
  struct vusb_ep* ep = to_vusb_ep(_ep);
  unsigned long flags;

  //UDCVDBG(ep->udc, "vusb_ep_dequeue %s\n", ep->name);

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
    if (ep->idx == ep_idx) {
      return ep;
    }
  }
  return 0;
}

static void vusb_ep_irq_data(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_irq_data);
  struct vusb_udc* udc = ep->udc;

  if (ep->dir == USB_DIR_BOTH) {
    
    u8 transfer[24];
    transfer[0] = REG_PIPE_SPFIFO; // setup register
    transfer[1] = ep->idx; // octopus pipe
    vusb_read_buffer(ep->udc, VUSB_REG_MAP_PIPE_GET, transfer, sizeof(struct usb_ctrlrequest));
    spi_cmd_t* cmd = (spi_cmd_t*)transfer;
    memmove(&ep->setup, cmd->data, sizeof(struct usb_ctrlrequest));
    // pr_hex_mark((void*)&setup, sizeof(struct usb_ctrlrequest), PRINTF_READ, ep->name);
    ep->ep0_dir = ep->setup.bRequestType & USB_DIR_IN? USB_DIR_IN:USB_DIR_OUT;
    vusb_handle_setup(ep);
  
  } else
  if (ep->dir == USB_DIR_OUT) {
   
    u8 transfer[64];
    // OUT data from the mcu...
    transfer[0] = REG_PIPE_FIFO; // write&read register
    transfer[1] = ep->idx; // octopus pipe
    vusb_read_buffer(ep->udc, VUSB_REG_MAP_PIPE_GET, transfer, 1 + 2 * sizeof(u8));
    spi_cmd_t* cmd = (spi_cmd_t*)transfer;
    UDCVDBG(ep->udc, "vusb_ep_irq_data, name: %s, pipe: %d, cnt: %d\n", ep->name, ep->idx, ep->maxpacket);
    pr_hex_mark_debug(transfer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ, ep->name, "irq_data");
    //vusb_do_data(ep->udc, ep);
  
  }

}

static void vusb_ep_data(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_data);

  if (ep->dir == USB_DIR_BOTH) {
    // process the data from the list
    while (vusb_do_data(ep->udc, ep));
  }
  if (ep->dir == USB_DIR_OUT) {
    //vusb_do_data(ep->udc, ep);
    struct vusb_req* req;
    void* buf;
    req = list_first_entry(&ep->queue, struct vusb_req, queue);
    buf = req->usb_req.buf + req->usb_req.actual;
    int length = req->usb_req.length - req->usb_req.actual;
    //pr_hex_mark_debug(buf, length, PRINTF_READ, ep->name, "ep_data");
    UDCVDBG(ep->udc, "vusb_ep_data out, length: %d, actual: %d, req:%x\n",
      req->usb_req.length, req->usb_req.actual, req);

  }
  if (ep->dir == USB_DIR_IN) {
    //UDCVDBG(ep->udc, "vusb_ep_data ep-in: %s, pipe: %d\n", ep->name, ep->idx);
    vusb_do_data(ep->udc, ep);
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

    //UDCVDBG(ep->udc, "vusb_ep_state enable name:%s, pipe: %x, maxp:%x, epaddr:%x\n",
    //  ep->name, ep->idx, ep->ep_usb.desc->wMaxPacketSize, ep->ep_usb.desc->bEndpointAddress);

    // ep type
    transfer[0] = REG_PIPE_TYPE; // reg
    transfer[1] = ep->idx; // pipe num
    transfer[2] = REG_EP_INTERRUPT;			// field to set
    vusb_write_buffer(ep->udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

    // max packetsize
    transfer[0] = REG_PIPE_MAXPKTS; // reg
    transfer[1] = ep->idx; // pipe num
    transfer[2] = ep->ep_usb.desc->wMaxPacketSize;			// field to set
    vusb_write_buffer(ep->udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

    // set the pipe endpoint address
    transfer[0] = REG_PIPE_EPADDRESS; // reg
    transfer[1] = ep->idx; // pipe num
    transfer[2] = ep->ep_usb.desc->bEndpointAddress;			// field to set
    vusb_write_buffer(ep->udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);
    // set the pipe interval
    transfer[0] = REG_PIPE_INTERVAL; // reg
    transfer[1] = ep->idx; // pipe num
    transfer[2] = ep->ep_usb.desc->bInterval;			// field to set
    vusb_write_buffer(ep->udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

    // set the pipe enable
    transfer[0] = REG_PIPE_ENABLED; // reg
    transfer[1] = ep->idx; // pipe num
    transfer[2] = 1;			  // field to set
    vusb_write_buffer(ep->udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

  } else
    if (ep->todo & DISABLE) {
      UDCVDBG(ep->udc, "vusb_ep_state disable name:%s, pipe: %x, attrib:%x, epaddr:%x\n",
        ep->name, ep->idx, ep->ep_usb.desc->bmAttributes, ep->ep_usb.desc->bEndpointAddress);

      // set the pipe enable
      transfer[0] = REG_PIPE_ENABLED; // reg
      transfer[1] = ep->idx; // pipe num
      transfer[2] = 0;			  // field to set
      vusb_write_buffer(ep->udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

    }    
  else
  if (ep->todo & STALL_EP) {

    spin_lock_irqsave(&ep->lock, flags);
    ep->todo &= ~STALL_EP;
    spin_unlock_irqrestore(&ep->lock, flags);
    UDCVDBG(ep->udc, "vusb_ep_state 'STALL' request, name: %s, pipe:%d\n", ep->name, ep->idx);
  }
  else {
    UDCVDBG(ep->udc, "vusb_ep_state '!STALL' request, name: %s, pipe: %d\n", ep->name, ep->idx);
    ep->halted = 0;
  }

}

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
    ep->port = 2; // port on the mcu
    ep->halted = 0;
    ep->maxpacket = 0;
    ep->ep_usb.name = ep->name;
    ep->ep_usb.ops = &vusb_ep_ops;
    INIT_WORK(&ep->wk_data, vusb_ep_data);
    INIT_WORK(&ep->wk_status, vusb_ep_status);
    INIT_WORK(&ep->wk_irq_data, vusb_ep_irq_data);
    usb_ep_set_maxpacket_limit(&ep->ep_usb, VUSB_EP_MAX_PACKET_LIMIT);
    ep->idx = idx + 2; //  _PIPIRQ2	BIT(2), Pipe 2
    ep->ep0_dir = 0; // set while reading setup packet

    if (idx == 0) { /* For EP0 */
 // ep->pipe = portnr - 1;
      ep->ep_usb.desc = &ep0_desc;
      ep->ep_usb.maxpacket = usb_endpoint_maxp(&ep0_desc);
      ep->ep_usb.caps.type_control = true;
      ep->ep_usb.caps.dir_in = true;
      ep->ep_usb.caps.dir_out = true;
      snprintf(ep->name, VUSB_EPNAME_SIZE, "ep%d", idx);
      ep->dir = USB_DIR_BOTH;
      continue;
    }

    if (idx == 1) { /* EP1 is OUT */
      ep->ep_usb.caps.dir_in = false;
      ep->ep_usb.caps.dir_out = true;
      ep->dir = USB_DIR_OUT;
      snprintf(ep->name, VUSB_EPNAME_SIZE, "ep%d-out", idx);
    }

    if (idx > 1) { /* EP2 & EP3 are IN */
      ep->ep_usb.caps.dir_in = true;
      ep->ep_usb.caps.dir_out = false;
      ep->dir = USB_DIR_IN;
      snprintf(ep->name, VUSB_EPNAME_SIZE, "ep%d-in", idx);
    }
    ep->ep_usb.caps.type_iso = false;
    ep->ep_usb.caps.type_int = true;
    ep->ep_usb.caps.type_bulk = true;

    list_add_tail(&ep->ep_usb.ep_list, &udc->gadget.ep_list);
  }

}
