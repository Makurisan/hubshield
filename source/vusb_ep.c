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

  ep->eptype = desc->bmAttributes;
  ep->maxpacket = usb_endpoint_maxp(desc);

  UDCVDBG(ep->udc, "vusb_ep_enable name:%s, ep/idx: %d, addr: %x, maxp:%d, bmattrib: %d\n", ep->name, ep->idx, 
        desc->bEndpointAddress, maxp, desc->bmAttributes);

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
  void* buf = req->usb_req.buf + req->usb_req.actual;
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
  struct vusb_port_dev* d = &udc->ports[0/*port*/].dev;

  int idx;
  for (idx = 0; idx < VUSB_MAX_EPS; idx++) {
    struct vusb_ep* ep = &d->ep[idx];
    if (ep->idx == ep_idx) {
      return ep;
    }
  }
  return 0;
}

static void vusb_ep_irq_data(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_irq_data);

  if (ep->dir == USB_DIR_BOTH) {
    u8 transfer[24];
    transfer[0] = REG_PIPE_SPFIFO; // setup register
    transfer[1] = ep->idx; // octopus pipe
    vusb_read_buffer(ep->udc, VUSB_REG_MAP_PIPE_GET, transfer, sizeof(struct usb_ctrlrequest));
    spi_cmd_t* cmd = (spi_cmd_t*)transfer;
    memmove(&ep->setup, cmd->data, sizeof(struct usb_ctrlrequest));
    // pr_hex_mark((void*)&setup, sizeof(struct usb_ctrlrequest), PRINTF_READ, ep->name);
    //UDCVDBG(ep->udc, "vusb_ep_irq_data - ctrl, name: %s, pipe: %d, cnt: %d\n", ep->name, ep->idx, ep->maxpacket);
    ep->ep0_dir = ep->setup.bRequestType & USB_DIR_IN? USB_DIR_IN:USB_DIR_OUT;
    vusb_handle_setup(ep);
  } else
  if (ep->dir == USB_DIR_OUT) {
    //UDCVDBG(ep->udc, "vusb_ep_irq_data, name: %s, pipe: %d, cnt: %d\n", ep->name, ep->idx, ep->maxpacket);
    //pr_hex_mark_debug(transfer, cmd->length + VUSB_SPI_HEADER, PRINTF_READ, ep->name, "irq_data");
    // read one record from the mcu
    vusb_do_data(ep->udc, ep);
  }

}

static void vusb_ep_data(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_data);
  unsigned long flags;

  if (ep->dir == USB_DIR_BOTH) {
    // process all the control data from the list
    while(vusb_do_data(ep->udc, ep));
  } else
  if (ep->dir == USB_DIR_OUT) {
    //UDCVDBG(ep->udc, "vusb_ep_data - USB_DIR_OUT, name: %s, ep/idx: %d\n",   ep->name, ep->idx);
    // prep for the next read...
  } else
  if (ep->dir == USB_DIR_IN) {
    //UDCVDBG(ep->udc, "vusb_ep_data - USB_DIR_IN: %s, pipe: %d\n", ep->name, ep->idx);
    while (vusb_do_data(ep->udc, ep));
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

    // ep name
    transfer[0] = REG_PIPE_NAME; // reg
    transfer[1] = ep->idx; // pipe num
    memcpy(&transfer[2], ep->name, sizeof(ep->name));			// field to set
    vusb_write_buffer(ep->udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8)*2+sizeof(ep->name));

    // ep type
    transfer[0] = REG_PIPE_TYPE; // reg
    transfer[1] = ep->idx; // pipe num
    transfer[2] = ep->ep_usb.desc->bmAttributes;			// field to set
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


/*
  This function predefines and reserve the endpoint pipes on the mcu
*/
static void vusb_configure_pipe(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_udc_work);
  struct vusb_udc* udc = ep->udc;
  u8 transfer[10];

  // set port on control pipe
  transfer[0] = REG_PIPE_PORT; // reg
  transfer[1] = ep->idx; // pipe num
  transfer[2] = ep->port; // port
  vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

  UDCVDBG(udc, "- port: %d with  pipe/id: %d ep/name: %s\n", ep->port, ep->idx, ep->name);

  // set the pipe type
  transfer[0] = REG_PIPE_TYPE; // reg
  transfer[1] = ep->idx; // pipe num
  transfer[2] = ep->ep_usb.caps.type_control ? REG_EP_CONTROL : REG_EP_INTERRUPT; // field to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

  // set the pipe type
  transfer[0] = REG_PIPE_MAXPKTS; // reg
  transfer[1] = ep->idx; // pipe num
  transfer[2] = 0x40;			// field to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

}


// the function is called with the control pipe of the gadget
static void vusb_port_start(struct work_struct* work)
{
  struct vusb_port_dev* d = container_of(work, struct vusb_port_dev, wk_start);
  struct vusb_ep* ep0 = &d->ep[0];
  struct vusb_udc* udc = ep0->udc;

  u8 transfer[24];

  // control pipe

  // define the maxpacketsize
  transfer[0] = REG_PIPE_MAXPKTS; // reg
  transfer[1] = ep0->idx; // pipe num
  transfer[2] = 0x40;			// field to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

  // register the control ep0 on the port in the hub
  transfer[0] = REG_PIPE_PORT; // reg
  transfer[1] = ep0->idx; // pipe num
  transfer[2] = ep0->port; // port
  vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

  // set enable on control pipe
  transfer[0] = REG_PIPE_ENABLED; // reg
  transfer[1] = ep0->idx; // pipe num
  transfer[2] = 1;			// value to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);
  //UDCVDBG(udc, "  - port: %d with  pipe/id: %d ep0/name: %s\n", ep0->port, ep0->idx, ep0->name);

  // port setting

  // remote or local port
  transfer[0] = PORT_REG_DEVTYPE; // reg
  transfer[1] = ep0->port; // port
  transfer[2] = VUSB_PORT_DEVICE_REMOTE; // field to set; activate remote or local device
  vusb_write_buffer(udc, VUSB_REG_MAP_PORT_SET, transfer, sizeof(u8) * 3);

  // set enable on port
  transfer[0] = PORT_REG_ENABLED; // reg
  transfer[1] = ep0->port; // port
  transfer[2] = 1;			// value to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PORT_SET, transfer, sizeof(u8) * 3);
  //UDCVDBG(udc, "  - Pipe: %d on port: %d is enabled name: %s\n", ep0->pipe, ep0->port, ep0->name);

  //UDCVDBG(udc, "Hub port %d with ctrl/pipe: %s is now in remote stage and enabled", ep0->port, ep0->name);
  UDCVDBG(udc, "Hub gadget vusb_udc_start started\n");

}


static void vusb_port_stop(struct work_struct* work)
{
  struct vusb_port_dev* d = container_of(work, struct vusb_port_dev, wk_stop);
  struct vusb_ep* ep0 = &d->ep[0];
  struct vusb_udc* udc = ep0->udc;
  //UDCVDBG(udc, "Port %d will be detached", ep->port);

  u8 transfer[24];
  // set disable on port
  transfer[0] = PORT_REG_ENABLED; // reg
  transfer[1] = ep0->port; // port: 2
  transfer[2] = 0;			// value to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PORT_SET, transfer, sizeof(u8) * 3);

}

static int vusb_wakeup(struct usb_gadget* gadget)
{
  struct vusb_ep* ep = ep_usb_to_vusb_ep(gadget->ep0);
  unsigned long flags;
  int ret = -EINVAL;
  dev_info(&ep->udc->spi->dev, "Hub gadget vusb_wakeup.\n");
  return ret;
}

static struct usb_ep* vusb_match_ep(struct usb_gadget* gadget,
  struct usb_endpoint_descriptor* desc,
  struct usb_ss_ep_comp_descriptor* ep_comp)
{
  struct usb_ep* _ep;
  struct vusb_ep* ep;

  //UDCVDBG(udc, "Hub vusb_match_ep \n");

  /* Look at endpoints until an unclaimed one looks usable */
  list_for_each_entry(_ep, &gadget->ep_list, ep_list) {
    if (usb_gadget_ep_match_desc(gadget, _ep, desc, ep_comp))
      goto found_ep;
  }
  /* Fail */
  return NULL;

found_ep:

  // schedule the ep
  ep = ep_usb_to_vusb_ep(_ep);
  INIT_WORK(&ep->wk_udc_work, vusb_configure_pipe);
  schedule_work(&ep->wk_udc_work);

  //UDCVDBG(ep->udc, "Hub vusb_match_ep ep0:%s\n", ep->name);

  return _ep;

}

static int vusb_udc_start(struct usb_gadget* gadget, struct usb_gadget_driver* driver)
{
  struct vusb_ep* ep = ep_usb_to_vusb_ep(gadget->ep0);
  struct vusb_udc* udc = ep->udc;
  struct vusb_port_dev* d = &udc->ports[ep->dev_idx].dev;

  unsigned long flags;
  spin_lock_irqsave(&udc->lock, flags);
  /* hook up the driver */
  driver->driver.bus = NULL;
  udc->driver = driver;
  d->gadget.speed = USB_SPEED_FULL;

  d->gadget.is_selfpowered = udc->is_selfpowered;
  udc->remote_wkp = 0;
  udc->softconnect = true;
  udc->todo |= UDC_START;
  spin_unlock_irqrestore(&udc->lock, flags);

  schedule_work(&d->wk_start);

  return 0;
}

static void vusb_dev_nuke(struct vusb_udc* udc, int status)
{
  unsigned int i;

  //for (i = 0; i < VUSB_MAX_EPS; i++) {
  //  if (udc->ep[i].ep_usb.caps.dir_in) {
  //    vusb_nuke(&udc->ep[i], status);
  //  }
  //}
}

static int vusb_udc_stop(struct usb_gadget* gadget)
{
  struct vusb_ep* ep = ep_usb_to_vusb_ep(gadget->ep0);
  struct vusb_udc* udc = ep->udc;
  struct vusb_port_dev* d = &udc->ports[ep->dev_idx].dev;
  unsigned long flags;

  spin_lock_irqsave(&udc->lock, flags);
  udc->is_selfpowered = d->gadget.is_selfpowered;
  d->gadget.speed = USB_SPEED_UNKNOWN;
  d->driver = NULL;
  udc->softconnect = false;
  udc->todo |= UDC_START;
  spin_unlock_irqrestore(&udc->lock, flags);

  //vusb_dev_nuke(udc, -ESHUTDOWN);
  //flush_workqueue(udc->irq_work_mcu);
  UDCVDBG(udc, "Hub gadget vusb_udc_stop\n");

#define DEBUG
#ifdef DEBUG
  schedule_work(&d->wk_stop);
#else
  u8 transfer[24];
  // set disable on port
  transfer[0] = PORT_REG_ENABLED; // reg
  transfer[1] = ep->port; // port
  transfer[2] = 0;			// value to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PORT_SET, transfer, sizeof(u8) * 3);
#endif // _DEBUG

  UDCVDBG(udc, "Hub device on port: %d is removed.\n", ep->port);
  return 0;
}

static const struct usb_gadget_ops vusb_udc_ops = {
  .udc_start = vusb_udc_start,
  .udc_stop = vusb_udc_stop,
  .wakeup = vusb_wakeup,
  .match_ep = vusb_match_ep,

};

static const char driver_name[] = "vusb-udc";

int vusb_port_init(struct vusb_udc* udc, unsigned int devidx)
{
  struct vusb_port_dev* d = &udc->ports[devidx].dev;
  struct device* parent = &udc->spi->dev;
  int rc;

  d->name = devm_kasprintf(parent, GFP_KERNEL, "port%d", devidx + 1);
  d->index = devidx; // as port reference

  INIT_LIST_HEAD(&d->gadget.ep_list);

  /* Setup gadget structure */
  d->gadget.ops = &vusb_udc_ops;
  d->gadget.max_speed = USB_SPEED_FULL;
  d->gadget.speed = USB_SPEED_UNKNOWN;
  d->gadget.ep0 = &d->ep[0].ep_usb;
  d->gadget.name = KBUILD_MODNAME;
  d->gadget.dev.of_node = udc->spi->dev.of_node;

  INIT_WORK(&d->wk_start, vusb_port_start);
  INIT_WORK(&d->wk_stop, vusb_port_stop);

  int idx;
  for (idx = 0; idx < VUSB_MAX_EPS; idx++) {
    struct vusb_ep* ep = &d->ep[idx];

    spin_lock_init(&ep->lock);
    INIT_LIST_HEAD(&ep->queue);
    ep->dev_idx = devidx; // device idx on linux
    ep->udc = udc;
// todo: temp....
    ep->port = 2; // port on the mcu
    ep->halted = 0;
    ep->ep_usb.name = ep->name;
    ep->ep_usb.ops = &vusb_ep_ops;
    INIT_WORK(&ep->wk_data, vusb_ep_data);
    INIT_WORK(&ep->wk_status, vusb_ep_status);
    INIT_WORK(&ep->wk_irq_data, vusb_ep_irq_data);
    usb_ep_set_maxpacket_limit(&ep->ep_usb, VUSB_EP_MAX_PACKET_LIMIT);
    ep->maxpacket = ep->ep_usb.maxpacket;
    ep->idx = idx + 2; //  _PIPIRQ2	BIT(2), Pipe 2
    ep->ep0_dir = 0; // set while reading setup packet

    if (idx == 0) { /* For EP0 */
 // ep->pipe = portnr - 1;
      ep->ep_usb.desc = &ep0_desc;
      ep->ep_usb.maxpacket = usb_endpoint_maxp(&ep0_desc);
      ep->maxpacket = ep->ep_usb.maxpacket;
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

    list_add_tail(&ep->ep_usb.ep_list, &d->gadget.ep_list);
  }
  dev_info(&udc->spi->dev, "UDC gadget preparing to add\n");
  // gadget must be the last activated in the probe
  rc = usb_add_gadget_udc(&udc->spi->dev, &d->gadget);
  if (rc) {
    dev_err(&udc->spi->dev, "UDC gadget could not be added\n");
    return rc;
  }
  d->registered = true;

  return 0;

}
