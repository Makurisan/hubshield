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

static int vusb_ep_set_halt(struct usb_ep* _ep, int stall)
{
  struct vusb_ep* ep = to_vusb_ep(_ep);
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
  unsigned long flags;

  //vusb_nuke(ep, -ESHUTDOWN);

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
  unsigned long flags;

  if (unlikely(!_req || !_req->complete || !_req->buf || !_ep))
    return -EINVAL;

  _req->status = -EINPROGRESS;
  _req->actual = 0;

#ifdef _xDEBUG
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

  schedule_work(&ep->wk_ep_data);
  //UDCVDBG(ep->udc, "vusb_ep_queue, name: %s\n", ep->name);
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

static void vusb_ep_irq_data(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_irq_data);

  if (ep->dir == USB_DIR_BOTH) {
    u8 transfer[24];
    spi_cmd_t* cmd;

    transfer[0] = REG_PIPE_SPFIFO; // setup register
    transfer[1] = ep->idx; // octopus pipe
    vusb_read_buffer(ep->udc, VUSB_REG_MAP_PIPE_GET, transfer, sizeof(struct usb_ctrlrequest));
    cmd = (spi_cmd_t*)transfer;
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

#define to_ctrl_ep(ep) ep->dev->ep[0].idx

static void vusb_ep_data(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_ep_data);
  struct vusb_pipe* ctrl_pipe;

  // check control pipe
  ctrl_pipe = vusb_get_pipe(ep->udc, to_ctrl_ep(ep));
  if (ctrl_pipe && ctrl_pipe->enabled) {
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
    return;
  }
  UDCVDBG(ep->udc, "** Fatal Error vusb_ep_data - USB_DIR_IN: %s, pipe: %d\n", ep->name, ep->idx);
}

// called over worker from enable/disable ....
static void vusb_ep_status(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_status);
  unsigned long flags;
  u8 transfer[24];
  struct vusb_pipe* pipe;

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

    // enable the pipe
    pipe = vusb_get_pipe(ep->udc, ep->idx);
    pipe->enabled = true;

    UDCVDBG(ep->udc, "vusb_ep_state enable request, name: %s, pipe/id: %d\n", ep->name, ep->idx);

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
    UDCVDBG(ep->udc, "vusb_ep_state 'ERROR' request, name: %s, pipe: %d\n", ep->name, ep->idx);
    ep->halted = 0;
  }

}

struct vusb_pipe* vusb_get_pipe(struct vusb_udc* udc, u8 pipe_id)
{
  if (pipe_id >= VUSB_MCU_PIPE_ID)  {
    // we can't use pipe id directly as index 
    return &udc->pipes[pipe_id_to_index(pipe_id)];
  }
  return NULL;
}

struct vusb_ep* vusb_get_ep(struct vusb_udc* udc, u8 pipe_id)
{
  struct vusb_ep* ep = NULL;
  struct vusb_pipe* pipe = vusb_get_pipe(udc, pipe_id);

  if (pipe && pipe->used) {
    ep = pipe->ep;
  } 
  if (!ep) {
    UDCVDBG(udc, "*** FATAL error vusb_get_ep pipe: %d\n", pipe_id);
  } 
  return ep;
}

static void vusb_clear_mcu_pipe(struct vusb_port_dev *dev)
{
  int idx;

  for (idx = 0; idx < VUSB_MAX_EPS; idx++) {
    struct vusb_ep* ep = &dev->ep[idx];
    struct vusb_pipe* pipe = vusb_get_pipe(dev->udc, ep->idx);
    if (ep->idx != 0 && pipe) {
      //UDCVDBG(ep->udc, "*** CHECK 2 vusb_clear_mcu_pipe port:%d ep/id: %d\n", ep->port, ep->idx);
      pipe->used = false;
      pipe->enabled = false;
      pipe->ep = NULL;
      ep->idx = 0;
    }
  }
}

/*
  Associate a free pipe from the MCU to the ep
*/
static int vusb_free_mcu_pipe(struct vusb_ep* ep)
{
  int idx;
  if (ep->idx == 0) {
    for (idx = 0; idx < VUSB_MAX_PIPES; idx++) {
      struct vusb_pipe* pipe = &ep->udc->pipes[idx];
      if (pipe->used == false) {
        pipe->used = true;
        pipe->ep = ep;
        ep->idx = pipe->pipe_idx;
        return true;
      }
    }
  }
  return false;
}

/*
  This function predefines and reserve the endpoint pipes on the mcu
*/
static void vusb_match_pipe(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_udc_work);
  struct vusb_udc* udc = ep->udc;
  u8 transfer[10];

  // assoc a new pipe from the mcu
  vusb_free_mcu_pipe(ep);

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

static int vusb_wakeup(struct usb_gadget* gadget)
{
  struct vusb_ep* ep = ep_usb_to_vusb_ep(gadget->ep0);
  int ret = -EINVAL;
  dev_info(&ep->udc->spi->dev, "Hub gadget vusb_wakeup.\n");
  return ret;
}

static struct usb_ep* vusb_match_ep(struct usb_gadget* gadget,  struct usb_endpoint_descriptor* desc,
  struct usb_ss_ep_comp_descriptor* ep_comp)
{
  struct usb_ep* _ep;
  struct vusb_ep* ep;
  struct vusb_port_dev* dev = gadget_to_dev(gadget);

  //UDCVDBG(udc, "Hub vusb_match_ep \n");

  /* Look at endpoints until an unclaimed one looks usable */
  list_for_each_entry(_ep, &gadget->ep_list, ep_list) {
    if (usb_gadget_ep_match_desc(gadget, _ep, desc, ep_comp))
      goto found_ep;
  }
  /* Fail */
  return NULL;

found_ep:

  ep = ep_usb_to_vusb_ep(_ep);
  
  UDCVDBG(dev->udc, "Hub vusb_match_ep on port: %d, ep/name: %s\n", ep->port, ep->name);

  // schedule the ep
  INIT_WORK(&ep->wk_udc_work, vusb_match_pipe);
  schedule_work(&ep->wk_udc_work);

  return _ep;

}
#ifdef _DEBUG
static void vusb_dev_nuke(struct vusb_port_dev* dev, int status)
{
  unsigned int idx;

  for (idx = 1; idx < VUSB_MAX_EPS; idx++) {
    //if (dev->ep[idx].ep_usb.caps.dir_in)
      vusb_nuke(&dev->ep[idx], -ECONNRESET);
  }
  UDCVDBG(dev->udc, "vusb_nuke port:%d\n", dev->ep[0].port);

}
#endif

// the function is called with the control pipe of the gadget
static void vusb_port_start(struct work_struct* work)
{
  struct vusb_port_dev* dev = container_of(work, struct vusb_port_dev, wk_start);
  struct vusb_ep* ep0 = &dev->ep[0]; // control pipe of the dev
  struct vusb_udc* udc = ep0->udc;
  struct vusb_pipe* pipe;

  u8 transfer[24];

  // control pipe

  // assoc a new pipe from the mcu
  vusb_free_mcu_pipe(ep0);

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

  // enable the pipe
  pipe = vusb_get_pipe(udc, ep0->idx);
  pipe->enabled = true;

  //UDCVDBG(udc, "Hub port %d with ctrl/pipe: %s is now in remote stage and enabled", ep0->port, ep0->name);
  UDCVDBG(udc, "Hub gadget started with pipe/id: %d, name: %s\n", ep0->idx, ep0->name);

}

static int vusb_udc_start(struct usb_gadget* gadget, struct usb_gadget_driver* driver)
{
  unsigned long flags;
  struct vusb_ep* ep = ep_usb_to_vusb_ep(gadget->ep0);
  struct vusb_udc* udc = ep->udc;
  struct vusb_port_dev* dev = ep->dev;

  spin_lock_irqsave(&udc->lock, flags);
  /* hook up the driver */
  driver->driver.bus = NULL;
  dev->driver = driver;
  dev->gadget.speed = USB_SPEED_FULL;

  dev->gadget.is_selfpowered = udc->is_selfpowered;
  udc->remote_wkp = 0;
  udc->softconnect = true;
  spin_unlock_irqrestore(&udc->lock, flags);

  schedule_work(&dev->wk_start);

  return 0;
}

static void vusb_port_stop(struct vusb_ep* ep0)
{
  struct vusb_udc* udc = ep0->udc;
  u8 transfer[24];

  UDCVDBG(udc, "Port %d will be detached %p", ep0->port, ep0->dev);

  vusb_clear_mcu_pipe(ep0->dev);

  // set disable on port
  transfer[0] = PORT_REG_ENABLED; // reg
  transfer[1] = ep0->port; // port: 2
  transfer[2] = 0;			// value to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PORT_SET, transfer, sizeof(u8) * 3);

}

static int vusb_udc_stop(struct usb_gadget* gadget)
{
  unsigned long flags;
  struct vusb_ep* ep0 = ep_usb_to_vusb_ep(gadget->ep0);
  struct vusb_udc* udc = ep0->udc;
  struct vusb_port_dev* dev = ep0->dev;;

  /* clear all pending requests */
  //vusb_dev_nuke(dev, -ESHUTDOWN);
  UDCVDBG(udc, "Hub gadget vusb_udc_stop on port: %d\n", ep0->port);

  spin_lock_irqsave(&udc->lock, flags);
  udc->is_selfpowered = dev->gadget.is_selfpowered;
  dev->gadget.speed = USB_SPEED_UNKNOWN;
  dev->driver = NULL;
  udc->softconnect = false;
  spin_unlock_irqrestore(&udc->lock, flags);

  /* write port disable to MCU */
  //schedule_work(&dev->wk_stop);
  vusb_port_stop(ep0);


  return 0;
}

static const struct usb_gadget_ops vusb_udc_ops = {
  .udc_start = vusb_udc_start,
  .udc_stop = vusb_udc_stop,
  .wakeup = vusb_wakeup,
  .match_ep = vusb_match_ep,

};

static const char driver_name[] = "vusb-udc";

static void vusb_dev_release(struct device* dev)
{
  kfree(dev);
}

int vusb_port_init(struct vusb_udc* udc, unsigned int devidx)
{
  struct vusb_port_dev* dev = &udc->ports[devidx].dev;
  struct device* parent = &udc->spi->dev;
  int rc, idx;

  dev->name = devm_kasprintf(parent, GFP_KERNEL, "port%d", devidx);
  dev->index = devidx; // as port reference
  dev->udc = udc;
  dev->port_dev = kzalloc(sizeof(struct device), GFP_KERNEL);
  device_initialize(dev->port_dev);
  dev->port_dev->release = vusb_dev_release;
  dev->port_dev->parent = parent;
  dev_set_name(dev->port_dev, "%s:p%d", dev_name(parent), devidx + 1);
  rc = device_add(dev->port_dev);

  INIT_LIST_HEAD(&dev->gadget.ep_list);

  /* Setup gadget structure */
  dev->gadget.ops = &vusb_udc_ops;
  dev->gadget.max_speed = USB_SPEED_FULL;
  dev->gadget.speed = USB_SPEED_UNKNOWN;
  dev->gadget.ep0 = &dev->ep[0].ep_usb;
  dev->gadget.name = KBUILD_MODNAME;
  dev->gadget.dev.of_node = udc->spi->dev.of_node;

  INIT_WORK(&dev->wk_start, vusb_port_start);
  //INIT_WORK(&dev->wk_stop, vusb_port_stop);

  for (idx = 0; idx < VUSB_MAX_EPS; idx++) {
    struct vusb_ep* ep = &dev->ep[idx];

    spin_lock_init(&ep->lock);
    INIT_LIST_HEAD(&ep->queue);
    ep->udc = udc;
    ep->dev = dev;
// todo: temp....
    ep->port = devidx + 1; // port on the mcu
    ep->halted = 0;
    ep->ep_usb.name = ep->name;
    ep->ep_usb.ops = &vusb_ep_ops;
    INIT_WORK(&ep->wk_ep_data, vusb_ep_data);
    INIT_WORK(&ep->wk_status, vusb_ep_status);
    INIT_WORK(&ep->wk_irq_data, vusb_ep_irq_data);
    usb_ep_set_maxpacket_limit(&ep->ep_usb, VUSB_EP_MAX_PACKET_LIMIT);
    ep->maxpacket = ep->ep_usb.maxpacket;
    ep->idx = 0; // unused 
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

    list_add_tail(&ep->ep_usb.ep_list, &dev->gadget.ep_list);
  }
  // gadget must be the last activated in the probe
  rc = usb_add_gadget_udc(dev->port_dev, &dev->gadget);
  if (rc) {
    dev_err(&udc->spi->dev, "UDC gadget could not be added\n");
    return rc;
  }
  dev->registered = true;
  dev_info(&udc->spi->dev, "UDC port gadget added with name: %s\n", dev_name(dev->port_dev));
  return 0;

}
