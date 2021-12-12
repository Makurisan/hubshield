// SPDX-License-Identifier: GPL-2.0+
/*
 * VUSB Device Controller driver for USB.
 *
 * Author: Manfred Kubica <ManfredKubica@web.de>
 *
 */
 
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
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


extern const struct file_operations vusb_ops;

static const char driver_name[] = "vusb-udc";

/* Forward declaration */
static int vusb_remove(struct spi_device* spi);
struct vusb_ep* vusb_get_ep(struct vusb_udc* udc, u8 ep_idx);

static void vusb_getstatus(struct vusb_ep* ep)
{
	u16 status = 0;
	struct vusb_udc* udc = ep->udc;
	struct vusb_ep* _ep;

	switch (ep->setup.bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		/* Get device status */
		status = udc->gadget.is_selfpowered << USB_DEVICE_SELF_POWERED;
		status |= (udc->remote_wkp << USB_DEVICE_REMOTE_WAKEUP);
		break;
	case USB_RECIP_INTERFACE:
		if (udc->driver->setup(&udc->gadget, &ep->setup) < 0)
			goto stall;
		break;
	case USB_RECIP_ENDPOINT:
		//_ep = &udc->ep[ep->setup.wIndex & USB_ENDPOINT_NUMBER_MASK];
    UDCVDBG(udc, "******** vusb_getstatus - not implemented yet...\n");
		break;
	default:
		goto stall;
	}

	status = cpu_to_le16(status);
	//spi_wr_buf(udc, VUSB_REG_EP0FIFO, &status, 2);
	//spi_wr8_ack(udc, VUSB_REG_EP0BC, 2, 1);
  vusb_spi_pipe_ack(udc, ep);
  UDCVDBG(udc, "Respond to getstatus request\n");
	return;
stall:
	UDCVDBG(udc, "Can't respond to getstatus request\n");
	//spi_wr8(udc, VUSB_REG_EPSTALLS, STLEP0IN | STLEP0OUT | STLSTAT);
}

static void vusb_set_clear_feature(struct vusb_ep* ep)
{
	struct vusb_udc* udc = ep->udc;
	struct vusb_ep *_ep;
	int set = ep->setup.bRequest == USB_REQ_SET_FEATURE;
	unsigned long flags;
	int id;

	switch (ep->setup.bRequestType) {
	case USB_RECIP_DEVICE:
		if (ep->setup.wValue != USB_DEVICE_REMOTE_WAKEUP)
			break;

		if (ep->setup.bRequest == USB_REQ_SET_FEATURE)
			udc->remote_wkp = 1;
		else
			udc->remote_wkp = 0;
		//return spi_ack_ctrl(udc);
    return;

	case USB_RECIP_ENDPOINT:
		if (ep->setup.wValue != USB_ENDPOINT_HALT)
			break;

		id = ep->setup.wIndex & USB_ENDPOINT_NUMBER_MASK;
		_ep = &udc->ep[id];

		spin_lock_irqsave(&_ep->lock, flags);
		_ep->todo &= ~STALL_EP;
		if (set)
			_ep->todo |= STALL;
		else
			_ep->todo |= UNSTALL;
		spin_unlock_irqrestore(&_ep->lock, flags);
		UDCVDBG(udc, "vusb_set_clear_feature: stall\n");
		schedule_work(&_ep->wk_status);
		return;
	default:
		break;
	}
	UDCVDBG(udc, "vusb_set_clear_feature: Can't respond\n");
	//spi_wr8(udc, VUSB_REG_EPSTALLS, STLEP0IN | STLEP0OUT | STLSTAT);
}

void vusb_handle_setup(struct vusb_ep* ep)
{
	struct vusb_udc* udc = ep->udc;

	//ep->setup = setup;
	//ep->setup.wValue = cpu_to_le16(setup.wValue);
	//ep->setup.wIndex = cpu_to_le16(setup.wIndex);
	//ep->setup.wLength = cpu_to_le16(setup.wLength);

	switch (ep->setup.bRequest) {
	case USB_REQ_GET_STATUS:
    UDCVDBG(udc, "Get status, reqtype: %x\n", ep->setup.bRequestType);
		/* Data+Status phase form udc */
		if ((ep->setup.bRequestType &
				(USB_DIR_IN | USB_TYPE_MASK)) !=
				(USB_DIR_IN | USB_TYPE_STANDARD)) {
			break;
		}
    return vusb_getstatus(ep);
	case USB_REQ_SET_ADDRESS:
		/* Status phase from udc */
		if (ep->setup.bRequestType != (USB_DIR_OUT |
				USB_TYPE_STANDARD | USB_RECIP_DEVICE)) {
			break;
		}
    // ack setaddress
    vusb_spi_pipe_ack(udc, ep);
    UDCVDBG(udc, "Assigned Address=%d, epname: %s\n", ep->setup.wValue, ep->name);
		return;
	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
    UDCVDBG(udc, "Clear/Set feature wValue:%d, pipe: %d\n", ep->setup.wValue, ep->idx);
    /* Requests with no data phase, status phase from udc */
		if ((ep->setup.bRequestType & USB_TYPE_MASK)
				!= USB_TYPE_STANDARD)
			break;
		return vusb_set_clear_feature(ep);
	default:
		break;
	}

  //UDCVDBG(udc, "handle_setup driver: %x\n", udc->driver);
	if (udc->driver != NULL && udc->driver->setup(&udc->gadget, &ep->setup) < 0) {
    UDCVDBG(udc, "setup error: epname: %s Type: %x Request: %x\n", ep->name, ep->setup.bRequestType, ep->setup.bRequest);
    // prints the setup packet which leads to the error
		pr_hex_mark_debug((void*)&ep->setup, sizeof(struct usb_ctrlrequest), PRINTF_READ, ep->name, "setup error");
  	//	/* Stall EP0 */
	}

}

void vusb_req_done(struct vusb_req *req, int status)
{
	struct vusb_ep *ep = req->ep;
	struct vusb_udc *udc = ep->udc;

  //UDCVDBG(udc, "%s vusb_req_done %p, status %d\n", ep->name, req, status);
  //UDCVDBG(udc, "---> vusb_req_done: %s\n", &ep->name);

	if (req->usb_req.status == -EINPROGRESS)
		req->usb_req.status = status;
  else {
    status = req->usb_req.status;
  }

  if (status && status != -ESHUTDOWN) {
    UDCVDBG(udc, "%s done %p, status %d\n", ep->ep_usb.name, req, status);
  }

	if (req->usb_req.complete)
		req->usb_req.complete(&ep->ep_usb, &req->usb_req);
}

int vusb_do_data(struct vusb_udc *udc, struct vusb_ep* ep)
{
	struct vusb_req *req;
	int done, length, psz;
	void *buf;

	if (list_empty(&ep->queue))
		return false;

	req = list_first_entry(&ep->queue, struct vusb_req, queue);
	buf = req->usb_req.buf + req->usb_req.actual;

	psz = ep->ep_usb.maxpacket;
	length = req->usb_req.length - req->usb_req.actual;
	length = min(length, psz);

	if (length == 0) {
		done = 1;
		goto xfer_done;
	}
	UDCVDBG(udc, "vusb_do_data, name: %s, ep/idx: %d, reqtype: %x, request:%x, length:%d\n", ep->name, ep->idx, ep->setup.bRequestType, 
		ep->setup.bRequest, length);

	done = 0;

	if (ep->dir == USB_DIR_BOTH) {
		// OUT setup packet
		if ( ep->ep0_dir == USB_DIR_OUT && ep->setup.wLength)	{
      length = min(length, psz);
      udc->spitransfer[0] = REG_PIPE_FIFO;
      udc->spitransfer[1] = req->ep->idx;
      vusb_read_buffer(udc, VUSB_REG_MAP_PIPE_GET, udc->spitransfer, length + 2 * sizeof(u8));
      spi_cmd_t* cmd = (spi_cmd_t*)udc->spitransfer;
      memmove(buf, cmd->data, length); // mcu pipe index
      pr_hex_mark_debug(buf, length, PRINTF_READ, req->ep->name, "Get - OUT");
      prefetchw(buf);
		}	else {
      prefetch(buf);
      udc->spitransfer[0] = REG_PIPE_FIFO;
      udc->spitransfer[1] = req->ep->idx;
      memmove(&udc->spitransfer[2], buf, length); // mcu pipe index
      vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, udc->spitransfer, length + 2 * sizeof(u8));
      pr_hex_mark_debug(buf, length, PRINTF_READ, req->ep->name, "Get - IN");
		}
		if (length < psz) {
			done = 1;
		}

	} 
  if (ep->dir == USB_DIR_OUT) {

    //pr_hex_mark_debug(buf, length, PRINTF_READ, req->ep->name, "EP-OUT");
    length = min(length, psz);

    u8 transfer[68];
    // OUT data from the mcu...
    transfer[0] = REG_PIPE_FIFO; // write&read register
    transfer[1] = ep->idx; // pipe
    vusb_read_buffer(ep->udc, VUSB_REG_MAP_PIPE_GET, transfer, 1 + 2 * sizeof(u8));
    spi_cmd_t* cmd = (spi_cmd_t*)transfer;
    //UDCVDBG(ep->udc, "vusb_do_data, name: %s, pipe: %d, cnt: %d\n", ep->name, ep->idx, ep->maxpacket);
		prefetchw(buf);
		length = cmd->length - 2; // 2 = crc16
		memmove(buf, cmd->data, cmd->length);
    pr_hex_mark_debug(buf, length, PRINTF_READ, req->ep->name, "EP-OUT");
		if (length < ep->ep_usb.maxpacket)
			done = 1;
	}

	if (ep->dir == USB_DIR_IN) {
    pr_hex_mark_debug(buf, length, PRINTF_READ, req->ep->name, "- EP-IN");
    prefetch(buf);
    udc->spitransfer[0] = REG_PIPE_FIFO;
    udc->spitransfer[1] = req->ep->idx;
    memmove(&udc->spitransfer[2], buf, length); // mcu pipe index
    vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, udc->spitransfer, length + 2 * sizeof(u8));
    if (length < psz) {
      done = 1;
    }

  }

	req->usb_req.actual += length;

	if (req->usb_req.actual == req->usb_req.length)
		done = 1;

	//UDCVDBG(udc, "***** vusb_req_dodata add, name: %s, length: %d psz: %d, done:%d\n", ep->name, req->usb_req.length,
	//	req->usb_req.actual, done);

xfer_done:
	if (done) { 
		unsigned long flags;

		spin_lock_irqsave(&ep->lock, flags);
		list_del_init(&req->queue);
		spin_unlock_irqrestore(&ep->lock, flags);

		// todo must be changed to 
		if (ep->ep_usb.caps.dir_in)
			vusb_spi_pipe_ack(udc, ep);
		vusb_req_done(req, 0);
		return false;
	}
	return true;
}

#define REG_USBIRQ	3
#define REG_IRQ_ELEMENTS 12

static void vusb_irq_mcu_handler(struct work_struct* work)
{
	struct vusb_udc* udc = container_of(work, struct vusb_udc, vusb_irq_wq);

	u8 transfer[48];
	if (vusb_read_buffer(udc, VUSB_REG_IRQ_GET, transfer, REG_IRQ_ELEMENTS)) {

		vusb_req_map_t irq_map;
		memmove(&irq_map, transfer + VUSB_SPI_HEADER, REG_IRQ_ELEMENTS);
		u8 usbirq = irq_map.USBIRQ & irq_map.USBIEN;

	// pipe handling
		irq_map.PIPIEN = htonl(irq_map.PIPIEN);
		irq_map.PIPIRQ = htonl(irq_map.PIPIRQ);
		u32 pipeirq = irq_map.PIPIEN & irq_map.PIPIRQ;
		if (pipeirq) {
			// clear pipeirq flags
			transfer[0] = REG_PIPEIRQ;
			*(u32*)&transfer[1] = htonl(irq_map.PIPIRQ);
			vusb_write_buffer(udc, VUSB_REG_IRQ_CLEAR, transfer, sizeof(u8) + sizeof(u32));
			// process the irqs
			while (hweight32(pipeirq)) {
				u32 irq = _bf_ffsl(pipeirq);
				struct vusb_ep* ep = vusb_get_ep(udc, irq);
				// schedule a setup packet
				if (ep)
					schedule_work(&ep->wk_irq_data);
				pipeirq &= ~BIT(irq);
			}
		}

		if (usbirq & SRESIRQ) {
			UDCVDBG(udc, "USB-Reset start\n");
			transfer[0] = REG_USBIRQ;
			transfer[1] = SRESIRQ;
			vusb_write_buffer(udc, VUSB_REG_IRQ_CLEAR, transfer, 2);
			return;
		}
		if (usbirq & URESIRQ) {
			UDCVDBG(udc, "USB-Reset end\n");
			transfer[0] = REG_USBIRQ;
			transfer[1] = URESIRQ;
			vusb_write_buffer(udc, VUSB_REG_IRQ_CLEAR, transfer, 2);
			return;
		}
		if (usbirq & HRESIRQ) {
			UDCVDBG(udc, "System-Reset\n");
			//msleep_interruptible(5);
			transfer[0] = REG_USBIRQ;
			transfer[1] = HRESIRQ;
			vusb_write_buffer(udc, VUSB_REG_IRQ_CLEAR, transfer, 2);
			// connect the usb to the host   
			transfer[0] = REG_CPUCTL;
			// firmware version auslesen, is chip compatible
			transfer[1] = SOFTCONT;
			vusb_write_buffer(udc, VUSB_REG_SET, transfer, 2);
			return;
		}

	}

}

// read all mcu IRQs    
static irqreturn_t vusb_mcu_irq(int irq, void* dev_id)
{
  struct vusb_udc* udc = dev_id;
  irqreturn_t iret = IRQ_HANDLED;

  struct irq_desc* desc = irq_to_desc(irq);
  struct irq_data* data = irq_desc_get_irq_data(desc);
  if (desc && data && desc->irq_data.hwirq == GPIO_LISTEN_IRQ_PIN)
  {
    unsigned long flags;
    struct irq_chip* chip = irq_desc_get_chip(desc);
    if (chip)
    {
      spin_lock_irqsave(&udc->wq_lock, flags);
			schedule_work(&udc->vusb_irq_wq);
			spin_unlock_irqrestore(&udc->wq_lock, flags);
    }
  }
  return iret;
}

static int vusb_wakeup(struct usb_gadget *gadget)
{
	struct vusb_udc *udc = gadget_to_udc(gadget);
	unsigned long flags;
	int ret = -EINVAL;
	dev_info(&udc->spi->dev, "Hub gadget vusb_wakeup.\n");
	return ret;
}

#define VUSB_TYPE_CTRL	0
#define VUSB_TYPE_INT		1
#define DEBUG

// the function is called with the control pipe of the gadget
static void vusb_port_start(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_udc_work);
  struct vusb_udc* udc = ep->udc;

  u8 transfer[24];

	// control pipe

	// register the control ep on the port in the hub
	transfer[0] = REG_PIPE_PORT; // reg
  transfer[1] = ep->idx; // pipe num
  transfer[2] = ep->port; // port
  vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

  // set enable on control pipe
  transfer[0] = REG_PIPE_ENABLED; // reg
  transfer[1] = ep->idx; // pipe num
  transfer[2] = 1;			// value to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);
	//UDCVDBG(udc, "  - port: %d with  pipe/id: %d ep/name: %s\n", ep->port, ep->idx, ep->name);

  // remote or local port
  transfer[0] = PORT_REG_DEVTYPE; // reg
  transfer[1] = ep->port; // port
  transfer[2] = VUSB_PORT_DEVICE_REMOTE; // field to set; activate remote or local device
  vusb_write_buffer(udc, VUSB_REG_MAP_PORT_SET, transfer, sizeof(u8) * 3);

  // set enable on port
  transfer[0] = PORT_REG_ENABLED; // reg
  transfer[1] = ep->port; // port
  transfer[2] = 1;			// value to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PORT_SET, transfer, sizeof(u8) * 3);
  //UDCVDBG(udc, "  - Pipe: %d on port: %d is enabled name: %s\n", ep->pipe, ep->port, ep->name);

	UDCVDBG(udc, "Hub port %d with ctrl/pipe: %s is now in remote stage and enabled", ep->port, ep->name);

}

static int vusb_udc_start(struct usb_gadget *gadget, struct usb_gadget_driver *driver)
{
	struct vusb_udc *udc = gadget_to_udc(gadget);
  struct vusb_ep* ep = ep_usb_to_vusb_ep(gadget->ep0);

	unsigned long flags;
	spin_lock_irqsave(&udc->lock, flags);
	/* hook up the driver */
	driver->driver.bus = NULL;
	udc->driver = driver;
	udc->gadget.speed = USB_SPEED_FULL;

	udc->gadget.is_selfpowered = udc->is_selfpowered;
	udc->remote_wkp = 0;
	udc->softconnect = true;
	udc->todo |= UDC_START;
	spin_unlock_irqrestore(&udc->lock, flags);

  INIT_WORK(&ep->wk_udc_work, vusb_port_start);
	schedule_work(&ep->wk_udc_work);

	//UDCVDBG(udc, "Hub gadget vusb_udc_start\n");

	return 0;
}

void vusb_nuke(struct vusb_ep* ep, int status);

static void vusb_dev_nuke(struct vusb_udc* udc, int status)
{
  unsigned int i;

  for (i = 0; i < VUSB_MAX_EPS; i++) {
		if (udc->ep[i].ep_usb.caps.dir_in)	{
      vusb_nuke(&udc->ep[i], status);
		}
  }
}

static void vusb_port_stop(struct work_struct* work)
{
	struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_udc_work);
	struct vusb_udc* udc = ep->udc;
  //UDCVDBG(udc, "Port %d will be detached", ep->port);

  u8 transfer[24];
	// set disable on port
  transfer[0] = PORT_REG_ENABLED; // reg
  transfer[1] = ep->port; // port: 2
  transfer[2] = 0;			// value to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PORT_SET, transfer, sizeof(u8) * 3);

}

static int vusb_udc_stop(struct usb_gadget *gadget)
{
	struct vusb_udc *udc = gadget_to_udc(gadget);
  struct vusb_ep* ep = ep_usb_to_vusb_ep(gadget->ep0);
	unsigned long flags;

	spin_lock_irqsave(&udc->lock, flags);
	udc->is_selfpowered = udc->gadget.is_selfpowered;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->driver = NULL;
	udc->softconnect = false;
	udc->todo |= UDC_START;
	spin_unlock_irqrestore(&udc->lock, flags);

  //vusb_dev_nuke(udc, -ESHUTDOWN);

#define DEBUG
#ifdef DEBUG
  INIT_WORK(&ep->wk_udc_work, vusb_port_stop);
  schedule_work(&ep->wk_udc_work);
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
  transfer[2] = ep->ep_usb.caps.type_control?REG_EP_CONTROL:REG_EP_INTERRUPT; // field to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

  // set the pipe type
  transfer[0] = REG_PIPE_MAXPKTS; // reg
  transfer[1] = ep->idx; // pipe num
  transfer[2] = 0x40;			// field to set
  vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, transfer, sizeof(u8) * 3);

}

static struct usb_ep* vusb_match_ep(struct usb_gadget* gadget,
	struct usb_endpoint_descriptor* desc,
	struct usb_ss_ep_comp_descriptor* ep_comp)
{
  struct vusb_udc* udc = gadget_to_udc(gadget);
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

  //UDCVDBG(udc, "Hub vusb_match_ep ep0:%s\n", ep->name);

	return _ep;

}
	
static const struct usb_gadget_ops vusb_udc_ops = {
	.udc_start	= vusb_udc_start,
	.udc_stop	= vusb_udc_stop,
	.wakeup		= vusb_wakeup,
  .match_ep = vusb_match_ep,

};

static int vusb_probe(struct spi_device *spi)
{
	struct vusb_udc *udc;
	int rc = 0;
	u8 reg[8];

  const struct device_node* np = spi->dev.of_node;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

  rc = spi_setup(spi);
	if (rc) {
		dev_err(&spi->dev, "Unable to setup SPI bus\n");
		return -EFAULT;
	}

	udc = devm_kzalloc(&spi->dev, sizeof(*udc), GFP_KERNEL);
	if (!udc)
		return -ENOMEM;

	udc->spi = spi;
	udc->remote_wkp = 0;

  // change to device tree later
  rc = of_property_read_u32(np, "max-ports", &udc->max_ports);
  if (rc < 0) {
    dev_err(&spi->dev, "Unable to allocate hub downstreams ports.\n");
    return -ENOMEM;
  }
	dev_info(&spi->dev, "Hub device has initiated %d hub ports.\n", udc->max_ports);

	dev_info(&spi->dev, "Spi clock set at %u KHz.\n",
                 (udc->spi->max_speed_hz + 500) / 1000);

	/* Setup gadget structure */
	udc->gadget.ops = &vusb_udc_ops;
	udc->gadget.max_speed = USB_SPEED_FULL;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.ep0 = &udc->ep[0].ep_usb;
	udc->gadget.name = driver_name;
  udc->gadget.dev.of_node = udc->spi->dev.of_node;

  spin_lock_init(&udc->lock);
  spin_lock_init(&udc->wq_lock);

  /* INTERRUPT spi read queue */
  init_waitqueue_head(&udc->spi_read_queue);
	mutex_init(&udc->spi_read_mutex);
	mutex_init(&udc->spi_write_mutex);

	/* setup Endpoints */
	vusb_eps_init(udc);

  udc->transfer = devm_kcalloc(&spi->dev, VUSB_SPI_BUFFER_LENGTH, sizeof(u8), GFP_KERNEL);
  if (!udc->transfer) {
    dev_err(&spi->dev, "Unable to allocate Hub transfer buffer.\n");
    return -ENOMEM;
  }
  udc->spitransfer = devm_kcalloc(&spi->dev, VUSB_SPI_BUFFER_LENGTH, sizeof(u8), GFP_KERNEL);
  if (!udc->spitransfer) {
    dev_err(&spi->dev, "Unable to allocate SPI transfer buffer.\n");
    return -ENOMEM;
  }
  udc->spiwritebuffer = devm_kcalloc(&spi->dev, VUSB_SPI_BUFFER_LENGTH, sizeof(u8), GFP_KERNEL);
  if (!udc->spiwritebuffer){
    dev_err(&spi->dev, "Unable to allocate SPI transfer buffer.\n");
    return -ENOMEM;
  }

  /* Init crc8 */
  crc8_populate_msb(udc->crc_table, 0x7);
// udc gagdet dev ?????
	udc->dev = &udc->gadget.dev;

	spi_set_drvdata(spi, udc);

  udc->spi_datrdy = gpio_to_irq(GPIO_DATRDY_IRQ_PIN);
  dev_info(&udc->spi->dev, "GPIO for mcu dtrdy hwirq %d is irq %d.\n", GPIO_DATRDY_IRQ_PIN, udc->spi_datrdy);
  // set the irq handler: list with "cat /proc/interrupts"
  rc = devm_request_threaded_irq(&udc->spi->dev, udc->spi_datrdy,
    NULL, vusb_spi_dtrdy, IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED | IRQF_NO_SUSPEND, "vusbdtrdy", udc);
  if (rc)
  {
    dev_err(&udc->spi->dev, "Failed to request dtrdy hwirq interrupt\n");
    rc = -ENOMEM;
    goto err;
  }

  rc = of_property_read_u32(np, "spi_irq", &udc->mcu_irq);
  if (rc < 0) {
    dev_err(&spi->dev, "Unable to get IRQ interrupt pin.\n");
    return -ENOMEM;
  }
  udc->mcu_irq = gpio_to_irq(GPIO_LISTEN_IRQ_PIN);
  dev_info(&udc->spi->dev, "GPIO for mcu listen hwirq %d is irq %d.\n", GPIO_LISTEN_IRQ_PIN, udc->mcu_irq);
  rc = devm_request_threaded_irq(&udc->spi->dev, udc->mcu_irq, NULL,
    vusb_mcu_irq, IRQF_SHARED | IRQF_ONESHOT | IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING, "vusbirq", udc);
  if (rc) {
    dev_err(&udc->spi->dev, "Failed to request listen hwirq interrupt\n");
    rc = -ENOMEM;
    goto err;
  }

#define _RESET_PIN
#ifdef _RESET_PIN
#define GPIO_RESET_PIN 24
  /* GPIO for mcu chip reset */
  udc->mcu_gpreset = devm_gpiod_get(&udc->spi->dev, "reset", GPIOD_OUT_HIGH_OPEN_DRAIN);
  dev_info(&udc->spi->dev, "Reset gpio is defined as gpio:%x\n", udc->mcu_gpreset);
  gpiod_set_value(udc->mcu_gpreset, 1);
#endif

#ifdef _DEBUG
	gpiod_set_value(udc->mcu_gpreset, 0);
	msleep_interruptible(500);
	gpiod_set_value(udc->mcu_gpreset, 1);
	msleep_interruptible(100);
#endif // _DEBUG


	udc->is_selfpowered = 1;
	udc->todo |= UDC_START;
  udc->softconnect = true;

  // char device
  dev_t usrdev;
  alloc_chrdev_region(&usrdev, 0, VUSB_MAX_CHAR_DEVICES, "vusb");
  udc->crdev_major = MAJOR(usrdev);
  cdev_init(&udc->cdev, &vusb_ops);

  rc = cdev_add(&udc->cdev, usrdev, VUSB_MAX_CHAR_DEVICES);
  if (rc < 0) {
    pr_warn("Couldn't cdev_add\n");
    goto err;
  }
  udc->chardev_class = class_create(THIS_MODULE, "vusb");
  udc->chardev_class->dev_uevent = vusb_chardev_uevent;

  device_create(udc->chardev_class, NULL, MKDEV(udc->crdev_major, 1), NULL, "vusb-%d", 1);

  UDCVDBG(udc, "Succesfully initialized vusb.\n");

  vusb_mpack_buffer(udc, 2, udc->transfer, 512);
	
	INIT_WORK(&udc->vusb_irq_wq, vusb_irq_mcu_handler);

  //udc->qwork = create_singlethread_workqueue("octohub");

	// gadget must be the last activated in the probe
	rc = usb_add_gadget_udc(&spi->dev, &udc->gadget);
  if (rc) {
    dev_err(&spi->dev, "UDC gadget could not be added\n");
    return rc;
  }

  usb_udc_vbus_handler(&udc->gadget, true);
  usb_gadget_set_state(&udc->gadget, USB_STATE_POWERED);

	return 0;
err:
  vusb_remove(spi);
  dev_err(&spi->dev, "Failed to initialize vusb.\n");
  return rc;
}

static int vusb_remove(struct spi_device *spi)
{
	struct vusb_udc *udc = spi_get_drvdata(spi);
	unsigned long flags;

  dev_info(&spi->dev, "Removing vusbhub\n");

  disable_irq(udc->mcu_irq);
  disable_irq(udc->spi_datrdy);

  cdev_del(&udc->cdev);

	usb_del_gadget_udc(&udc->gadget);

  // remove the char device
  device_destroy(udc->chardev_class, MKDEV(udc->crdev_major, 1));
  class_destroy(udc->chardev_class);

  dev_t dev_id = MKDEV(udc->crdev_major, 0);
  unregister_chrdev_region(dev_id, VUSB_MAX_CHAR_DEVICES);

	//if (udc->qwork) {
	//	flush_workqueue(udc->qwork);
	//	destroy_workqueue(udc->qwork);
	//}

  dev_info(&spi->dev, "Char device from v-hub removed.\n");

	return 0;
}

static const struct of_device_id vusb_udc_of_match[] = {
  {.compatible = "hubshield,v-hub", },
  {},
};
MODULE_DEVICE_TABLE(of, vusb_udc_of_match);

static struct spi_driver vusb_driver = {
	.probe = vusb_probe,
	.remove = vusb_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_match_ptr(vusb_udc_of_match),
	},
};

module_spi_driver(vusb_driver);
MODULE_DESCRIPTION("USB hub udc driver");
MODULE_AUTHOR("Manfred Kubica <manfredkubica@web.de>");
MODULE_LICENSE("GPL");
