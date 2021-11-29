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

static void __vusb_stop(struct vusb_udc *udc)
{
	u8 val;
	int i;

	/* clear all pending requests */
	for (i = 1; i < VUSB_MAX_EPS; i++)
		vusb_nuke(&udc->ep[i], -ECONNRESET);

	dev_info(udc->dev, "vusb_stop\n");

	/* Disable IRQ to CPU */
	//spi_wr8(udc, VUSB_REG_CPUCTL, 0);

}

static void __vusb_start(struct vusb_udc *udc)
{
	u8 val;

	/* Need this delay if bus-powered,
	 * but even for self-powered it helps stability
	 */
	msleep_interruptible(250);

	/* configure SPI */
	//spi_wr8(udc, VUSB_REG_PINCTL, FDUPSPI);

	/* Chip Reset */
	//spi_wr8(udc, VUSB_REG_USBCTL, CHIPRES);
	msleep_interruptible(5);
	//spi_wr8(udc, VUSB_REG_USBCTL, 0);

	/* Enable PULL-UP only when Vbus detected */
	//val = spi_rd8(udc, VUSB_REG_USBCTL);
	//val |= VBGATE | CONNECT;
	//spi_wr8(udc, VUSB_REG_USBCTL, val);

	////val = URESDNIRQ | URESIRQ;
	////if (udc->is_selfpowered)
	////	val |= NOVBUSIRQ;
	//spi_wr8(udc, VUSB_REG_USBIEN, val);

	/* Enable only EP0 interrupts */
	//val = IN0BAVIRQ | OUT0DAVIRQ | SUDAVIRQ;
	//spi_wr8(udc, VUSB_REG_EPIEN, val);

	/* Enable IRQ to CPU */
	//spi_wr8(udc, VUSB_REG_CPUCTL, IE);
}

static int vusb_start(struct vusb_udc *udc)
{
	unsigned long flags;
	int todo;

	spin_lock_irqsave(&udc->lock, flags);
	todo = udc->todo & UDC_START;
	udc->todo &= ~UDC_START;
	spin_unlock_irqrestore(&udc->lock, flags);

	if (!todo)
		return false;

	if (udc->softconnect)
		__vusb_start(udc);
	else
		__vusb_stop(udc);

	return true;
}

static void vusb_getstatus(struct vusb_udc *udc)
{
	struct vusb_ep *ep;
	u16 status = 0;

	switch (udc->setup.bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		/* Get device status */
		status = udc->gadget.is_selfpowered << USB_DEVICE_SELF_POWERED;
		status |= (udc->remote_wkp << USB_DEVICE_REMOTE_WAKEUP);
		break;
	case USB_RECIP_INTERFACE:
		if (udc->driver->setup(&udc->gadget, &udc->setup) < 0)
			goto stall;
		break;
	case USB_RECIP_ENDPOINT:
		ep = &udc->ep[udc->setup.wIndex & USB_ENDPOINT_NUMBER_MASK];
		if (udc->setup.wIndex & USB_DIR_IN) {
			if (!ep->ep_usb.caps.dir_in)
				goto stall;
		} else {
			if (!ep->ep_usb.caps.dir_out)
				goto stall;
		}
		if (ep->halted)
			status = 1 << USB_ENDPOINT_HALT;
		break;
	default:
		goto stall;
	}

	status = cpu_to_le16(status);
	//spi_wr_buf(udc, VUSB_REG_EP0FIFO, &status, 2);
	//spi_wr8_ack(udc, VUSB_REG_EP0BC, 2, 1);
	return;
stall:
	UDCVDBG(udc, "Can't respond to getstatus request\n");
	//spi_wr8(udc, VUSB_REG_EPSTALLS, STLEP0IN | STLEP0OUT | STLSTAT);
}

static void vusb_set_clear_feature(struct vusb_udc *udc)
{
	struct vusb_ep *ep;
	int set = udc->setup.bRequest == USB_REQ_SET_FEATURE;
	unsigned long flags;
	int id;

	switch (udc->setup.bRequestType) {
	case USB_RECIP_DEVICE:
		if (udc->setup.wValue != USB_DEVICE_REMOTE_WAKEUP)
			break;

		if (udc->setup.bRequest == USB_REQ_SET_FEATURE)
			udc->remote_wkp = 1;
		else
			udc->remote_wkp = 0;

		//return spi_ack_ctrl(udc);
    return;

	case USB_RECIP_ENDPOINT:
		if (udc->setup.wValue != USB_ENDPOINT_HALT)
			break;

		id = udc->setup.wIndex & USB_ENDPOINT_NUMBER_MASK;
		ep = &udc->ep[id];

		spin_lock_irqsave(&ep->lock, flags);
		ep->todo &= ~STALL_EP;
		if (set)
			ep->todo |= STALL;
		else
			ep->todo |= UNSTALL;
		spin_unlock_irqrestore(&ep->lock, flags);
		UDCVDBG(udc, "vusb_set_clear_feature: stall\n");
		schedule_work(&ep->wk_status);
		return;
	default:
		break;
	}
	UDCVDBG(udc, "vusb_set_clear_feature: Can't respond\n");
	//spi_wr8(udc, VUSB_REG_EPSTALLS, STLEP0IN | STLEP0OUT | STLSTAT);
}

void vusb_handle_setup(struct vusb_udc *udc, struct vusb_ep* ep, struct usb_ctrlrequest setup)
{
	udc->setup = setup;
	udc->setup.wValue = cpu_to_le16(setup.wValue);
	udc->setup.wIndex = cpu_to_le16(setup.wIndex);
	udc->setup.wLength = cpu_to_le16(setup.wLength);

	switch (udc->setup.bRequest) {
	case USB_REQ_GET_STATUS:
		/* Data+Status phase form udc */
		if ((udc->setup.bRequestType &
				(USB_DIR_IN | USB_TYPE_MASK)) !=
				(USB_DIR_IN | USB_TYPE_STANDARD)) {
			break;
		}
    UDCVDBG(udc, "Get status %x\n", udc->setup.wValue);
    return vusb_getstatus(udc);
	case USB_REQ_SET_ADDRESS:
		/* Status phase from udc */
		if (udc->setup.bRequestType != (USB_DIR_OUT |
				USB_TYPE_STANDARD | USB_RECIP_DEVICE)) {
			break;
		}
    // ack setaddress
    vusb_spi_pipe_ack(udc, ep->pipe);
    UDCVDBG(udc, "Assigned Address=%d, pipe: %d\n", udc->setup.wValue, ep->pipe);
		return;
	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
    UDCVDBG(udc, "Clear/Set feature wValue:%d, pipe: %d\n", udc->setup.wValue, ep->pipe);
    /* Requests with no data phase, status phase from udc */
		if ((udc->setup.bRequestType & USB_TYPE_MASK)
				!= USB_TYPE_STANDARD)
			break;
		return vusb_set_clear_feature(udc);
	default:
		break;
	}
  //UDCVDBG(udc, "handle_setup driver: %x\n", udc->driver);
	if (udc->driver != NULL && udc->driver->setup(&udc->gadget, &setup) < 0) {
    //UDCVDBG(udc, "setup error: Type: %x Request: %x\n", setup.bRequestType, setup.bRequest);
    // print the setup packet which leads to the error
    pr_hex_mark((void*)&setup, sizeof(struct usb_ctrlrequest), PRINTF_ERROR, "setup error");
  	//	/* Stall EP0 */
    //UDCVDBG(udc, "Stall EP0 %x\n", udc->gadget);
    //	//spi_wr8(udc, VUSB_REG_EPSTALLS,
	  //	//STLEP0IN | STLEP0OUT | STLSTAT);
	}

}

void vusb_req_done(struct vusb_req *req, int status)
{
	struct vusb_ep *ep = req->ep;
	struct vusb_udc *udc = ep->udc;

  //UDCVDBG(udc, "%s reqdone %p, status %d\n",
  //  ep->ep_usb.name, req, status);
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

	done = 0;
	if (ep->ep_usb.caps.dir_in) {
		prefetch(buf);
		//pr_hex_mark(buf, length, PRINTF_READ, req->ep->name);
		udc->spitransfer[0] = req->ep->port;
		udc->spitransfer[1] = req->ep->pipe;
		memmove(&udc->spitransfer[2], buf, length); // mcu pipe index
		vusb_write_buffer(udc, VUSB_REG_PIPE_WRITE_DATA, udc->spitransfer, length+2*sizeof(u8));
		if (length < psz) {
			done = 1;
		}
	}
	if (ep->ep_usb.caps.dir_out) {
		//psz = spi_rd8(udc, VUSB_REG_EP0BC + ep_id);
		length = min(length, psz);
		prefetchw(buf);
		//spi_rd_buf(udc, VUSB_REG_EP0FIFO + ep_id, buf, length);
		if (length < ep->ep_usb.maxpacket)
			done = 1;
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
			vusb_spi_pipe_ack(udc, ep->pipe);
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
			//vusb_spi_pipe_attach(udc, 1);
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

static void vusb_port_start(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_udc_work);
  struct vusb_udc* udc = ep->udc;

  u8 transfer[24];
  transfer[0] = ep->port; // port
  transfer[1] = VUSB_TYPE_CTRL; // pipe type 0 is ctrl ep
	// we need a separate function to activate ep0
#ifdef DEBUG
	if (vusb_read_buffer(udc, VUSB_REG_PIPE_EP_ENABLE, transfer, sizeof(u8) * 2)) {
		uint8_t port = transfer[0];
    dev_info(&udc->spi->dev, "Hub port %d is in configured stage", ep->port);
	}
#else
  dev_info(&udc->spi->dev, "Port %d is enabled with control pipe: %d", ep->port, ep->pipe);
#endif // _DEBUG
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
  //dev_info(&udc->spi->dev, "Hub vusb_udc_start on port:%d, index:%d",
		//			ep->port, ep->pipe);

	return 0;
}

static void vusb_port_stop(struct work_struct* work)
{
	struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_udc_work);
	struct vusb_udc* udc = ep->udc;
  dev_info(&udc->spi->dev, "Device on port %d is detached", ep->port);

  u8 transfer[24];
  transfer[0] = ep->port; // port
  transfer[1] = VUSB_TYPE_CTRL;

	if (vusb_write_buffer(udc, VUSB_REG_PORT_DETACH, transfer, sizeof(u8) * 2)) {
    dev_info(&udc->spi->dev, "Device on port %d is detached", ep->port);
  }

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
  
  INIT_WORK(&ep->wk_udc_work, vusb_port_stop);
  schedule_work(&ep->wk_udc_work);


  dev_info(&udc->spi->dev, "Hub gadget vusb_udc_stop.\n");
	return 0;
}

/*
	This function enable and reserve the pipe on the mcu 
*/
static void vusb_allocate_pipe(struct work_struct* work)
{
  struct vusb_ep* ep = container_of(work, struct vusb_ep, wk_udc_work);
  struct vusb_udc* udc = ep->udc;

	// mark the hub port as spi active
	// set the control pipe for the hub port
	if (ep->ep_usb.caps.type_control)	{
    //dev_info(&udc->spi->dev, "Hub vusb_allocate_pipe - port activation ep:%s on port:%d\n", ep->name, ep->port);
    u8 transfer[10];
    transfer[0] = ep->port; // port
    transfer[1] = VUSB_PORT_STAGE_SPI_START;// activate spi configure stage
		if (vusb_write_buffer(udc, VUSB_REG_MAP_PORT_SET, transfer, sizeof(u8) * 2)) {
      dev_info(&udc->spi->dev, "Hub port %d is now in spi stage", ep->port);
    }
    dev_info(&udc->spi->dev, "  - port: %d with cpipe/id: %d ep/name: %s\n", ep->port, ep->pipe, ep->name);
		return;
	}
	// find the next free pipe
  // enable the pipe on the mcu
	dev_info(&udc->spi->dev, "  - port: %d with  pipe/id: %d ep/name: %s\n", ep->port, ep->pipe, ep->name);

}

static struct usb_ep* vusb_match_ep(struct usb_gadget* gadget,
	struct usb_endpoint_descriptor* desc,
	struct usb_ss_ep_comp_descriptor* ep_comp)
{
  struct vusb_udc* udc = gadget_to_udc(gadget);
  struct usb_ep* _ep;
	struct vusb_ep* ep;

  /* Look at endpoints until an unclaimed one looks usable */
  list_for_each_entry(_ep, &gadget->ep_list, ep_list) {
    if (usb_gadget_ep_match_desc(gadget, _ep, desc, ep_comp))
      goto found_ep;
  }
  /* Fail */
  return NULL;

found_ep:

	if (udc->ep[0].todo == START)
	{
		unsigned int flags;
    spin_lock_irqsave(&udc->lock, flags);
		udc->ep[0].todo = 0;
    spin_unlock_irqrestore(&udc->lock, flags);

		INIT_WORK(&udc->ep[0].wk_udc_work, vusb_allocate_pipe);
    schedule_work(&udc->ep[0].wk_udc_work);

	}


	ep = ep_usb_to_vusb_ep(_ep);
  //dev_info(&udc->spi->dev, "Hub vusb_match_ep ep0:%s\n", ep->name);

  INIT_WORK(&ep->wk_udc_work, vusb_allocate_pipe);
  schedule_work(&ep->wk_udc_work);

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

  dev_info(&udc->spi->dev, "Spi clock set at %u KHz.\n",
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

	//udc->ep0req.ep = &udc->ep[0];
	//udc->ep0req.usb_req.buf = udc->ep0buf;
	//INIT_LIST_HEAD(&udc->ep0req.queue);

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
  if (rc)
  {
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
