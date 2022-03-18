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

/* Forward declaration */
static int vusb_remove(struct spi_device* spi);
struct vusb_ep* vusb_get_ep(struct vusb_udc* udc, u8 ep_idx);

static void vusb_getstatus(struct vusb_ep* ep)
{
	u16 status = 0;
	struct vusb_udc* udc = ep->udc;
	struct vusb_port_dev* dev = ep->dev;

	switch (ep->setup.bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		/* Get device status */
		status = dev->gadget.is_selfpowered << USB_DEVICE_SELF_POWERED;
		status |= (udc->remote_wkp << USB_DEVICE_REMOTE_WAKEUP);
		break;
	case USB_RECIP_INTERFACE:
		//UDCVDBG(udc, "** Get status IRQ RESET raised on port:%d\n", d->driver, udc->driver);
		if (dev->driver->setup(&dev->gadget, &ep->setup) < 0)
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
	struct vusb_port_dev* dev = ep->dev;
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
		_ep = &dev->ep[id];

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
	struct vusb_port_dev* dev = ep->dev;

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
    UDCVDBG(udc, "Clear/Set feature wValue:%d, ep/idx: %d\n", ep->setup.wValue, ep->idx);
    /* Requests with no data phase, status phase from udc */
		if ((ep->setup.bRequestType & USB_TYPE_MASK)
				!= USB_TYPE_STANDARD)
			break;
		return vusb_set_clear_feature(ep);
	default:
		//UDCVDBG(udc, "Default vusb_handle_setup request: %d\n", ep->setup.bRequest);
		break;
	}
	if (dev->driver != NULL && dev->driver->setup(&dev->gadget, &ep->setup) < 0) {
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
	spi_cmd_t* cmd;
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

	if (ep->dir == USB_DIR_BOTH) {
		// OUT setup packet
		if ( ep->ep0_dir == USB_DIR_OUT && ep->setup.wLength)	{
      udc->spitransfer[0] = REG_PIPE_FIFO;
      udc->spitransfer[1] = req->ep->idx;
      vusb_read_buffer(udc, VUSB_REG_MAP_PIPE_GET, udc->spitransfer, length + 2 * sizeof(u8));
      cmd = (spi_cmd_t*)udc->spitransfer;
      prefetchw(buf);
      memmove(buf, cmd->data, length); // mcu pipe index
      //pr_hex_mark_debug(buf, length, PRINTF_READ, req->ep->name, "Get - OUT");
		}	else {
      udc->spitransfer[0] = REG_PIPE_FIFO;
      udc->spitransfer[1] = req->ep->idx;
      prefetch(buf);
      memmove(&udc->spitransfer[2], buf, length); // mcu pipe index
      vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, udc->spitransfer, length + 2 * sizeof(u8));
      //pr_hex_mark_debug(buf, length, PRINTF_READ, req->ep->name, "Get - IN");
		}
		if (length < psz) {
			done = 1;
		}

	} 
  if (ep->dir == USB_DIR_OUT) {
    u8 transfer[80];
	//	pr_hex_mark_debug(buf, length, PRINTF_READ, req->ep->name, "EP-OUT");
		length = min(length, psz);
    // OUT data from the mcu...
    transfer[0] = REG_PIPE_FIFO; // write&read register
    transfer[1] = ep->idx; // pipe
    vusb_read_buffer(ep->udc, VUSB_REG_MAP_PIPE_GET, transfer, 1 + 2 * sizeof(u8));
    cmd = (spi_cmd_t*)transfer;
		length = cmd->length; 
    //UDCVDBG(ep->udc, "vusb_do_data - EP-OUT, name: %s, ep/idx: %d, length: %d\n", ep->name, ep->idx, length);
		prefetchw(buf);
		memmove(buf, cmd->data, length);
    ////pr_hex_mark_debug(buf, length, PRINTF_READ, req->ep->name, "EP-OUT");
		//if (length < psz)
			done = 1;
	}

	if (ep->dir == USB_DIR_IN) {
    udc->spitransfer[0] = REG_PIPE_FIFO;
    udc->spitransfer[1] = req->ep->idx; // mcu pipe index
		prefetch(buf);
    memmove(&udc->spitransfer[2], buf, length); 
    vusb_write_buffer(udc, VUSB_REG_MAP_PIPE_SET, udc->spitransfer, length + 2 * sizeof(u8));
    if (length < psz)
      done = 1;
  }

	req->usb_req.actual += length;


	if (req->usb_req.actual == req->usb_req.length)
		done = 1;

xfer_done:
	if (done) { 
		unsigned long flags;

		spin_lock_irqsave(&ep->lock, flags);
		list_del_init(&req->queue);
		spin_unlock_irqrestore(&ep->lock, flags);

	if (ep->dir == USB_DIR_OUT) {
		UDCVDBG(udc, "vusb_do_data, name: %s, length: %d, actual:%d\n", ep->name, length, req->usb_req.actual);
		pr_hex_mark_debug(req->usb_req.buf, 64, PRINTF_READ, req->ep->name, "EP-OUT");
	}
		// ack read and write
		if (ep->dir != USB_DIR_OUT)	{
			vusb_spi_pipe_ack(udc, ep);
		}
		vusb_req_done(req, 0);
		return false;
	}
	return true;
}

#define REG_USBIRQ	3
#define REG_IRQ_ELEMENTS 12

static void vusb_irq_mcu_handler(struct work_struct* work)
{
	u8 transfer[64];
	struct vusb_udc* udc = container_of(work, struct vusb_udc, vusb_irq_wq_mcu);

	if (vusb_read_buffer(udc, VUSB_REG_IRQ_GET, transfer, REG_IRQ_ELEMENTS)) {
		u8 portirq, usbirq;
		u32 pipeirq;
		vusb_req_map_t irq_map;
		
		memmove(&irq_map, transfer + VUSB_SPI_HEADER, REG_IRQ_ELEMENTS);
		portirq = irq_map.PRTIRQ & irq_map.PRTIEN;
		// port handling
		if (portirq) {
			transfer[0] = REG_PRTIRQ;
			transfer[1] = portirq;
			vusb_write_buffer(udc, VUSB_REG_IRQ_CLEAR, transfer, 2);
			while (hweight32(portirq)) {
				u16 port = _bf_ffsl(portirq);
				struct vusb_port_dev* d = &udc->ports[port - 1].dev;
				//UDCVDBG(udc, "** MCU handler IRQ RESET raised on port:%p, %p\n", d->driver, udc->driver);
				// use reset only if gadget is used before
				if(d->driver)
					usb_gadget_udc_reset(&d->gadget, d->driver);
				portirq &= ~BIT(port);
			}
		}

	// pipe handling
		irq_map.PIPIEN = htonl(irq_map.PIPIEN);
		irq_map.PIPIRQ = htonl(irq_map.PIPIRQ);
		pipeirq = irq_map.PIPIEN & irq_map.PIPIRQ;
		if (pipeirq) {
			// clear pipeirq flags
			transfer[0] = REG_PIPEIRQ;
			*(u32*)&transfer[1] = htonl(irq_map.PIPIRQ);
			vusb_write_buffer(udc, VUSB_REG_IRQ_CLEAR, transfer, sizeof(u8) + sizeof(u32));
			// process the irqs
			while (hweight32(pipeirq)) {
				u32 irq = _bf_ffsl(pipeirq);
				struct vusb_ep* ep = vusb_get_ep(udc, irq);
				if (ep != NULL) {
					queue_work(udc->irq_work_data, &ep->wk_irq_data);
				}
				pipeirq &= ~BIT(irq);
			}
		}

		usbirq = irq_map.USBIRQ & irq_map.USBIEN;
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
    struct irq_chip* chip = irq_desc_get_chip(desc);
    if (chip && udc->connected && list_empty(&udc->vusb_irq_wq_mcu.entry))
    {
			queue_work(udc->irq_work_mcu, &udc->vusb_irq_wq_mcu);
    }
  }
  return iret;
}

#define VUSB_TYPE_CTRL	0
#define VUSB_TYPE_INT		1
#define DEBUG

static int vusb_probe(struct spi_device *spi)
{
	struct vusb_udc *udc;
	dev_t usrdev;
	int rc = 0, i;

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
    dev_err(&spi->dev, "Unable to get hub downstreams ports.\n");
    return -ENOMEM;
  }
	
	udc->ports = devm_kcalloc(&spi->dev, udc->max_ports, sizeof(struct vusb_port), GFP_KERNEL);
	if (!udc->ports)
		return -ENOMEM;

	dev_info(&spi->dev, "Hub device has initiated %d hub ports.\n", udc->max_ports);

	dev_info(&spi->dev, "Spi clock set at %u KHz.\n",
                 (udc->spi->max_speed_hz + 500) / 1000);

  spin_lock_init(&udc->lock);
  spin_lock_init(&udc->wq_lock);

  /* INTERRUPT spi read queue */
  init_waitqueue_head(&udc->spi_read_queue);
	mutex_init(&udc->spi_read_mutex);
	mutex_init(&udc->spi_write_mutex);

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
  //dev_info(&udc->spi->dev, "Reset gpio is defined as gpio:%p\n", udc->mcu_gpreset);
  gpiod_set_value(udc->mcu_gpreset, 1);
#endif

#ifdef _DEBUG
	gpiod_set_value(udc->mcu_gpreset, 0);
	msleep_interruptible(500);
	gpiod_set_value(udc->mcu_gpreset, 1);
	msleep_interruptible(100);
#endif // _DEBUG

  // char device
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
	
	// worker queue for irqs
	udc->irq_work_mcu = create_singlethread_workqueue("vusb_irq_mcu");
	udc->irq_work_data = create_singlethread_workqueue("vusb_irq_data");
	INIT_WORK(&udc->vusb_irq_wq_mcu, vusb_irq_mcu_handler);

	/* we have 32 pipes to use, starting with pipe id 2 */
	for (i = 0; i < VUSB_MAX_PIPES; i++) {
		udc->pipes[i].ep = NULL;
		udc->pipes[i].used = false;
		udc->pipes[i].enabled = false;
		udc->pipes[i].pipe_idx = i + VUSB_MCU_PIPE_ID; // start pipe index is + 2 ...
	}

	/* setup ports */
	for (i = 0; i < udc->max_ports && rc == 0; i++)
		rc = vusb_port_init(udc, i);
	if (rc)
		goto err;

	// for the pi shield
	udc->is_selfpowered = 1; 
	// driver is ready to use
	udc->connected = true;

	return 0;
err:
  vusb_remove(spi);
  dev_err(&spi->dev, "Failed to initialize vusb.\n");
  return rc;
}

static int vusb_remove(struct spi_device *spi)
{
	dev_t dev_id;
	u8 i;
	struct vusb_udc *udc = spi_get_drvdata(spi);

  dev_info(&spi->dev, "Removing vusb hub...\n");

  disable_irq(udc->mcu_irq);
  disable_irq(udc->spi_datrdy);
	udc->connected = false;

  cdev_del(&udc->cdev);

	if (udc->irq_work_mcu) {
		flush_workqueue(udc->irq_work_mcu);
		destroy_workqueue(udc->irq_work_mcu);
	}
	if (udc->irq_work_data) {
		flush_workqueue(udc->irq_work_data);
		destroy_workqueue(udc->irq_work_data);
	}

  // remove the char device
  device_destroy(udc->chardev_class, MKDEV(udc->crdev_major, 1));
  class_destroy(udc->chardev_class);

  dev_id = MKDEV(udc->crdev_major, 0);
  unregister_chrdev_region(dev_id, VUSB_MAX_CHAR_DEVICES);

  dev_info(&spi->dev, "Char device from v-hub removed.\n");

	// remove port devices
	for (i = 0; i < udc->max_ports; i++) {
		struct vusb_port_dev* dev = &udc->ports[i].dev;
		if (dev->registered) {
			dev->registered = false;
			usb_del_gadget_udc(&dev->gadget);
			device_unregister(dev->port_dev);
		}
	}

	return 0;
}

static void vusb_shutdown(struct spi_device* spi)
{
  dev_info(&spi->dev, "*** vusb_shutdown\n");
}

static const struct of_device_id vusb_udc_of_match[] = {
  {.compatible = "hubshield,v-hub", },
  {},
};

MODULE_DEVICE_TABLE(of, vusb_udc_of_match);

static struct spi_driver vusb_driver = {
	.probe = vusb_probe,
	.remove = vusb_remove,
  .shutdown = vusb_shutdown,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_match_ptr(vusb_udc_of_match),
	},

};

module_spi_driver(vusb_driver);
MODULE_DESCRIPTION("USB hub udc driver");
MODULE_AUTHOR("Manfred Kubica <manfredkubica@web.de>");
MODULE_LICENSE("GPL");
