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
#define to_udc(g)		container_of((g), struct vusb_udc, gadget)

static const char driver_name[] = "vusb-udc";

/* Forward declaration */
static int vusb_remove(struct spi_device* spi);

/* Control endpoint configuration.*/
static const struct usb_endpoint_descriptor ep0_desc = {
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize		= cpu_to_le16(VUSB_EP_MAX_PACKET),
};

static void spi_ack_ctrl(struct vusb_udc *udc)
{
	struct spi_device *spi = udc->spi;
	struct spi_transfer transfer;
	struct spi_message msg;
	u8 txdata[1];

	memset(&transfer, 0, sizeof(transfer));

	spi_message_init(&msg);

	txdata[0] = VUSB_ACKSTAT;
	transfer.tx_buf = txdata;
	transfer.len = 1;

	spi_message_add_tail(&transfer, &msg);
	spi_sync(spi, &msg);
}

static u8 spi_rd8_ack(struct vusb_udc *udc, u8 reg, int actstat)
{
	struct spi_device *spi = udc->spi;
	struct spi_transfer transfer;
	struct spi_message msg;
	u8 txdata[2], rxdata[2];

	memset(&transfer, 0, sizeof(transfer));

	spi_message_init(&msg);

	txdata[0] = VUSB_SPI_CMD_RD(reg) | (actstat ? VUSB_ACKSTAT : 0);
	transfer.tx_buf = txdata;
	transfer.rx_buf = rxdata;
	transfer.len = 2;

	spi_message_add_tail(&transfer, &msg);
	spi_sync(spi, &msg);

	return rxdata[1];
}

static u8 spi_rd8(struct vusb_udc *udc, u8 reg)
{
	return spi_rd8_ack(udc, reg, 0);
}

static void spi_wr8_ack(struct vusb_udc *udc, u8 reg, u8 val, int actstat)
{
	struct spi_device *spi = udc->spi;
	struct spi_transfer transfer;
	struct spi_message msg;
	u8 txdata[2];

	memset(&transfer, 0, sizeof(transfer));

	spi_message_init(&msg);

	txdata[0] = VUSB_SPI_CMD_WR(reg) | (actstat ? VUSB_ACKSTAT : 0);
	txdata[1] = val;

	transfer.tx_buf = txdata;
	transfer.len = 2;

	spi_message_add_tail(&transfer, &msg);
	spi_sync(spi, &msg);
}

static void spi_wr8(struct vusb_udc *udc, u8 reg, u8 val)
{
	spi_wr8_ack(udc, reg, val, 0);
}

static void spi_rd_buf(struct vusb_udc *udc, u8 reg, void *buf, u8 len)
{
	struct spi_device *spi = udc->spi;
	struct spi_transfer transfer;
	struct spi_message msg;
	u8 local_buf[VUSB_EP_MAX_PACKET + 1] = {};

	memset(&transfer, 0, sizeof(transfer));

	spi_message_init(&msg);

	local_buf[0] = VUSB_SPI_CMD_RD(reg);
	transfer.tx_buf = &local_buf[0];
	transfer.rx_buf = &local_buf[0];
	transfer.len = len + 1;

	spi_message_add_tail(&transfer, &msg);
	spi_sync(spi, &msg);

	memcpy(buf, &local_buf[1], len);
}

static void spi_wr_buf(struct vusb_udc *udc, u8 reg, void *buf, u8 len)
{
	struct spi_device *spi = udc->spi;
	struct spi_transfer transfer;
	struct spi_message msg;
	u8 local_buf[VUSB_EP_MAX_PACKET + 1] = {};

	memset(&transfer, 0, sizeof(transfer));

	spi_message_init(&msg);

	local_buf[0] = VUSB_SPI_CMD_WR(reg);
	memcpy(&local_buf[1], buf, len);

	transfer.tx_buf = local_buf;
	transfer.len = len + 1;

	spi_message_add_tail(&transfer, &msg);
	spi_sync(spi, &msg);
}

static int spi_vusb_enable(struct vusb_ep *ep)
{
	struct vusb_udc *udc = ep->udc;
	unsigned long flags;
	u8 epdis, epien;
	int todo;

	spin_lock_irqsave(&ep->lock, flags);
	todo = ep->todo & ENABLE_EP;
	ep->todo &= ~ENABLE_EP;
	spin_unlock_irqrestore(&ep->lock, flags);

	if (!todo || ep->id == 0)
		return false;

	epien = spi_rd8(udc, VUSB_REG_EPIEN);
	epdis = spi_rd8(udc, VUSB_REG_CLRTOGS);

	if (todo == ENABLE) {
		epdis &= ~BIT(ep->id + 4);
		epien |= BIT(ep->id + 1);
	} else {
		epdis |= BIT(ep->id + 4);
		epien &= ~BIT(ep->id + 1);
	}

	spi_wr8(udc, VUSB_REG_CLRTOGS, epdis);
	spi_wr8(udc, VUSB_REG_EPIEN, epien);

	return true;
}

static int spi_vusb_stall(struct vusb_ep *ep)
{
	struct vusb_udc *udc = ep->udc;
	unsigned long flags;
	u8 epstalls;
	int todo;

	spin_lock_irqsave(&ep->lock, flags);
	todo = ep->todo & STALL_EP;
	ep->todo &= ~STALL_EP;
	spin_unlock_irqrestore(&ep->lock, flags);

	if (!todo || ep->id == 0)
		return false;

	epstalls = spi_rd8(udc, VUSB_REG_EPSTALLS);
	if (todo == STALL) {
		ep->halted = 1;
		epstalls |= BIT(ep->id + 1);
	} else {
		u8 clrtogs;

		ep->halted = 0;
		epstalls &= ~BIT(ep->id + 1);
		clrtogs = spi_rd8(udc, VUSB_REG_CLRTOGS);
		clrtogs |= BIT(ep->id + 1);
		spi_wr8(udc, VUSB_REG_CLRTOGS, clrtogs);
	}
	spi_wr8(udc, VUSB_REG_EPSTALLS, epstalls | ACKSTAT);

	return true;
}

static int spi_vusb_rwkup(struct vusb_udc *udc)
{
	unsigned long flags;
	int wake_remote;
	u8 usbctl;

	spin_lock_irqsave(&udc->lock, flags);
	wake_remote = udc->todo & REMOTE_WAKEUP;
	udc->todo &= ~REMOTE_WAKEUP;
	spin_unlock_irqrestore(&udc->lock, flags);

	if (!wake_remote || !udc->suspended)
		return false;

	/* Set Remote-WkUp Signal*/
	usbctl = spi_rd8(udc, VUSB_REG_USBCTL);
	usbctl |= SIGRWU;
	spi_wr8(udc, VUSB_REG_USBCTL, usbctl);

	msleep_interruptible(5);

	/* Clear Remote-WkUp Signal*/
	usbctl = spi_rd8(udc, VUSB_REG_USBCTL);
	usbctl &= ~SIGRWU;
	spi_wr8(udc, VUSB_REG_USBCTL, usbctl);

	udc->suspended = false;

	return true;
}

static void vusb_nuke(struct vusb_ep *ep, int status);
static void __vusb_stop(struct vusb_udc *udc)
{
	u8 val;
	int i;

	/* clear all pending requests */
	for (i = 1; i < VUSB_MAX_EPS; i++)
		vusb_nuke(&udc->ep[i], -ECONNRESET);

	/* Disable IRQ to CPU */
	spi_wr8(udc, VUSB_REG_CPUCTL, 0);

	val = spi_rd8(udc, VUSB_REG_USBCTL);
	val |= PWRDOWN;
	if (udc->is_selfpowered)
		val &= ~HOSCSTEN;
	else
		val |= HOSCSTEN;
	spi_wr8(udc, VUSB_REG_USBCTL, val);
}

static void __vusb_start(struct vusb_udc *udc)
{
	u8 val;

	/* Need this delay if bus-powered,
	 * but even for self-powered it helps stability
	 */
	msleep_interruptible(250);

	/* configure SPI */
	spi_wr8(udc, VUSB_REG_PINCTL, FDUPSPI);

	/* Chip Reset */
	spi_wr8(udc, VUSB_REG_USBCTL, CHIPRES);
	msleep_interruptible(5);
	spi_wr8(udc, VUSB_REG_USBCTL, 0);

	/* Poll for OSC to stabilize */
	while (1) {
		val = spi_rd8(udc, VUSB_REG_USBIRQ);
		if (val & OSCOKIRQ)
			break;
		cond_resched();
	}

	/* Enable PULL-UP only when Vbus detected */
	val = spi_rd8(udc, VUSB_REG_USBCTL);
	val |= VBGATE | CONNECT;
	spi_wr8(udc, VUSB_REG_USBCTL, val);

	val = URESDNIRQ | URESIRQ;
	if (udc->is_selfpowered)
		val |= NOVBUSIRQ;
	spi_wr8(udc, VUSB_REG_USBIEN, val);

	/* Enable only EP0 interrupts */
	val = IN0BAVIRQ | OUT0DAVIRQ | SUDAVIRQ;
	spi_wr8(udc, VUSB_REG_EPIEN, val);

	/* Enable IRQ to CPU */
	spi_wr8(udc, VUSB_REG_CPUCTL, IE);
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

static irqreturn_t vusb_irq_handler(int irq, void *dev_id)
{
	struct vusb_udc *udc = dev_id;
	struct spi_device *spi = udc->spi;
	unsigned long flags;

	spin_lock_irqsave(&udc->lock, flags);
	if ((udc->todo & ENABLE_IRQ) == 0) {
		disable_irq_nosync(spi->irq);
		udc->todo |= ENABLE_IRQ;
	}
	spin_unlock_irqrestore(&udc->lock, flags);

	if (udc->thread_task &&
	    udc->thread_task->state != TASK_RUNNING)
		wake_up_process(udc->thread_task);

	return IRQ_HANDLED;
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
	spi_wr_buf(udc, VUSB_REG_EP0FIFO, &status, 2);
	spi_wr8_ack(udc, VUSB_REG_EP0BC, 2, 1);
	return;
stall:
	dev_err(udc->dev, "Can't respond to getstatus request\n");
	spi_wr8(udc, VUSB_REG_EPSTALLS, STLEP0IN | STLEP0OUT | STLSTAT);
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

		return spi_ack_ctrl(udc);

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

		spi_vusb_stall(ep);
		return;
	default:
		break;
	}

	dev_err(udc->dev, "Can't respond to SET/CLEAR FEATURE\n");
	spi_wr8(udc, VUSB_REG_EPSTALLS, STLEP0IN | STLEP0OUT | STLSTAT);
}

static void vusb_handle_setup(struct vusb_udc *udc)
{
	struct usb_ctrlrequest setup;

	spi_rd_buf(udc, VUSB_REG_SUDFIFO, (void *)&setup, 8);

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
		return vusb_getstatus(udc);
	case USB_REQ_SET_ADDRESS:
		/* Status phase from udc */
		if (udc->setup.bRequestType != (USB_DIR_OUT |
				USB_TYPE_STANDARD | USB_RECIP_DEVICE)) {
			break;
		}
		spi_rd8_ack(udc, VUSB_REG_FNADDR, 1);
		dev_dbg(udc->dev, "Assigned Address=%d\n", udc->setup.wValue);
		return;
	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
		/* Requests with no data phase, status phase from udc */
		if ((udc->setup.bRequestType & USB_TYPE_MASK)
				!= USB_TYPE_STANDARD)
			break;
		return vusb_set_clear_feature(udc);
	default:
		break;
	}

	if (udc->driver->setup(&udc->gadget, &setup) < 0) {
		/* Stall EP0 */
		spi_wr8(udc, VUSB_REG_EPSTALLS,
			STLEP0IN | STLEP0OUT | STLSTAT);
	}
}

static void vusb_req_done(struct vusb_req *req, int status)
{
	struct vusb_ep *ep = req->ep;
	struct vusb_udc *udc = ep->udc;

	if (req->usb_req.status == -EINPROGRESS)
		req->usb_req.status = status;
	else
		status = req->usb_req.status;

	if (status && status != -ESHUTDOWN)
		dev_err(udc->dev, "%s done %p, status %d\n",
			ep->ep_usb.name, req, status);

	if (req->usb_req.complete)
		req->usb_req.complete(&ep->ep_usb, &req->usb_req);
}

static int vusb_do_data(struct vusb_udc *udc, int ep_id, int in)
{
	struct vusb_ep *ep = &udc->ep[ep_id];
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
	if (in) {
		prefetch(buf);
		spi_wr_buf(udc, VUSB_REG_EP0FIFO + ep_id, buf, length);
		spi_wr8(udc, VUSB_REG_EP0BC + ep_id, length);
		if (length < psz)
			done = 1;
	} else {
		psz = spi_rd8(udc, VUSB_REG_EP0BC + ep_id);
		length = min(length, psz);
		prefetchw(buf);
		spi_rd_buf(udc, VUSB_REG_EP0FIFO + ep_id, buf, length);
		if (length < ep->ep_usb.maxpacket)
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

		if (ep_id == 0)
			spi_ack_ctrl(udc);

		vusb_req_done(req, 0);
	}

	return true;
}

static int vusb_handle_irqs(struct vusb_udc *udc)
{
	u8 epien, epirq, usbirq, usbien, reg[4];
	bool ret = false;

	spi_rd_buf(udc, VUSB_REG_EPIRQ, reg, 4);
	epirq = reg[0];
	epien = reg[1];
	usbirq = reg[2];
	usbien = reg[3];

	usbirq &= usbien;
	epirq &= epien;

	if (epirq & SUDAVIRQ) {
		spi_wr8(udc, VUSB_REG_EPIRQ, SUDAVIRQ);
		vusb_handle_setup(udc);
		return true;
	}

	if (usbirq & VBUSIRQ) {
		spi_wr8(udc, VUSB_REG_USBIRQ, VBUSIRQ);
		dev_dbg(udc->dev, "Cable plugged in\n");
		return true;
	}

	if (usbirq & NOVBUSIRQ) {
		spi_wr8(udc, VUSB_REG_USBIRQ, NOVBUSIRQ);
		dev_dbg(udc->dev, "Cable pulled out\n");
		return true;
	}

	if (usbirq & URESIRQ) {
		spi_wr8(udc, VUSB_REG_USBIRQ, URESIRQ);
		dev_dbg(udc->dev, "USB Reset - Start\n");
		return true;
	}

	if (usbirq & URESDNIRQ) {
		spi_wr8(udc, VUSB_REG_USBIRQ, URESDNIRQ);
		dev_dbg(udc->dev, "USB Reset - END\n");
		spi_wr8(udc, VUSB_REG_USBIEN, URESDNIRQ | URESIRQ);
		spi_wr8(udc, VUSB_REG_EPIEN, SUDAVIRQ | IN0BAVIRQ
			| OUT0DAVIRQ);
		return true;
	}

	if (usbirq & SUSPIRQ) {
		spi_wr8(udc, VUSB_REG_USBIRQ, SUSPIRQ);
		dev_dbg(udc->dev, "USB Suspend - Enter\n");
		udc->suspended = true;
		return true;
	}

	if (usbirq & BUSACTIRQ) {
		spi_wr8(udc, VUSB_REG_USBIRQ, BUSACTIRQ);
		dev_dbg(udc->dev, "USB Suspend - Exit\n");
		udc->suspended = false;
		return true;
	}

	if (usbirq & RWUDNIRQ) {
		spi_wr8(udc, VUSB_REG_USBIRQ, RWUDNIRQ);
		dev_dbg(udc->dev, "Asked Host to wakeup\n");
		return true;
	}

	if (usbirq & OSCOKIRQ) {
		spi_wr8(udc, VUSB_REG_USBIRQ, OSCOKIRQ);
		dev_dbg(udc->dev, "Osc stabilized, start work\n");
		return true;
	}

	if (epirq & OUT0DAVIRQ && vusb_do_data(udc, 0, 0)) {
		spi_wr8_ack(udc, VUSB_REG_EPIRQ, OUT0DAVIRQ, 1);
		ret = true;
	}

	if (epirq & IN0BAVIRQ && vusb_do_data(udc, 0, 1))
		ret = true;

	if (epirq & OUT1DAVIRQ && vusb_do_data(udc, 1, 0)) {
		spi_wr8_ack(udc, VUSB_REG_EPIRQ, OUT1DAVIRQ, 1);
		ret = true;
	}

	if (epirq & IN2BAVIRQ && vusb_do_data(udc, 2, 1))
		ret = true;

	if (epirq & IN3BAVIRQ && vusb_do_data(udc, 3, 1))
		ret = true;

	return ret;
}

static int vusb_thread(void *dev_id)
{
	struct vusb_udc *udc = dev_id;
	struct spi_device *spi = udc->spi;
	int i, loop_again = 1;
	unsigned long flags;

	while (!kthread_should_stop()) {
		if (!loop_again) {
			ktime_t kt = ns_to_ktime(1000 * 1000 * 250); /* 250ms */

			set_current_state(TASK_INTERRUPTIBLE);

			spin_lock_irqsave(&udc->lock, flags);
			if (udc->todo & ENABLE_IRQ) {
				enable_irq(spi->irq);
				udc->todo &= ~ENABLE_IRQ;
			}
			spin_unlock_irqrestore(&udc->lock, flags);

			schedule_hrtimeout(&kt, HRTIMER_MODE_REL);
		}
		loop_again = 0;

		mutex_lock(&udc->spi_bus_mutex);

		/* If disconnected */
		if (!udc->softconnect)
			goto loop;

		if (vusb_start(udc)) {
			loop_again = 1;
			goto loop;
		}

		if (vusb_handle_irqs(udc)) {
			loop_again = 1;
			goto loop;
		}

		if (spi_vusb_rwkup(udc)) {
			loop_again = 1;
			goto loop;
		}

		vusb_do_data(udc, 0, 1); /* get done with the EP0 ZLP */

		for (i = 1; i < VUSB_MAX_EPS; i++) {
			struct vusb_ep *ep = &udc->ep[i];

			if (spi_vusb_enable(ep))
				loop_again = 1;
			if (spi_vusb_stall(ep))
				loop_again = 1;
		}
loop:
		mutex_unlock(&udc->spi_bus_mutex);
	}

	set_current_state(TASK_RUNNING);
	dev_info(udc->dev, "SPI thread exiting\n");
	return 0;
}

static int vusb_ep_set_halt(struct usb_ep *_ep, int stall)
{
	struct vusb_ep *ep = to_vusb_ep(_ep);
	struct vusb_udc *udc = ep->udc;
	unsigned long flags;

	spin_lock_irqsave(&ep->lock, flags);

	ep->todo &= ~STALL_EP;
	if (stall)
		ep->todo |= STALL;
	else
		ep->todo |= UNSTALL;

	spin_unlock_irqrestore(&ep->lock, flags);

	wake_up_process(udc->thread_task);

	dev_dbg(udc->dev, "%sStall %s\n", stall ? "" : "Un", ep->name);
	return 0;
}

static int __vusb_ep_enable(struct vusb_ep *ep,
			       const struct usb_endpoint_descriptor *desc)
{
	unsigned int maxp = usb_endpoint_maxp(desc);
	unsigned long flags;

	spin_lock_irqsave(&ep->lock, flags);
	ep->ep_usb.desc = desc;
	ep->ep_usb.maxpacket = maxp;

	ep->todo &= ~ENABLE_EP;
	ep->todo |= ENABLE;
	spin_unlock_irqrestore(&ep->lock, flags);

	return 0;
}

static int vusb_ep_enable(struct usb_ep *_ep,
			     const struct usb_endpoint_descriptor *desc)
{
	struct vusb_ep *ep = to_vusb_ep(_ep);
	struct vusb_udc *udc = ep->udc;

	__vusb_ep_enable(ep, desc);

	wake_up_process(udc->thread_task);

	return 0;
}

static void vusb_nuke(struct vusb_ep *ep, int status)
{
	struct vusb_req *req, *r;
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

static void __vusb_ep_disable(struct vusb_ep *ep)
{
	struct vusb_udc *udc = ep->udc;
	unsigned long flags;

	spin_lock_irqsave(&ep->lock, flags);

	ep->ep_usb.desc = NULL;

	ep->todo &= ~ENABLE_EP;
	ep->todo |= DISABLE;

	spin_unlock_irqrestore(&ep->lock, flags);

	dev_dbg(udc->dev, "Disabled %s\n", ep->name);
}

static int vusb_ep_disable(struct usb_ep *_ep)
{
	struct vusb_ep *ep = to_vusb_ep(_ep);
	struct vusb_udc *udc = ep->udc;

	vusb_nuke(ep, -ESHUTDOWN);

	__vusb_ep_disable(ep);

	wake_up_process(udc->thread_task);

	return 0;
}

static struct usb_request *vusb_alloc_request(struct usb_ep *_ep,
						 gfp_t gfp_flags)
{
	struct vusb_ep *ep = to_vusb_ep(_ep);
	struct vusb_req *req;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		return NULL;

	req->ep = ep;

	return &req->usb_req;
}

static void vusb_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	kfree(to_vusb_req(_req));
}

static int vusb_ep_queue(struct usb_ep *_ep, struct usb_request *_req,
			    gfp_t ignored)
{
	struct vusb_req *req = to_vusb_req(_req);
	struct vusb_ep *ep  = to_vusb_ep(_ep);
	struct vusb_udc *udc = ep->udc;
	unsigned long flags;

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	spin_lock_irqsave(&ep->lock, flags);
	list_add_tail(&req->queue, &ep->queue);
	spin_unlock_irqrestore(&ep->lock, flags);

	wake_up_process(udc->thread_task);
	return 0;
}

static int vusb_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct vusb_req *t, *req = to_vusb_req(_req);
	struct vusb_ep *ep = to_vusb_ep(_ep);
	unsigned long flags;

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
	.enable		= vusb_ep_enable,
	.disable	= vusb_ep_disable,
	.alloc_request	= vusb_alloc_request,
	.free_request	= vusb_free_request,
	.queue		= vusb_ep_queue,
	.dequeue	= vusb_ep_dequeue,
	.set_halt	= vusb_ep_set_halt,
};

static int vusb_wakeup(struct usb_gadget *gadget)
{
	struct vusb_udc *udc = to_udc(gadget);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);

	/* Only if wakeup allowed by host */
	if (udc->remote_wkp) {
		udc->todo |= REMOTE_WAKEUP;
		ret = 0;
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	if (udc->thread_task &&
	    udc->thread_task->state != TASK_RUNNING)
		wake_up_process(udc->thread_task);
	return ret;
}

static int vusb_udc_start(struct usb_gadget *gadget,
			     struct usb_gadget_driver *driver)
{
	struct vusb_udc *udc = to_udc(gadget);
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

	if (udc->thread_task &&
	    udc->thread_task->state != TASK_RUNNING)
		wake_up_process(udc->thread_task);

	return 0;
}

static int vusb_udc_stop(struct usb_gadget *gadget)
{
	struct vusb_udc *udc = to_udc(gadget);
	unsigned long flags;

	spin_lock_irqsave(&udc->lock, flags);
	udc->is_selfpowered = udc->gadget.is_selfpowered;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->driver = NULL;
	udc->softconnect = false;
	udc->todo |= UDC_START;
	spin_unlock_irqrestore(&udc->lock, flags);

	if (udc->thread_task &&
	    udc->thread_task->state != TASK_RUNNING)
		wake_up_process(udc->thread_task);

	return 0;
}

static const struct usb_gadget_ops vusb_udc_ops = {
	.udc_start	= vusb_udc_start,
	.udc_stop	= vusb_udc_stop,
	.wakeup		= vusb_wakeup,
};

static void vusb_eps_init(struct vusb_udc *udc)
{
	int idx;

	INIT_LIST_HEAD(&udc->gadget.ep_list);

	for (idx = 0; idx < VUSB_MAX_EPS; idx++) {
		struct vusb_ep *ep = &udc->ep[idx];

		spin_lock_init(&ep->lock);
		INIT_LIST_HEAD(&ep->queue);

		ep->udc = udc;
		ep->id = idx;
		ep->halted = 0;
		ep->maxpacket = 0;
		ep->ep_usb.name = ep->name;
		ep->ep_usb.ops = &vusb_ep_ops;
		usb_ep_set_maxpacket_limit(&ep->ep_usb, VUSB_EP_MAX_PACKET);

		if (idx == 0) { /* For EP0 */
			ep->ep_usb.desc = &ep0_desc;
			ep->ep_usb.maxpacket = usb_endpoint_maxp(&ep0_desc);
			ep->ep_usb.caps.type_control = true;
			ep->ep_usb.caps.dir_in = true;
			ep->ep_usb.caps.dir_out = true;
			snprintf(ep->name, VUSB_EPNAME_SIZE, "ep0");
			continue;
		}

		if (idx == 1) { /* EP1 is OUT */
			ep->ep_usb.caps.dir_in = false;
			ep->ep_usb.caps.dir_out = true;
			snprintf(ep->name, VUSB_EPNAME_SIZE, "ep1-bulk-out");
		} else { /* EP2 & EP3 are IN */
			ep->ep_usb.caps.dir_in = true;
			ep->ep_usb.caps.dir_out = false;
			snprintf(ep->name, VUSB_EPNAME_SIZE,
				 "ep%d-bulk-in", idx);
		}
		ep->ep_usb.caps.type_iso = false;
		ep->ep_usb.caps.type_int = false;
		ep->ep_usb.caps.type_bulk = true;

		list_add_tail(&ep->ep_usb.ep_list,
			      &udc->gadget.ep_list);
	}
}

static irqreturn_t vusb_mcu_irq(int irq, void* dev_id)
{
  struct vusb_udc* udc = dev_id;
  irqreturn_t iret = IRQ_HANDLED;

  struct irq_desc* desc = irq_to_desc(irq);
  struct irq_data* data = irq_desc_get_irq_data(desc);
  if (desc && data && desc->irq_data.hwirq == GPIO_LISTEN_IRQ_PIN)
  {
    struct irq_chip* chip = irq_desc_get_chip(desc);
    if (chip)
    {
      UDCVDBG(udc, "mcu ast write/read: VUSB_DEVICE_IRQ\n");
      trace_printk("irq/desc:%d, irqs/unhandled:%d, irq/count:%d\n",
        desc->irq_data.hwirq, desc->irqs_unhandled, desc->irq_count);
      vusb_write_buffer(udc, VUSB_SPI_CMD_READ | VUSB_SPI_DEVICE_IRQ, udc->transfer, 0);
    }
  }
  return iret;
}

static irqreturn_t vusb_spi_dtrdy(int irq, void* dev_id)
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
      chip->irq_ack(data);

      trace_printk("irq/desc:%d, irqs/unhandled:%d, irq/count:%d\n",
        irq, desc->irqs_unhandled, 0);
      //clear and read
      spi_cmd_t* cmd = (spi_cmd_t*)udc->transfer;
      cmd->length = VUSB_SPI_BUFFER_LENGTH >> 1;
      memset(udc->transfer, 0, cmd->length);
      if (vusb_read_buffer(udc, VUSB_SPI_CMD_READ, udc->transfer, cmd->length)) {
        cmd = (spi_cmd_t*)udc->transfer;
        pr_hex_mark(udc->transfer, cmd->length + VUSB_SPI_HEADER, PR_READ);
      }
      wake_up_interruptible(&udc->spi_read_queue);
    }
  }
  return iret;
}

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

	spin_lock_init(&udc->lock);
	mutex_init(&udc->spi_bus_mutex);

  /* INTERRUPT spi read queue */
  init_waitqueue_head(&udc->spi_read_queue);
  mutex_init(&udc->spi_read_mutex);

	udc->ep0req.ep = &udc->ep[0];
	udc->ep0req.usb_req.buf = udc->ep0buf;
	INIT_LIST_HEAD(&udc->ep0req.queue);

	/* setup Endpoints */
	vusb_eps_init(udc);

	/* configure SPI */
	spi_rd_buf(udc, VUSB_REG_EPIRQ, reg, 8);
	spi_wr8(udc, VUSB_REG_PINCTL, FDUPSPI);

  rc = usb_add_gadget_udc(&spi->dev, &udc->gadget);
  if (rc) {
    dev_err(&spi->dev, "UDC gadget could not be added\n");
    return rc;
  }

  udc->transfer = devm_kcalloc(&spi->dev, VUSB_SPI_BUFFER_LENGTH,
    sizeof(*udc->transfer), GFP_KERNEL);
  if (!udc->transfer)
  {
    dev_err(&spi->dev, "Unable to allocate Hub transfer buffer.\n");
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

	udc->thread_task = kthread_create(vusb_thread, udc,
					  "vusb-thread");
	if (IS_ERR(udc->thread_task))
		return PTR_ERR(udc->thread_task);

	udc->is_selfpowered = 1;
	udc->todo |= UDC_START;
	usb_udc_vbus_handler(&udc->gadget, true);
	usb_gadget_set_state(&udc->gadget, USB_STATE_POWERED);
	//vusb_start(udc);

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

  dev_info(&spi->dev, "Removing USB udc hub\n");

	usb_del_gadget_udc(&udc->gadget);

	spin_lock_irqsave(&udc->lock, flags);

	kthread_stop(udc->thread_task);

	spin_unlock_irqrestore(&udc->lock, flags);

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
