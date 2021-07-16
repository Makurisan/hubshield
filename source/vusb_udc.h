/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __VUSB_UDC_H
#define __VUSB_UDC_H

#include <linux/usb.h>

/*********************************
 *                               *
 * SPI definition for read/write *
 *                               *
 *********************************/

#define VUSB_SPI_HEADER		        (1 <<  2)
#define VUSB_SPI_BUFFER_LENGTH		(1 << 10)

#define VUSB_SPI_CMD_READ    0x40
#define VUSB_SPI_CMD_WRITE   0x80
#define VUSB_SPI_DEVICE_IRQ  0x1a
#define VUSB_SPI_DATRDY_TIMEOUT  800 // ms

#define VUSB_DEVICE_PING      0x11
#define VUSB_DEVICE_RESET     0x12
#define VUSB_DEVICE_ATTACH    0x13
#define VUSB_DEVICE_DETACH    0x14
#define VUSB_DEVICE_MEMORY    0x15
#define VUSB_DEVICE_HWATTACH  0x16
#define VUSB_DEVICE_DATA      0x17
#define VUSB_DEVICE_HEADER    0x18 // the header with the length field
#define VUSB_DEVICE_CLEARSCRN 0x19
#define VUSB_DEVICE_IRQ       0x20

#define VUSB_DEVICE_ERROR 0x3e // diagnose
#define VUSB_DEVICE_MAX 0x3f // max cmd nbr


 /***********************************
 *                                  *
 * GPIO definition for irq handling  *
 *                                  *
 ***********************************/

#define GPIO_DATRDY_IRQ_PIN   5
#define GPIO_LISTEN_IRQ_PIN   6

 /***********************************
 *                                  *
 * PRINT                            *
 *                                  *
 ***********************************/

#define PR_WRITE 1
#define PR_READ  2
#define PR_ERROR 4


 /***********************************
 *                                  *
 * xxxxxx                           *
 *                                  *
 ***********************************/

#define VUSB_MAX_EPS		4
#define VUSB_EP_MAX_PACKET		64  /* Same for all Endpoints */
#define VUSB_EPNAME_SIZE		16  /* Buffer size for endpoint name */

#define VUSB_ACKSTAT		BIT(0)

#define VUSB_SPI_DIR_RD	0	/* read register from VUSB */
#define VUSB_SPI_DIR_WR	1	/* write register to VUSB */

/* SPI commands: */
#define VUSB_SPI_DIR_SHIFT	1
#define VUSB_SPI_REG_SHIFT	3

#define VUSB_REG_EP0FIFO	0
#define VUSB_REG_EP1FIFO	1
#define VUSB_REG_EP2FIFO	2
#define VUSB_REG_EP3FIFO	3
#define VUSB_REG_SUDFIFO	4
#define VUSB_REG_EP0BC	5
#define VUSB_REG_EP1BC	6
#define VUSB_REG_EP2BC	7
#define VUSB_REG_EP3BC	8

#define VUSB_REG_EPSTALLS	9
#define ACKSTAT		BIT(6)
#define STLSTAT		BIT(5)
#define STLEP3IN	BIT(4)
#define STLEP2IN	BIT(3)
#define STLEP1OUT	BIT(2)
#define STLEP0OUT	BIT(1)
#define STLEP0IN	BIT(0)

#define VUSB_REG_CLRTOGS	10
#define EP3DISAB	BIT(7)
#define EP2DISAB	BIT(6)
#define EP1DISAB	BIT(5)
#define CTGEP3IN	BIT(4)
#define CTGEP2IN	BIT(3)
#define CTGEP1OUT	BIT(2)

#define VUSB_REG_EPIRQ	11
#define VUSB_REG_EPIEN	12
#define SUDAVIRQ	BIT(5)
#define IN3BAVIRQ	BIT(4)
#define IN2BAVIRQ	BIT(3)
#define OUT1DAVIRQ	BIT(2)
#define OUT0DAVIRQ	BIT(1)
#define IN0BAVIRQ	BIT(0)

#define VUSB_REG_USBIRQ	13
#define VUSB_REG_USBIEN	14
#define OSCOKIRQ	BIT(0)
#define RWUDNIRQ	BIT(1)
#define BUSACTIRQ	BIT(2)
#define URESIRQ		BIT(3) // reset start
#define SUSPIRQ		BIT(4)
#define NOVBUSIRQ	BIT(5)
#define VBUSIRQ		BIT(6)
#define URESDNIRQ	BIT(7) // reset end

#define VUSB_REG_USBCTL	15
#define HOSCSTEN	BIT(7)
#define VBGATE		BIT(6)
#define CHIPRES		BIT(5)
#define PWRDOWN		BIT(4)
#define CONNECT		BIT(3)
#define SIGRWU		BIT(2)

#define VUSB_REG_CPUCTL	16
#define IE		BIT(0)

#define VUSB_REG_PINCTL	17
#define EP3INAK		BIT(7)
#define EP2INAK		BIT(6)
#define EP0INAK		BIT(5)
#define FDUPSPI		BIT(4)
#define INTLEVEL	BIT(3)
#define POSINT		BIT(2)
#define GPXB		BIT(1)
#define GPXA		BIT(0)

#define VUSB_REG_REVISION	18

#define VUSB_REG_FNADDR	19
#define FNADDR_MASK	0x7f

#define VUSB_REG_IOPINS	20
#define VUSB_REG_IOPINS2	21
#define VUSB_REG_GPINIRQ	22
#define VUSB_REG_GPINIEN	23
#define VUSB_REG_GPINPOL	24
#define VUSB_REG_HIRQ	25
#define VUSB_REG_HIEN	26
#define VUSB_REG_MODE	27
#define VUSB_REG_PERADDR	28
#define VUSB_REG_HCTL	29
#define VUSB_REG_HXFR	30
#define VUSB_REG_HRSL	31

#define ENABLE_IRQ	BIT(0)
#define IOPIN_UPDATE	BIT(1)
#define REMOTE_WAKEUP	BIT(2)
#define CONNECT_HOST	GENMASK(4, 3)
#define	HCONNECT	(1 << 3)
#define	HDISCONNECT	(3 << 3)
#define UDC_START	GENMASK(6, 5)
#define	START		(1 << 5)
#define	STOP		(3 << 5)
#define ENABLE_EP	GENMASK(8, 7)
#define	ENABLE		(1 << 7)
#define	DISABLE		(3 << 7)
#define STALL_EP	GENMASK(10, 9)
#define	STALL		(1 << 9)
#define	UNSTALL		(3 << 9)

#define VUSB_CMD(c)		FIELD_PREP(GENMASK(7, 3), c)
#define VUSB_SPI_CMD_RD(c)	(VUSB_CMD(c) | (0 << 1))
#define VUSB_SPI_CMD_WR(c)	(VUSB_CMD(c) | (1 << 1))

#define CRC8_TABLE_SIZE   256
#define VUSB_MAX_CHAR_DEVICES 3

#define UDCVDBG(u, fmt...)	dev_info(&(u)->spi->dev, fmt)

struct vusb_req {
  struct usb_request usb_req;
  struct list_head queue;
  struct vusb_ep* ep;
};

struct vusb_ep {
  struct usb_ep ep_usb;
  struct vusb_udc* udc;
  struct list_head queue;
  char name[VUSB_EPNAME_SIZE];
  unsigned int maxpacket;
  spinlock_t lock;
  int halted;
  u32 todo;
  int id;
};

#pragma pack(8)
typedef union {
  struct {
    uint8_t reg : 6;  /* Register R0 through ... 64 */
    uint8_t read : 1; 	/* read  */
    uint8_t write : 1;  /* write */
  } bit;
  u8 val;
} spi_reg_t;

#pragma pack(8)
typedef struct vusb_spi_cmd {
  spi_reg_t reg;
  u8 crc8;
  u16 length;
  u8 data[0]; /* dummy */
} spi_cmd_t;
#pragma pack()

struct vusb_udc {

  /* SPI master */
  struct spi_device* spi;
  /* SPI transfer buffer */
  u8* transfer;

  /* GPIO SPI IRQs */
  int irq_reset;
  int spi_datrdy;
  int mcu_irq;

  struct mutex spi_read_mutex;
  struct wait_queue_head spi_read_queue;

  u8 crc_table[CRC8_TABLE_SIZE];

  /* char device */
  struct cdev cdev;
  struct class* chardev_class;
  int crdev_major;

  /* Per-port info */
  //struct vusb_port* ports;
  u32			max_ports;

  struct usb_gadget gadget;

  struct vusb_ep ep[VUSB_MAX_EPS];

  struct usb_gadget_driver* driver;

  struct task_struct* thread_task;

  int remote_wkp, is_selfpowered;
  bool softconnect;

  struct usb_ctrlrequest setup;

  struct mutex spi_bus_mutex;

  struct vusb_req ep0req;

  struct device* dev;

  spinlock_t lock;

  bool suspended;

  u8 ep0buf[64];

  u32 todo;

};

int vusb_chardev_uevent(struct device* dev, struct kobj_uevent_env* env);
void pr_hex_mark(const char* mem, int count, int mark);
int vusb_write_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length);
int vusb_read_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length);
void vusb_eps_init(struct vusb_udc* udc);
void vusb_req_done(struct vusb_req* req, int status);
void vusb_nuke(struct vusb_ep* ep, int status);

#endif /* __VUSB_UDC_H */
