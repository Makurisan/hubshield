/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __VUSB_UDC_H
#define __VUSB_UDC_H

#include <linux/usb.h>

/*********************************
 *                               *
 * SPI definition for read/write *
 *                               *
 *********************************/

enum vusb_req_code {
  VUSB_REQ_GET_STATUS = 0,
  VUSB_REQ_CLEAR_FTR = 1,
  VUSB_REQ_GET_STATE = 2,
  VUSB_REQ_SET_FTR = 3,
  VUSB_REQ_SET_ADDRESS = 5,
  VUSB_REQ_GET_DESC = 6,
  VUSB_REQ_SET_DESC = 7,
  VUSB_REQ_GET_CONFIG = 8,
  VUSB_REQ_SET_CONFIG = 9,
  VUSB_REQ_GET_INTERFACE = 10,
  VUSB_REQ_SET_INTERFACE = 11,
  VUSB_REQ_SYNCH_FRAME = 12,
  VUSB_REQ_SET_SEL = 48,
  VUSB_REQ_ISOCH_DELAY = 49
};

#define VUSB_SPI_DATRDY_TIMEOUT  800 // ms

#define VUSB_SPI_HEADER		        (1 <<  2)
#define VUSB_SPI_BUFFER_LENGTH		(1024)

#define VUSB_SPI_CMD_READ    0x40
#define VUSB_SPI_CMD_WRITE   0x80

#define VUSB_REG_RESET      0x01
#define VUSB_REG_ATTACH     0x02
#define VUSB_REG_HWDETACH   0x03
#define VUSB_REG_HWATTACH   0x04
#define VUSB_REG_MEMORY		  0x05
 // register function
#define VUSB_REG_SET        0x06
#define VUSB_REG_PORT_ATTACH  0x07
#define VUSB_REG_PORT_DETACH  0x08
#define VUSB_REG_ACK					0x09

#define VUSB_REG_USB				0x10
#define VUSB_REG_USB_GET_STATUS   	(VUSB_REG_USB + VUSB_REQ_GET_STATUS   )
#define VUSB_REG_USB_CLEAR_FTR    	(VUSB_REG_USB + VUSB_REQ_CLEAR_FTR    )
#define VUSB_REG_USB_GET_STATE    	(VUSB_REG_USB + VUSB_REQ_GET_STATE    )
#define VUSB_REG_USB_SET_FTR      	(VUSB_REG_USB + VUSB_REQ_SET_FTR      )
#define VUSB_REG_USB_GET_DESC     	(VUSB_REG_USB + VUSB_REQ_GET_DESC     )
#define VUSB_REG_USB_SET_DESC     	(VUSB_REG_USB + VUSB_REQ_SET_DESC     )
#define VUSB_REG_USB_GET_CONFIG   	(VUSB_REG_USB + VUSB_REQ_GET_CONFIG   )
#define VUSB_REG_USB_SET_CONFIG   	(VUSB_REG_USB + VUSB_REQ_SET_CONFIG   )
#define VUSB_REG_USB_GET_INTERFACE	(VUSB_REG_USB + VUSB_REQ_GET_INTERFACE)
#define VUSB_REG_USB_SET_INTERFACE	(VUSB_REG_USB + VUSB_REQ_SET_INTERFACE)

#define VUSB_REG_READ_LOCK  0x29
#define VUSB_REG_PRINTF		  0x30
#define VUSB_REG_PRINTF1	  0x31
#define VUSB_REG_PRINTF2	  0x32
#define VUSB_REG_PRINTF3	  0x33
#define VUSB_REG_PRINTF4	  0x34
#define VUSB_REG_PRINTF5	  0x35
#define VUSB_REG_PRINTF99	  0x99

 // irq register
#define VUSB_REG_IRQ_GET    0x10
#define VUSB_REG_IRQ_SET    0x11
#define VUSB_REG_IRQ_CLEAR  0x12
// port register							 

// pipe register
#define VUSB_REG_PIPE_GET_DATA    0x15
#define VUSB_REG_DEBUG            0x16
#define VUSB_REG_PIPE_WRITE_DATA  0x17
#define VUSB_REG_PIPE_EP_ENABLE   0x18
#define VUSB_REG_PIPE_MAXPKTSIZE  0x19

#define VUSB_REG_MAX 0x3f // max cmd nbr


 /***********************************
 *                                  *
 * GPIO definition for irq handling  *
 *                                  *
 ***********************************/

#define GPIO_LISTEN_IRQ_PIN   5
#define GPIO_DATRDY_IRQ_PIN   6

 /***********************************
 *                                  *
 * PRINT                            *
 *                                  *
 ***********************************/

#define PRINTF_WRITE 1
#define PRINTF_READ  2
#define PRINTF_ERROR 4
#define PRINTF_READX 5

#define bswap32(_value) __builtin_bswap32(_value)
#define _bf_ffsl(x) (__builtin_ffsl (x) - 1)
#define _bf_popcount(x) __builtin_popcount (x)

 /***********************************
 *                                  *
 *                                  *
 *                                  *
 ***********************************/
#define REG_IRQ_ELEMENTS 12

#define REG_CPUCTL		1
#define IE				BIT(0)	// enable irq pin handling
#define SOFTCONT	BIT(1)	// soft connect

#define REG_HUBSTAT	2

#define REG_USBIRQ	3
  #define SOFIRQ		BIT(0) // SOF from host
  #define HRESIRQ		BIT(1) // System reset
  #define SRESIRQ		BIT(2) // Usb reset start
  #define URESIRQ		BIT(3) // Usb reset end
  #define SUSPIRQ		BIT(4) // Setup pakets
  #define UINTIRQ		BIT(5) // Irq interrupt	
  #define SPIENIRQ	BIT(6) // SPI enable
#define REG_USBIEN	4
#define REG_PRTIRQ	5
  #define PRTIRQ(_p) BIT(_p)
  #define PRTIRQ_GET(_p, _data)	FIELD_GET(GENMASK(_p+1, _p), _data)	
  #define PRTIRQ_MASK_SET(_p)	FIELD_PREP(GENMASK(_p, _p), 1)	

#define REG_PRTIEN	6

#define REG_PIPIRQ4	7
#define REG_PIPEIRQ REG_PIPIRQ4 // all 32 bits
#define REG_PIPIEN4	11
#define REG_PIPIEN REG_PIPIEN4 // all 32 bits

//#define REG_PIPIEN_REG __builtin_bswap32(*(uint32_t*)&_spi_reg_data.vusb[REG_PIPIEN4])
//#define REG_PIPIRQ_REG __builtin_bswap32(*(uint32_t*)&_spi_reg_data.vusb[REG_PIPIRQ4])

#define REG_SPIMOD	15 // Edge mode: rising/falling; low, high
#define REG_PORTS 	16 // count of ports
#define REG_PIPES 	17 // count of pipes
#define REG_TDS   	18 // count of tds


#define VUSB_MAX_EPS		4
#define VUSB_EP_MAX_PACKET_LIMIT	64 /* Same for all Endpoints */
#define VUSB_EPNAME_SIZE		16  /* Buffer size for endpoint name */

#define ENABLE_IRQ	BIT(0)
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

#define CRC8_TABLE_SIZE   256
#define VUSB_MAX_CHAR_DEVICES 3

#define UDCVDBG(u, fmt...)	dev_info(&(u)->spi->dev, fmt)

#define VUSB_MCU_IRQ_GPIO    0x00
#define VUSB_MCU_EP_ENABLE   0x03 // enable the EP on the mcu at the specified port



// register data map
#pragma pack(1)
typedef struct vusb_req_map {
// usb
  u8 USBIRQ; u8 USBIEN;
// port
  u8 PRTIRQ; u8 PRTIEN;
// pipe
  u32 PIPIRQ;
  u32 PIPIEN;
}vusb_req_map_t;
#pragma pack()

struct vusb_req {
  struct usb_request usb_req;
  struct list_head queue;
  struct vusb_ep* ep;
};

struct vusb_ep {
  struct usb_ep ep_usb;
  struct vusb_udc* udc;
  struct work_struct ep_wq; // ep data handling
  struct work_struct ep_wt; // ep status handling
  struct work_struct ep_ws; // ep control setup
  struct list_head queue;
  char name[VUSB_EPNAME_SIZE];
  int pipe; // pipe index on MCU
  int port;   // port on MCU
  unsigned int maxpacket;
  spinlock_t lock;
  int halted;
  u32 todo;
  int id;
};

#pragma pack(1)
typedef union {
  struct {
    uint8_t reg : 6;  /* Register R0 through ... 64 */
    uint8_t read : 1; 	/* read  */
    uint8_t write : 1;  /* write */
  } bit;
  u8 val;
} spi_reg_t;
#pragma pack()

#pragma pack(1)
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
  u8* spitransfer; /*can be used*/
  u8* spiwritebuffer; /*internal usage*/
  u8* transfer;

  struct mutex spi_read_mutex;
  struct mutex spi_write_mutex;

  struct work_struct	vusb_irq_wq;
  /* GPIO SPI IRQs */
  struct gpio_desc* mcu_gpreset;
  int mcu_reset;
  int spi_datrdy;
  int mcu_irq;

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
  struct vusb_req ep0req;
  u8 ep0buf[64];
  struct usb_ctrlrequest setup;

  struct usb_gadget_driver* driver;

  int remote_wkp, is_selfpowered;
  bool softconnect;

  struct device* dev;

  spinlock_t lock;
  spinlock_t wq_lock;

  bool suspended;

  u32 todo;

};

int vusb_chardev_uevent(struct device* dev, struct kobj_uevent_env* env);
void pr_hex_mark(const char* mem, int count, int mark, const char* label);
void pr_hex_mark_debug(const char* mem, int count, int mark, const char* label, const char* debug);
int vusb_write_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length);
int vusb_read_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length);
void vusb_eps_init(struct vusb_udc* udc);
void vusb_req_done(struct vusb_req* req, int status);
void vusb_nuke(struct vusb_ep* ep, int status);
irqreturn_t vusb_spi_dtrdy(int irq, void* dev_id);

int vusb_mpack_buffer(struct vusb_udc* udc, u8 reg, u8* buffer, u16 length);
void vusb_spi_pipe_ack(struct vusb_udc* udc, u8 irq);
void vusb_handle_setup(struct vusb_udc* udc, struct vusb_ep* ep, struct usb_ctrlrequest setup);
void vusb_spi_pipe_attach(struct vusb_udc* udc, u8 port);
void vusb_work_handler(struct work_struct* work);
void vusb_work_irqhandler(struct work_struct* work);
int vusb_do_data(struct vusb_udc* udc, struct vusb_ep* ep);

#endif /* __VUSB_UDC_H */
