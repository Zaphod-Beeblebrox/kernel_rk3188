/*
 * Driver for RK-UART controller.
 * Based on drivers/tty/serial/8250.c
 *
 * Copyright (C) 2011 Rochchip.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or

 * (at your option) any later version.
 *
 * Author: hhb@rock-chips.com
 * Date: 2011.06.18
 */

#ifndef CONFIG_SERIAL_RK_CONSOLE
#if defined(CONFIG_SERIAL_RK29_CONSOLE)
#define CONFIG_SERIAL_RK_CONSOLE
#endif
#endif

#if defined(CONFIG_SERIAL_RK_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/ratelimit.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/nmi.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm/irq.h>

//****************************************
//* bonovo's android box relation
//****************************************
#include <linux/rtc.h>
#include <linux/input.h>
#include <linux/backlight.h>
#include "../../input/touchscreen/rk29_i2c_goodix.h"
//****************************************

/*
*			 Driver Version Note
*
*v0.0 : this driver is 2.6.32 kernel driver;
*v0.1 : this driver is 3.0.8 kernel driver;
*v1.0 : 2012-08-09
*		1.modify dma dirver;
*		2.enable Programmable THRE Interrupt Mode, so we can just judge ((up->iir & 0x0f) == 0x02) when transmit
*		3.reset uart and set it to loopback state to ensure setting baud rate sucessfully 
*v1.1 : 2012-08-23
*		1. dma driver:make "when tx dma is only enable" work functionally  
*v1.2 : 2012-08-28
*		1. dma driver:serial rx use new dma interface  rk29_dma_enqueue_ring 
*v1.3 : 2012-12-14
*		1. When enable Programmable THRE Interrupt Mode, in lsr register, only UART_LSR_TEMT means transmit empty, but
		 UART_LSR_THRE doesn't. So, the macro BOTH_EMPTY should be replaced with UART_LSR_TEMT.
*v1.4 : 2013-04-16
*		1.fix bug dma buffer free error
*v1.5 : 2013-10-17
*		1.in some case, set uart rx as gpio interrupt to wake up arm, when arm suspends 
*v1.6 : 2013-12-23
*		1.clear receive time out interrupt request in irq handler
*/
#define VERSION_AND_TIME  "rk_serial.c v1.6 2013-12-23"

#define PORT_RK		90
#define UART_USR	0x1F	/* UART Status Register */
#define UART_USR_BUSY (1)
#define UART_IER_PTIME	0x80	/* Programmable THRE Interrupt Mode Enable */
#define UART_LSR_RFE	0x80    /* receive fifo error */
#define UART_SRR		0x22    /* software reset register */
#define UART_RESET		0x01


//#define BOTH_EMPTY 	(UART_LSR_TEMT | UART_LSR_THRE)

#define UART_NR	4   //uart port number


/* configurate whether the port transmit-receive by DMA in menuconfig*/
#define OPEN_DMA      1
#define CLOSE_DMA     0

#define TX_DMA (1)
#define RX_DMA (2)

#ifdef CONFIG_UART0_DMA_RK29 
#define UART0_USE_DMA CONFIG_UART0_DMA_RK29
#else
#define UART0_USE_DMA CLOSE_DMA
#endif
#ifdef CONFIG_UART1_DMA_RK29
#define UART1_USE_DMA CONFIG_UART1_DMA_RK29
#else
#define UART1_USE_DMA CLOSE_DMA
#endif
#ifdef CONFIG_UART2_DMA_RK29
#define UART2_USE_DMA CONFIG_UART2_DMA_RK29
#else
#define UART2_USE_DMA CLOSE_DMA
#endif
#ifdef CONFIG_UART3_DMA_RK29
#define UART3_USE_DMA CONFIG_UART3_DMA_RK29
#else
#define UART3_USE_DMA CLOSE_DMA
#endif

//serial wake up 
#ifdef CONFIG_UART0_WAKEUP_RK29 
#define UART0_USE_WAKEUP CONFIG_UART0_WAKEUP_RK29
#else
#define UART0_USE_WAKEUP 0
#endif
#ifdef CONFIG_UART1_WAKEUP_RK29
#define UART1_USE_WAKEUP CONFIG_UART1_WAKEUP_RK29
#else
#define UART1_USE_WAKEUP 0
#endif
#ifdef CONFIG_UART2_WAKEUP_RK29
#define UART2_USE_WAKEUP CONFIG_UART2_WAKEUP_RK29
#else
#define UART2_USE_WAKEUP 0
#endif
#ifdef CONFIG_UART3_WAKEUP_RK29
#define UART3_USE_WAKEUP CONFIG_UART3_WAKEUP_RK29
#else
#define UART3_USE_WAKEUP 0
#endif



#define USE_TIMER    1           // use timer for dma transport
#define POWER_MANEGEMENT 1
#define RX_TIMEOUT		(3000*3)  //uint ms
#define DMA_TX_TRRIGE_LEVEL 128
#define SERIAL_CIRC_CNT_TO_END(xmit)   CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE)


#define USE_DMA (UART0_USE_DMA | UART1_USE_DMA | UART2_USE_DMA | UART3_USE_DMA)
#define USE_WAKEUP (UART0_USE_WAKEUP | UART1_USE_WAKEUP | UART2_USE_WAKEUP | UART3_USE_WAKEUP)

#if USE_DMA
#ifdef CONFIG_ARCH_RK29
#include <mach/dma-pl330.h>
#else
#include <plat/dma-pl330.h>
#endif
#endif

#if USE_WAKEUP
#include <mach/iomux.h>
#include <linux/wakelock.h>
#endif



static struct uart_driver serial_rk_reg;

/*
 * Debugging.
 */
#ifdef CONFIG_ARCH_RK29
#define DBG_PORT 1   //DBG_PORT which uart is used to print log message
#else
#ifndef CONFIG_RK_DEBUG_UART   //DBG_PORT which uart is used to print log message
#define DBG_PORT 2
#else
#define DBG_PORT CONFIG_RK_DEBUG_UART
#endif
#endif

#ifdef CONFIG_SERIAL_CORE_CONSOLE
#define uart_console(port)	((port)->cons && (port)->cons->index == (port)->line)
#else
#define uart_console(port)	(0)
#endif


extern void printascii(const char *);
static void dbg(const char *fmt, ...)
{
	va_list va;
	char buff[256];

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

#if defined(CONFIG_DEBUG_LL) || defined(CONFIG_RK_EARLY_PRINTK)
	printascii(buff);
#endif
}

//************************************************
//* add by bonovo zbiao
//************************************************
static int cePowerOffFlag = 0;
//************************************************
//enable log output
#define DEBUG 0
static int log_port = -1;
module_param(log_port, int, S_IRUGO|S_IWUSR);

#if DEBUG
#define DEBUG_INTR(fmt...)	if (up->port.line == log_port && !uart_console(&up->port)) dbg(fmt)
#else
#define DEBUG_INTR(fmt...)	do { } while (0)
#endif


#if USE_DMA
/* added by hhb@rock-chips.com for uart dma transfer */

struct rk_uart_dma {
	u32 use_dma;            //1:used
	enum dma_ch rx_dmach;
	enum dma_ch tx_dmach;

	//receive and transfer buffer
	char * rx_buffer;    //visual memory
	char * tx_buffer;
	dma_addr_t rx_phy_addr;  //physical memory
	dma_addr_t tx_phy_addr;
	u32 rb_size;		 //buffer size
	u32 tb_size;

	//regard the rx buffer as a circular buffer
	u32 rb_head;
	u32 rb_tail;
	u32 rx_size;

	spinlock_t		tx_lock;
	spinlock_t		rx_lock;

	char tx_dma_inited;   //1:dma tx channel has been init
	char rx_dma_inited;	 //1:dma rx channel has been init
	char tx_dma_used;	 //1:dma tx is working
	char rx_dma_used;    //1:dma rx is working

	/* timer to poll activity on rx dma */
	char use_timer;
	int	 rx_timeout;
	struct timer_list rx_timer;
};
#endif

#if USE_WAKEUP	
struct uart_wake_up {
	unsigned int enable;
	unsigned int rx_mode;
	unsigned int tx_mode;
	unsigned int rx_pin;
	char rx_pin_name[32];
	unsigned int tx_pin;
	unsigned int rx_irq;
	char rx_irq_name[32];
	struct wake_lock wakelock;
	char wakelock_name[32];
};
#endif

struct uart_rk_port {
	struct uart_port	port;
	struct platform_device	*pdev;
	struct clk		*clk;
	struct clk		*pclk;
	unsigned int		tx_loadsz;	/* transmit fifo load size */
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		iir;
	unsigned char		fcr;
	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
#define LSR_SAVE_FLAGS UART_LSR_BRK_ERROR_BITS
	unsigned char		lsr_saved_flags;
#if 0
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
	unsigned char		msr_saved_flags;
#endif

	char			name[16];
	char			fifo[64];
	char 			fifo_size;
	unsigned long		port_activity;
	struct work_struct uart_work;
	struct work_struct uart_work_rx;
	struct workqueue_struct *uart_wq;
#if USE_DMA
	struct rk_uart_dma *dma;
#endif
#if USE_WAKEUP
	struct uart_wake_up *wakeup;
#endif
    //******* bonovo android box used member **********// 
	spinlock_t		write_lock;               // Add by zbiao
	spinlock_t   		read_lock;
	struct timer_list	tx_point_timer;
	struct timer_list	rx_point_timer;
	struct work_struct uart_work_point;
	struct workqueue_struct *uart_wq_point;
};

#if USE_DMA
static void serial_rk_release_dma_tx(struct uart_port *port);
static int serial_rk_start_tx_dma(struct uart_port *port);
//static void serial_rk_rx_timeout(unsigned long uart);
static void serial_rk_release_dma_rx(struct uart_port *port);
static int serial_rk_start_rx_dma(struct uart_port *port);
#else
static inline int serial_rk_start_tx_dma(struct uart_port *port) { return 0; }
#endif
static int serial_rk_startup(struct uart_port *port);

static inline unsigned int serial_in(struct uart_rk_port *up, int offset)
{
	offset = offset << 2;

	return __raw_readb(up->port.membase + offset);
}

/* Save the LCR value so it can be re-written when a Busy Detect IRQ occurs. */
static inline void dwapb_save_out_value(struct uart_rk_port *up, int offset,
					unsigned char value)
{
	if (offset == UART_LCR)
		up->lcr = value;
}

/* Read the IER to ensure any interrupt is cleared before returning from ISR. */
static inline void dwapb_check_clear_ier(struct uart_rk_port *up, int offset)
{
	if (offset == UART_TX || offset == UART_IER)
		serial_in(up, UART_IER);
}

static inline void serial_out(struct uart_rk_port *up, int offset, unsigned char value)
{
	dwapb_save_out_value(up, offset, value);
	__raw_writeb(value, up->port.membase + (offset << 2));
	if (offset != UART_TX)
		dsb();
	dwapb_check_clear_ier(up, offset);
}

/* Uart divisor latch read */
static inline int serial_dl_read(struct uart_rk_port *up)
{
	return serial_in(up, UART_DLL) | serial_in(up, UART_DLM) << 8;
}

/* Uart divisor latch write */
static int serial_dl_write(struct uart_rk_port *up, unsigned int value)
{
	unsigned int tmout = 100;

	while(!(serial_in(up, UART_LCR) & UART_LCR_DLAB)){
		if (--tmout == 0){
			if(up->port.line != DBG_PORT)
				dbg("set serial.%d baudrate fail with DLAB not set\n", up->port.line);
			return -1;
		}
	}

	tmout = 15000;
	while(serial_in(up, UART_USR) & UART_USR_BUSY){
		if (--tmout == 0){
			if(up->port.line != DBG_PORT)
				dbg("set serial.%d baudrate timeout\n", up->port.line);
			return -1;
		}
		udelay(1);
	}

	serial_out(up, UART_DLL, value & 0xff);
	serial_out(up, UART_DLM, value >> 8 & 0xff);

	return 0;

}

static int serial_lcr_write(struct uart_rk_port *up, unsigned char value)
{
	unsigned int tmout = 15000;

	while(serial_in(up, UART_USR) & UART_USR_BUSY){
		if (--tmout == 0){
			if(up->port.line != DBG_PORT)
				dbg("set serial.%d lc r = 0x%02x timeout\n", up->port.line, value);
			return -1;
		}
		udelay(1);
	}

	serial_out(up, UART_LCR, value);

	return 0;
}

static inline void serial_rk_enable_ier_thri(struct uart_rk_port *up)
{
	if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
}


static inline void serial_rk_disable_ier_thri(struct uart_rk_port *up)
{
	if (up->ier & UART_IER_THRI) {
		up->ier &= ~UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
}

static int rk29_uart_dump_register(struct uart_rk_port *up){

	unsigned int reg_value = 0;

	reg_value = serial_in(up, UART_IER);
	dbg("UART_IER = 0x%0x\n", reg_value);
	reg_value = serial_in(up, UART_IIR);
	dbg("UART_IIR = 0x%0x\n", reg_value);
    reg_value = serial_in(up, UART_LSR);
    dbg("UART_LSR = 0x%0x\n", reg_value);
    reg_value = serial_in(up, UART_MSR);
    dbg("UART_MSR = 0x%0x\n", reg_value);
    reg_value = serial_in(up, UART_MCR);
    dbg("UART_MCR = 0x%0x\n", reg_value);
    reg_value = serial_in(up, 0x21);
    dbg("UART_RFL = 0x%0x\n", reg_value);
    reg_value = serial_in(up, UART_LCR);
    dbg("UART_LCR = 0x%0x\n", reg_value);
	return 0;

}

/*
 * FIFO support.
 */
static void serial_rk_clear_fifos(struct uart_rk_port *up)
{
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
		       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_out(up, UART_FCR, 0);
}

static inline void __stop_tx(struct uart_rk_port *p)
{
	if (p->ier & UART_IER_THRI) {
		p->ier &= ~UART_IER_THRI;
		serial_out(p, UART_IER, p->ier);
	}
}

static void serial_rk_stop_tx(struct uart_port *port)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);
#if USE_DMA
	struct rk_uart_dma *uart_dma = up->dma;
	if(uart_dma->use_dma & TX_DMA){
		serial_rk_release_dma_tx(port);
	}
#endif
	__stop_tx(up);

}


static void serial_rk_start_tx(struct uart_port *port)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);

#if USE_DMA
	if(up->dma->use_dma & TX_DMA) {
		if(!up->dma->tx_dma_used)
			serial_rk_enable_ier_thri(up);
	}else {
		serial_rk_enable_ier_thri(up);
	}
#else
	serial_rk_enable_ier_thri(up);
#endif
}


static void serial_rk_stop_rx(struct uart_port *port)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);
#if USE_DMA
	struct rk_uart_dma *uart_dma = up->dma;
	if(uart_dma->use_dma & RX_DMA){
		serial_rk_release_dma_rx(port);
	}
#endif
	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);
}


static void serial_rk_enable_ms(struct uart_port *port)
{
	/* no MSR capabilities */
#if 0
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);

	dev_dbg(port->dev, "%s\n", __func__);
	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
#endif
}

#if USE_WAKEUP
static struct uart_wake_up rk29_uart_ports_wakeup[] = {
		{UART0_USE_WAKEUP, UART0_SIN, UART0_SOUT},
		{UART1_USE_WAKEUP, UART1_SIN, UART1_SOUT},
		{UART2_USE_WAKEUP, UART2_SIN, UART2_SOUT},
		{UART3_USE_WAKEUP, UART3_SIN, UART3_SOUT},
};
#endif

#if USE_DMA
/*
 * Start transmitting by dma.
 */
#define DMA_SERIAL_BUFFER_SIZE     UART_XMIT_SIZE

/* added by hhb@rock-chips.com  for uart dma transfer*/
static struct rk_uart_dma rk29_uart_ports_dma[] = {
		{UART0_USE_DMA, DMACH_UART0_RX, DMACH_UART0_TX},
		{UART1_USE_DMA, DMACH_UART1_RX, DMACH_UART1_TX},
		{UART2_USE_DMA, DMACH_UART2_RX, DMACH_UART2_TX},
		{UART3_USE_DMA, DMACH_UART3_RX, DMACH_UART3_TX},
};


/* DMAC PL330 add by hhb@rock-chips.com */
static struct rk29_dma_client rk29_uart_dma_client = {
	.name = "rk-uart-dma",
};

/*TX*/

static void serial_rk_release_dma_tx(struct uart_port *port)
{
	struct uart_rk_port *up =
			container_of(port, struct uart_rk_port, port);
	struct rk_uart_dma *uart_dma = up->dma;
	if(!port){
		return;
	}
	if(uart_dma && uart_dma->tx_dma_inited) {
		rk29_dma_free(uart_dma->tx_dmach, &rk29_uart_dma_client);
		uart_dma->tx_dma_inited = 0;
		uart_dma->tx_dma_used = 0;
	}
}

/*this function will be called every time after rk29_dma_enqueue() be invoked*/
static void serial_rk_dma_txcb(void *buf, int size, enum rk29_dma_buffresult result) {
	struct uart_port *port = buf;
	struct uart_rk_port *up = container_of(port, struct uart_rk_port, port);
	struct circ_buf *xmit = &port->state->xmit;

	if(result != RK29_RES_OK){
		printk(">>>>%s:%d result:%d\n", __func__, __LINE__, result);
		up->dma->tx_dma_used = 0;
		return;
	}

	//spin_lock(&(up->dma->rx_lock));
	xmit->tail = (xmit->tail + size) & (UART_XMIT_SIZE - 1);
	port->icount.tx += size;
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	//spin_lock(&(up->dma->tx_lock));
	up->dma->tx_dma_used = 0;
	//spin_unlock(&(up->dma->tx_lock));
	serial_rk_enable_ier_thri(up);
	up->port_activity = jiffies;
//	dev_info(up->port.dev, "s:%d\n", size);
}

static int serial_rk_init_dma_tx(struct uart_port *port) {

	struct uart_rk_port *up =
				container_of(port, struct uart_rk_port, port);
	struct rk_uart_dma *uart_dma = up->dma;

	if(!port || !uart_dma){
		dev_info(up->port.dev, "serial_rk_init_dma_tx fail\n");
		return -1;
	}

	if(uart_dma->tx_dma_inited) {
		return 0;
	}

	if (rk29_dma_request(uart_dma->tx_dmach, &rk29_uart_dma_client, NULL) == -EBUSY) {
		dev_info(up->port.dev, "rk29_dma_request tx fail\n");
		return -1;
	}

	if (rk29_dma_set_buffdone_fn(uart_dma->tx_dmach, serial_rk_dma_txcb)) {
		dev_info(up->port.dev, "rk29_dma_set_buffdone_fn tx fail\n");
		return -1;
	}
	
	if (rk29_dma_devconfig(uart_dma->tx_dmach, RK29_DMASRC_MEM, (unsigned long)(port->iobase + UART_TX))) {
		dev_info(up->port.dev, "rk29_dma_devconfig tx fail\n");
		return -1;
	}
	
	if (rk29_dma_config(uart_dma->tx_dmach, 1, 16)) {
		dev_info(up->port.dev, "rk29_dma_config tx fail\n");
		return -1;
	}

	uart_dma->tx_dma_inited = 1;
	dev_info(up->port.dev, "serial_rk_init_dma_tx sucess\n");
	return 0;

}

static int serial_rk_start_tx_dma(struct uart_port *port)
{
	int count = 0;
	struct circ_buf *xmit = &port->state->xmit;
	struct uart_rk_port *up = container_of(port, struct uart_rk_port, port);
	struct rk_uart_dma *uart_dma = up->dma;

	if(!uart_dma->use_dma)
		goto err_out;

	if(-1 == serial_rk_init_dma_tx(port))
		goto err_out;

	if (1 == uart_dma->tx_dma_used)
		return 1;

//	spin_lock(&(uart_dma->tx_lock));
	__stop_tx(up);

	count = SERIAL_CIRC_CNT_TO_END(xmit);
	count -= count%16;
	if(count >= DMA_TX_TRRIGE_LEVEL) {
		if (rk29_dma_enqueue(uart_dma->tx_dmach, port, uart_dma->tx_phy_addr + xmit->tail , count)) {
			goto err_out;
		}
		rk29_dma_ctrl(uart_dma->tx_dmach, RK29_DMAOP_START);
		up->dma->tx_dma_used = 1;
	}
//	spin_unlock(&(uart_dma->tx_lock));
	return 1;
err_out:
	dev_info(up->port.dev, "-serial_rk_start_tx_dma-error-\n");
	return -1;
}



/*RX*/
static void serial_rk_dma_rxcb(void *buf, int size, enum rk29_dma_buffresult result) {

	//printk(">>%s:%d\n", __func__, result);
}

static void serial_rk_release_dma_rx(struct uart_port *port)
{
	struct uart_rk_port *up =
				container_of(port, struct uart_rk_port, port);
	struct rk_uart_dma *uart_dma = up->dma;
	
	if(!port){
		return;
	}
	
	if(uart_dma && uart_dma->rx_dma_inited) {
		del_timer(&uart_dma->rx_timer);
		rk29_dma_free(uart_dma->rx_dmach, &rk29_uart_dma_client);
		uart_dma->rb_tail = 0;
		uart_dma->rx_dma_inited = 0;
		uart_dma->rx_dma_used = 0;
	}
}


static int serial_rk_init_dma_rx(struct uart_port *port) {

	struct uart_rk_port *up =
				container_of(port, struct uart_rk_port, port);
	struct rk_uart_dma *uart_dma = up->dma;

	if(!port || !uart_dma){
		dev_info(up->port.dev, "serial_rk_init_dma_rx: port fail\n");
		return -1;
	}

	if(uart_dma->rx_dma_inited) {
		return 0;
	}

	if (rk29_dma_request(uart_dma->rx_dmach, &rk29_uart_dma_client, NULL) == -EBUSY) {
		dev_info(up->port.dev, "rk29_dma_request fail rx \n");
		return -1;
	}

	if (rk29_dma_set_buffdone_fn(uart_dma->rx_dmach, serial_rk_dma_rxcb)) {
		dev_info(up->port.dev, "rk29_dma_set_buffdone_fn rx fail\n");
		return -1;
	}

	if (rk29_dma_devconfig(uart_dma->rx_dmach, RK29_DMASRC_HW, (unsigned long)(port->iobase + UART_RX))) {
		dev_info(up->port.dev, "rk29_dma_devconfig rx fail\n");
		return -1;
	}

	if (rk29_dma_config(uart_dma->rx_dmach, 1, 1)) {
		dev_info(up->port.dev, "rk29_dma_config rx fail 1 1 \n");
		return -1;
	}

	//rk29_dma_setflags(uart_dma->rx_dmach, RK29_DMAF_CIRCULAR);

	uart_dma->rx_dma_inited = 1;
	dev_info(up->port.dev, "serial_rk_init_dma_rx sucess\n");
	return 0;

}

static int serial_rk_start_rx_dma(struct uart_port *port)
{
	struct uart_rk_port *up =
				container_of(port, struct uart_rk_port, port);
	struct rk_uart_dma *uart_dma = up->dma;
	if(!uart_dma->use_dma)
		return 0;

	if(uart_dma->rx_dma_used == 1)
		return 0;

	if(-1 == serial_rk_init_dma_rx(port)){
		dev_info(up->port.dev, "*******serial_rk_init_dma_rx*******error*******\n");
		return -1;
	}
	
#if 0	
		if (rk29_dma_enqueue(uart_dma->rx_dmach, (void *)up, uart_dma->rx_phy_addr,
				uart_dma->rb_size/2)) {
			dev_info(up->port.dev, "*******rk29_dma_enqueue fail*****\n");
			return -1;
		}

		if (rk29_dma_enqueue(uart_dma->rx_dmach, (void *)up,
				uart_dma->rx_phy_addr+uart_dma->rb_size/2, uart_dma->rb_size/2)) {
			dev_info(up->port.dev, "*******rk29_dma_enqueue fail*****\n");
			return -1;
		}

#else
	rk29_dma_enqueue_ring(uart_dma->rx_dmach, (void *)up, uart_dma->rx_phy_addr, uart_dma->rb_size/4, 4, false);
#endif
	rk29_dma_ctrl(uart_dma->rx_dmach, RK29_DMAOP_START);
	uart_dma->rx_dma_used = 1;
	if(uart_dma->use_timer == 1){
		mod_timer(&uart_dma->rx_timer, jiffies + msecs_to_jiffies(uart_dma->rx_timeout));
	}
	up->port_activity = jiffies;
	return 1;
}

static void serial_rk_update_rb_addr(struct uart_rk_port *up){
	dma_addr_t current_pos = 0;
	dma_addr_t rx_current_pos = 0;
	struct rk_uart_dma *uart_dma = up->dma;
	//spin_lock(&(up->dma->rx_lock));
	uart_dma->rx_size = 0;
	if(uart_dma->rx_dma_used == 1){
		rk29_dma_getposition(uart_dma->rx_dmach, &current_pos, &rx_current_pos);
		uart_dma->rb_head = (rx_current_pos - uart_dma->rx_phy_addr);
		uart_dma->rx_size = CIRC_CNT(uart_dma->rb_head, uart_dma->rb_tail, uart_dma->rb_size);
	}
	//spin_unlock(&(up->dma->rx_lock));
}

static void serial_rk_report_dma_rx(unsigned long uart)
{
	int count, flip = 0;
	struct uart_rk_port *up = (struct uart_rk_port *)uart;
	struct rk_uart_dma *uart_dma = up->dma;

	if(!uart_dma->rx_dma_used || !up->port.state->port.tty)
		return;

	serial_rk_update_rb_addr(up);

	while(1) {
		count = CIRC_CNT_TO_END(uart_dma->rb_head, uart_dma->rb_tail, uart_dma->rb_size);
		if(count <= 0)
			break;
		up->port.icount.rx += count;
		flip = tty_insert_flip_string(up->port.state->port.tty, uart_dma->rx_buffer
				+ uart_dma->rb_tail, count);
		tty_flip_buffer_push(up->port.state->port.tty);
		uart_dma->rb_tail = (uart_dma->rb_tail + count) & (uart_dma->rb_size - 1);
		up->port_activity = jiffies;
	}

	//if (uart_dma->rx_size > 0)
	//	printk("rx_size:%d ADDR:%x\n", uart_dma->rx_size, uart_dma->rb_head);

	if(uart_dma->use_timer == 1){
		mod_timer(&uart_dma->rx_timer, jiffies + msecs_to_jiffies(uart_dma->rx_timeout));
	}

}
#if 0
static void serial_rk_rx_timeout(unsigned long uart)
{
	struct uart_rk_port *up = (struct uart_rk_port *)uart;

	serial_rk_report_dma_rx(up);
	//queue_work(up->uart_wq, &up->uart_work);
}

static void serial_rk_report_revdata_workfunc(struct work_struct *work)
{
	struct uart_rk_port *up =
				container_of(work, struct uart_rk_port, uart_work);

	serial_rk_report_dma_rx((unsigned long)up);

	//spin_lock(&(up->dma->rx_lock));

	if(up->port.state->port.tty && up->dma->use_timer != 1 && up->fifo_size > 0){

		tty_insert_flip_string(up->port.state->port.tty, up->fifo, up->fifo_size);
		tty_flip_buffer_push(up->port.state->port.tty);
		up->port.icount.rx += up->fifo_size;
		up->ier |= UART_IER_RDI;
		serial_out(up, UART_IER, up->ier);
	}

	//spin_unlock(&(up->dma->rx_lock));

}


static void serial_rk_start_dma_rx(struct work_struct *work)
{
	struct uart_rk_port *up =
					container_of(work, struct uart_rk_port, uart_work_rx);

	//mod_timer(&up->dma->rx_timer, jiffies + msecs_to_jiffies(up->dma->rx_timeout));
	//rk29_dma_ctrl(up->dma->rx_dmach, RK29_DMAOP_START);
	//serial_rk_start_rx_dma(&up->port);

}
#endif

#endif /* USE_DMA */

//*************************************************************************//
//* bonovo's android box relation functions
//* add by zbiao
//*************************************************************************//
#define USE_ONE_BUFF			0

#define PRINTLOG				0
#define FRAME_LEN_MIN			7

#define POINT_POWER_OFF			0x01
#define POINT_BOX_ALIVE			0x02
#define POINT_SYS_VOL           0x07
#define POINT_TOUCH_DATA		0x10
#define POINT_TOUCH_UP			0x11
#define POINT_WINCE_KEYDOWN		0x20
#define POINT_WINCE_KEYUP		0x21
#define POINT_ADVANCE_KEY       0x22
#define POINT_GPS_DATA			0x30
#define POINT_DATE_TIME 		0x40
#define POINT_WINCE_BACKLIGHT	0x50
#define POINT_BOX_WAKEUP		0x60
#define POINT_SOUND_CHANNEL		0x70
#define POINT_FM_STATUS         0x71
#define POINT_BLUETOOTH_INFO    0x72
#define POINT_DVD_INFO          0x73
#define POINT_CAN_INFO          0x74
#define POINT_MCU_STATUS        0x75
#define POINT_MCU_SERAIL        0x76
#define POINT_MCU_FLASH_STATUS  0x05
#define POINT_MCU_REQUEST       0x06
#define POINT_MCU_VERSION       0x03


#define UART3_SEND_LOG 0
#define BONOVO_DEBUG	0
#if BONOVO_DEBUG
#define bonovo_printk(fmt, arg...) printk(fmt, ##arg)
#else
#define bonovo_printk(fmt, arg...)
#endif

struct rtc_time rtc_current_rtc_time;
struct timespec tv = {
	.tv_nsec = NSEC_PER_SEC >> 1,
};

extern void rk29_send_power_key(int state);
extern void ft5x0x_report_bonovo_value(struct ts_event *event);
extern void rk28_send_wakeup_key(void);
extern void bonovo_set_brightness_leven(int leven);
extern void bonovo_update_brightness_key(void);
extern int bonovo_get_suspend_status(void);
extern void bonovo_send_power_key(int state);
extern int bonovo_set_rtc(struct timespec new_time);
extern void bonovo_update_time_key(void);
extern void bonovo_set_wince_volume(int volume);
extern int bonovo_wince_key(int key_val, int key_stat);
extern void bonovo_update_volume_key(void);
extern void bonovo_set_sys_vol(int vol);
extern void bonovo_update_sys_vol_key(void);
extern int bonovo_set_radio_status(unsigned char status);
extern int bonovo_write_radio_buff(unsigned char freq_low, unsigned char freq_high, unsigned char valid);
extern int bonovo_write_bt_buff(char* data, int size);
extern int write_mcu_buff(char* data, int size);
extern int bonovo_deal_board_key(int keyCode, int keyStatus);
extern int bonovo_deal_advance_key(int keyCode, int keyStatus);
extern void bonovo_set_android_status(int state);
extern int virtual_serial_write_buff(char* data, int size);
extern int mcu_status_write_buff(char* data, int size);
extern void bonovo_mcu_status(char* data, int size);
extern void bonovo_light_state_key(void);
extern void ft5x0x_report_bonovo_touch_event(unsigned char *buf, unsigned int buf_len);


int send_ack(struct uart_rk_port * up, const char *data, int len);
unsigned char start_complete[] = {0xFA, 0xFA, 0x07, 0x00, 0x80, 0x7B, 0x02};

//static struct ts_event ts_data;
static unsigned int check_false_count;

#if USE_ONE_BUFF

#define FRAME_FULL_DATA			0
#define FRAME_SHORE_DATA		1
#define FRAME_FIRSR_NOT_FA1		2
#define FRAME_FIRSR_NOT_FA2		3
#define FRAME_CHECKSUM_ERROR  	4

#define READ_BUFF_TIME			20
#define FRAME_MAX_SIZE			8*1024

static unsigned char m_buffer[FRAME_MAX_SIZE];
static unsigned char m_read_buffer[FRAME_MAX_SIZE];
static unsigned char m_receive_buffer[256];

static unsigned int m_write_pos=0;
static unsigned int m_read_pos=0;
static unsigned int m_data_len=0;

unsigned int write_serial_buffer(int count)
{
	if(m_data_len + count <= FRAME_MAX_SIZE)
	{
		if(m_write_pos + count < FRAME_MAX_SIZE)
		{
			memcpy(&m_buffer[m_write_pos], &m_receive_buffer[0], count);
			m_write_pos += count;
		}
		else if(m_write_pos + count > FRAME_MAX_SIZE)
		{
			memcpy(&m_buffer[m_write_pos], &m_receive_buffer[0], FRAME_MAX_SIZE-m_write_pos);
			memcpy(&m_buffer[0], &m_receive_buffer[FRAME_MAX_SIZE-m_write_pos], m_write_pos+count-FRAME_MAX_SIZE);
			m_write_pos = m_write_pos+count-FRAME_MAX_SIZE;
		}
		else
		{
			memcpy(&m_buffer[m_write_pos], &m_receive_buffer[0], count);
			m_write_pos = 0;
		}
		m_data_len += count;
	}
	else
	{
		int i;
		for (i=0; i<FRAME_MAX_SIZE-12; i+=13)
		{
			printk(KERN_INFO "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
				m_buffer[i+0], m_buffer[i+1],
				m_buffer[i+2], m_buffer[i+3],
				m_buffer[i+4], m_buffer[i+5],
				m_buffer[i+6], m_buffer[i+7],
				m_buffer[i+8], m_buffer[i+9],
				m_buffer[i+10], m_buffer[i+11],m_buffer[i+12]
				);
		}
		//while(1)
		{
			printk(KERN_INFO "========%s error buffer out ==========\n",__func__);
		}
	}
#if 0
	if(m_data_len < FRAME_MAX_SIZE)
	{
		m_write_pos &= FRAME_MAX_SIZE-1;
		m_buffer[m_write_pos++] = ch;
		m_data_len++;
		/*
		if(m_write_pos < FRAME_MAX_SIZE)
		{
			m_buffer[m_write_pos++] = ch;
			m_data_len++;
		}
		else
		{
			m_write_pos = 0;
			m_buffer[m_write_pos++] = ch;
			m_data_len++;
		}
		*/
	}
	else
	{
		int i;
		for (i=0; i<FRAME_MAX_SIZE-12; i+=13)
		{
			printk(KERN_INFO "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
				m_buffer[i+0], m_buffer[i+1],
				m_buffer[i+2], m_buffer[i+3],
				m_buffer[i+4], m_buffer[i+5],
				m_buffer[i+6], m_buffer[i+7],
				m_buffer[i+8], m_buffer[i+9],
				m_buffer[i+10], m_buffer[i+11],m_buffer[i+12]
				);
		}
		while(1)
		{
			printk(KERN_INFO "========%s error buffer out ==========\n",__func__);
		}
	}
#endif
	return 1;
}

unsigned int check_frame_sum(unsigned int read_pos, unsigned int len)
{
	unsigned int i;
	unsigned int orisum, sum=0;
	for(i=read_pos; i<read_pos+len-2; i++)
	{
		sum += m_read_buffer[i];
	}
	orisum =( (m_read_buffer[read_pos+len-1]<<8) + m_read_buffer[read_pos+len-2]) & 0xFFFF;

	if(orisum == sum)
	{
		return 1;
	}
	else
	{
		check_false_count += 1;
		if(1) printk(KERN_INFO "========%s false=%02x orisum=%02x check_false_count=%d=======\n",__func__,sum, orisum, check_false_count);
		return 0;
	}
}

unsigned int find_one_frame(unsigned int *read_pos, unsigned int *read_data_len, unsigned int *frame_len)
{
	unsigned int i, error_len;
	unsigned int find_result = FRAME_SHORE_DATA;
	*frame_len = 0;
	while(*read_data_len >= FRAME_LEN_MIN)
	{
		if(m_read_buffer[*read_pos]  == 0xFA)
		{
			if(m_read_buffer[*read_pos+1]  == 0xFA)
			{
				*frame_len = (m_read_buffer[*read_pos+3]<<8) + m_read_buffer[*read_pos+2];
				//printk(KERN_INFO "-----------------%s *frame_len=%d m_read_buffer[*read_pos+3]=%d m_read_buffer[*read_pos+2]=%d---------\n",__func__, *frame_len, m_read_buffer[*read_pos+3],m_read_buffer[*read_pos+2]);
				if(*read_data_len >= *frame_len)
				{
					if(check_frame_sum(*read_pos, *frame_len))
					{
						find_result = FRAME_FULL_DATA;
						break;
					}
					else
					{
						if(*read_data_len > *frame_len)
							error_len = *read_pos+*frame_len+1;
						else
							error_len = *read_pos+*frame_len;
						for(i=*read_pos; i<error_len; i++)
						{
							printk(KERN_INFO "========%s m_read_buffer[%02x]=%02x=======\n",__func__, i, m_read_buffer[i]);
						}

						*read_pos += 2;
						*read_data_len -= 2;
						//printk(KERN_INFO "---%s check_frame_sum false read_pos=%d read_data_len=%d frame_len=%d---\n",__func__, *read_pos, *read_data_len,*frame_len);
					}
				}
				else
				{
					find_result = FRAME_SHORE_DATA;
					break;
				}
			}
			else
			{
				*read_pos += 2;
				*read_data_len -= 2;
			}
		}
		else
		{
			//printk(KERN_INFO "------------------%s firse not 0xFA-----------------\n",__func__);
			*read_pos += 1;
			*read_data_len -= 1;
		}
	}
	
	return  find_result;
}

unsigned int read_serial_buffer(void * ptr)
{
	unsigned int frame_len=0;
	unsigned int read_data_len = m_data_len;    //volatile
	unsigned int curpos,read_buff_pos = 0;
	unsigned int result = FRAME_SHORE_DATA;
	unsigned int i,brightness;

	struct uart_rk_port *up = (struct uart_rk_port *)ptr;

	if(read_data_len < FRAME_LEN_MIN)
	{
		mod_timer(&up->rx_point_timer,  jiffies + msecs_to_jiffies(READ_BUFF_TIME));
		return 0;
	}
	memset(&ts_data, 0, sizeof(ts_data)) ;

	if(PRINTLOG) printk(KERN_INFO "\n----%s m_read_pos=%d m_write_pos=%d read_data_len= %d----\n",__func__,m_read_pos,m_write_pos,read_data_len);
	memset(m_read_buffer, 0, sizeof(m_read_buffer)) ;
	if(read_data_len + m_read_pos <= FRAME_MAX_SIZE)
	{
		memcpy(&m_read_buffer[0], &m_buffer[m_read_pos], read_data_len);
	}
	else
	{
		memcpy(&m_read_buffer[0], &m_buffer[m_read_pos], FRAME_MAX_SIZE-m_read_pos);
		memcpy(&m_read_buffer[FRAME_MAX_SIZE-m_read_pos], &m_buffer[0], read_data_len+m_read_pos-FRAME_MAX_SIZE);
	}
#if 0
	for(i=0; i<read_data_len; i++)
	{		
		printk(KERN_INFO "========%s m_read_buffer[%d]=%x ========\n",__func__,i,m_read_buffer[i]);
	}
#endif

	while(read_data_len > 0)
	{
		// printk(KERN_INFO "------%s read_data_len=%d------\n",__func__,read_data_len);

		result = find_one_frame(&read_buff_pos, &read_data_len, &frame_len);
		if(result == FRAME_FULL_DATA)
		{
#if (0)
			switch(m_read_buffer[read_buff_pos+4])
			{
				case POINT_TOUCH_DATA:	//down or up
					if(bonovo_get_suspend_status())
					{
						rk28_send_wakeup_key();
					}
					curpos = read_buff_pos+5;
					ts_data.touch_point = m_read_buffer[curpos++];
					//printk(KERN_INFO "----%s ts_data.touch_point=%d ----\n",__func__,ts_data.touch_point);
					for(i=0; i<ts_data.touch_point; i++)
					{
						ts_data.point[i].id = m_read_buffer[curpos++];
						ts_data.point[i].status = m_read_buffer[curpos++];
						ts_data.point[i].x = m_read_buffer[curpos]+(m_read_buffer[curpos+1]<<8);
						curpos += 2;
						ts_data.point[i].y = m_read_buffer[curpos]+(m_read_buffer[curpos+1]<<8);
						curpos += 3;
						/*
						down_x = m_read_buffer[curpos+1]+(m_read_buffer[curpos+2]<<8);
						down_y = m_read_buffer[curpos+3]+(m_read_buffer[curpos+4]<<8);
						if(PRINTLOG) printk(KERN_INFO "----%s finger_num=%d num=%d down_x=%d down_y=%d----\n",__func__,finger_num, m_read_buffer[curpos],down_x,down_y);
						ft5x0x_report_bonovo_value(1, m_read_buffer[curpos], down_x, down_y);
						*/
					}
					ft5x0x_report_bonovo_value(&ts_data);
					break;

				case POINT_WINCE_BACKLIGHT:
					brightness = m_read_buffer[read_buff_pos+5];
					printk(KERN_INFO "----%s POINT_WINCE_BACKLIGHT brightness=%d----\n",__func__,brightness);
					if(brightness < 20) brightness = 20;
					bonovo_set_brightness_leven(brightness);
					bonovo_update_brightness_key();
					rk28_send_wakeup_key();
					break;

				case POINT_GPS_DATA:	//gps
					spin_lock(&(up->read_lock));
					tty_insert_flip_string(up->port.state->port.tty, &m_read_buffer[read_buff_pos+5], frame_len-7);
					tty_flip_buffer_push(up->port.state->port.tty);
					spin_unlock(&(up->read_lock));
					break;

				case POINT_BOX_WAKEUP:
					rk28_send_wakeup_key();
					break;

				case POINT_DATE_TIME:	//set date time
					rtc_current_rtc_time.tm_sec = m_read_buffer[read_buff_pos+12];
					rtc_current_rtc_time.tm_min = m_read_buffer[read_buff_pos+11];
					rtc_current_rtc_time.tm_hour = m_read_buffer[read_buff_pos+10];
					rtc_current_rtc_time.tm_mday = m_read_buffer[read_buff_pos+9];
					rtc_current_rtc_time.tm_wday = m_read_buffer[read_buff_pos+8];
					rtc_current_rtc_time.tm_mon = m_read_buffer[read_buff_pos+7]-1;
					rtc_current_rtc_time.tm_year = (m_read_buffer[(read_buff_pos+5)]+(m_read_buffer[(read_buff_pos+6)]<<8))-1900;
					printk(KERN_INFO "----%s rtc_current_time s,m,h,d,w,m,y %d,%d,%d,%d,%d,%d,%d----\n",
											__func__,
											rtc_current_rtc_time.tm_sec,
											rtc_current_rtc_time.tm_min,
											rtc_current_rtc_time.tm_hour,
											rtc_current_rtc_time.tm_mday,
											rtc_current_rtc_time.tm_wday,
											rtc_current_rtc_time.tm_mon,
											rtc_current_rtc_time.tm_year);
					queue_work(up->uart_wq_point, &up->uart_work_point);
					break;

				case POINT_POWER_OFF:	//android box power off
					rk29_send_power_key(1);
					break;

				case POINT_BOX_ALIVE:
					//to do if android box is alive
					break;

				default:
					break;
			}
#endif
			if(PRINTLOG) printk(KERN_INFO "----%s read_buff_pos=%d frame_len=%d----\n",__func__,read_buff_pos,frame_len);
			read_buff_pos += frame_len;
			read_data_len -= frame_len;
		}
		else if (result == FRAME_SHORE_DATA)
		{
			break;
		}
		else
		{
			if(PRINTLOG) printk(KERN_INFO "========%s in else branch========\n",__func__);
		}
	}

	if(m_data_len < read_buff_pos)
	{
		//while(1)
		{
			printk(KERN_INFO "==%s m_data_len=%d read_buff_pos=%d read_data_len= %d frame_len=%d result=%d",
				__func__, m_data_len, read_buff_pos, read_data_len, frame_len, result);
		}
	}
	m_data_len -= read_buff_pos;
	m_read_pos += read_buff_pos;
	if(m_read_pos >= FRAME_MAX_SIZE)
	{
		m_read_pos -= FRAME_MAX_SIZE;
	}
	if(PRINTLOG) printk(KERN_INFO "----%s m_read_pos=%d m_data_len=%d----\n\n",__func__,m_read_pos,m_data_len);
	mod_timer(&up->rx_point_timer,  jiffies + msecs_to_jiffies(READ_BUFF_TIME));
	return 1;
}

#else

#define READ_BUFF_TIME		20
#define UART_BUF_SIZE		256
#define UART_BUF_NUM		32

// uart receive buffer control struct
struct uart_rec_buf	{
	int w_idx;			// indicate the buffer that is written now
	int r_idx;			// indicate the buffer that is read now
	int valid_buf_num;	// indicate how many buffers that contain the valid data
	int idx_in_buf;		// the index of byte in the current buffer
	int flag;			// flag==0:has not found the sync head byte
						// flag==1:has found first sync byte(0xFA)
						// flag==2:has found second sync byte(0xFA)
						// flag==3:since flag==2 has found 4 sync byte
						// flag==4:as flag==3 has found first sync byte(0xFA)
	unsigned int cur_frame_len;
	unsigned int buf_len[UART_BUF_NUM];
	unsigned char buf[UART_BUF_NUM][UART_BUF_SIZE];
}uart3_buf;

static int m_checksum_calc=0;
static int sencond_notfa_count;

void init_uart3_buf_ctrl(void)
{
	//uart3_buf.w_idx = 0;
	//uart3_buf.r_idx = 0;
	//uart3_buf.valid_buf_num = 0;
	//uart3_buf.idx_in_buf = 0;
	//uart3_buf.flag = 0;
	memset(&uart3_buf, 0x00, sizeof(struct uart_rec_buf));
}

#if (0)
int check_frame_validation(void)
{
	unsigned char *ptr;
	unsigned int i,frame_checksum;

	if (uart3_buf.flag == 4) {
		return -2;
	}

	if (uart3_buf.cur_frame_len != 0 && uart3_buf.cur_frame_len == uart3_buf.idx_in_buf) {
		ptr = uart3_buf.buf[uart3_buf.w_idx];
		frame_checksum = (ptr[uart3_buf.idx_in_buf-1]<<8) + ptr[uart3_buf.idx_in_buf-2];
		m_checksum_calc -= ptr[uart3_buf.idx_in_buf-1] + ptr[uart3_buf.idx_in_buf-2];

		if (frame_checksum == m_checksum_calc) {
			uart3_buf.buf_len[uart3_buf.w_idx] = uart3_buf.idx_in_buf;
			uart3_buf.valid_buf_num += 1;

			if (uart3_buf.valid_buf_num == UART_BUF_NUM) {
				// all the buffer is occupied, print error info
				//while(1)
				printk(KERN_INFO "uart3_buf.valid_buf_num == UART_BUF_NUM \n");
			} else {
				// update buffer index of writting
				uart3_buf.w_idx += 1;
				uart3_buf.w_idx &= UART_BUF_NUM - 1;
			}

			uart3_buf.idx_in_buf = 0;
			uart3_buf.cur_frame_len = 0;
			m_checksum_calc = 0;
			return 0;		// good frame
		} else {
			// frame_len or frame_checksum error
			check_false_count++;
			printk(KERN_INFO "checksum error check_false_count=%d\n",check_false_count);
			for(i=0; i<uart3_buf.idx_in_buf; i++) {
				printk(KERN_INFO "--frame error ptr[%02x]=%02x-- \n",i,ptr[i]);
			}
			uart3_buf.idx_in_buf = 0;
			uart3_buf.cur_frame_len = 0;
			m_checksum_calc = 0;
			return -1;		// bad frame
		}
	} else {
		return -2;		// not a whole frame
	}
}

// the code for receiving chars int the interrupt handle
void receive_int_char(unsigned char ch)
{
	unsigned char *ptr;

	if (uart3_buf.flag == 0) {
		if (ch == 0xFA) {
			uart3_buf.flag = 1;
		} else {
			//printk(KERN_INFO "-- sencond_notfa_count=0x%02x--\n",ch);
		}
	} else if (uart3_buf.flag == 1) {
		if (ch == 0xFA) {
			uart3_buf.flag = 2;
			uart3_buf.idx_in_buf = 0;
			uart3_buf.cur_frame_len = 0;
			uart3_buf.buf[uart3_buf.w_idx][uart3_buf.idx_in_buf++] = 0xFA;
			uart3_buf.buf[uart3_buf.w_idx][uart3_buf.idx_in_buf++] = 0xFA;
			m_checksum_calc = 0xFA+0xFA;
		} else {
			uart3_buf.flag = 0;
			sencond_notfa_count++;
			printk(KERN_INFO "-- sencond_notfa_count=%d--\n",sencond_notfa_count);
		}
	} else if (uart3_buf.flag == 2) {
		uart3_buf.buf[uart3_buf.w_idx][uart3_buf.idx_in_buf++] = ch;
		m_checksum_calc += ch;

		if (uart3_buf.idx_in_buf == 4) {
			ptr = uart3_buf.buf[uart3_buf.w_idx];
			uart3_buf.cur_frame_len = (ptr[3]<<8) + ptr[2];
			uart3_buf.flag = 3;

            if((uart3_buf.cur_frame_len >= UART_BUF_SIZE)
                || (uart3_buf.cur_frame_len < 7)){
                uart3_buf.flag = 0;
                uart3_buf.idx_in_buf = 0;
                uart3_buf.cur_frame_len = 0;
                m_checksum_calc = 0;
            }
		}
	}else if(uart3_buf.flag == 3){
		// the char is the frame data
		// check the validation of idx_in_buf
		if (uart3_buf.idx_in_buf == UART_BUF_SIZE) {
			//while(1)
			printk(KERN_INFO "uart3_buf.idx_in_buf == UART_BUF_SIZE \n");
		} else {
			uart3_buf.buf[uart3_buf.w_idx][uart3_buf.idx_in_buf++] = ch;
			m_checksum_calc += ch;
		}
        
        if(uart3_buf.cur_frame_len <= uart3_buf.idx_in_buf){
            check_frame_validation();
            uart3_buf.flag = 0;
            uart3_buf.idx_in_buf = 0;
            uart3_buf.cur_frame_len = 0;
            m_checksum_calc = 0;
        }
    /*
    else if (uart3_buf.flag == 3) {
		if (ch == 0xFA) {
			uart3_buf.flag = 4;
		} else {
			// the char is the frame data
			// check the validation of idx_in_buf
			if (uart3_buf.idx_in_buf == UART_BUF_SIZE) {
				//while(1)
				printk(KERN_INFO "uart3_buf.idx_in_buf == UART_BUF_SIZE \n");
			} else {
				uart3_buf.buf[uart3_buf.w_idx][uart3_buf.idx_in_buf++] = ch;
				m_checksum_calc += ch;
			}
		}
	} else if (uart3_buf.flag == 4) {
		if (ch == 0xFA) {
			// two 0xFA has been found, that is new frame starts
			uart3_buf.flag = 3;
			check_frame_validation();

			// the new frame coming, so initial the variables
			uart3_buf.idx_in_buf = 0;
			uart3_buf.cur_frame_len = 0;
			uart3_buf.flag = 2;
			uart3_buf.buf[uart3_buf.w_idx][uart3_buf.idx_in_buf++] = 0xFA;
			uart3_buf.buf[uart3_buf.w_idx][uart3_buf.idx_in_buf++] = 0xFA;
			m_checksum_calc = 0xFA+0xFA;
		} else {
			// only one 0xFA, the 0xFA is the data of the frame, not the head of frame
			uart3_buf.buf[uart3_buf.w_idx][uart3_buf.idx_in_buf++] = 0xFA;
			m_checksum_calc += 0xFA;
			uart3_buf.buf[uart3_buf.w_idx][uart3_buf.idx_in_buf++] = ch;
			m_checksum_calc += ch;
			uart3_buf.flag = 3;
		}
    }*/
	} else {
		// error state
		//while(1)
		printk(KERN_INFO "----error state----\n");
	}
}
#else
//=============================================================================
// 下面的代码根据上面的代码进行改写和优化
// 最初接收数据并查找数据帧时,优先考虑连续双0xFA,即搜索到连续双0xFA即认为是
// 一帧的开始,但这样的解析有一个问题:即数据帧中包含连续双0xFA,则会把数据中的
// 双0xFA误认为是新一帧的开始,导致无法正确接收含有双0xFA的数据帧
// 针对这个bug,张彪对原有代码进行了改写,即上面的代码
// 上面的代码也有一个问题,即当一个数据帧有部分数据丢失时,该数据帧后续的那个
// 数据帧中的数据会被当作前一帧的数据来出来.这样一旦有一帧数据不全,会导致
// 后续的数据帧也受影响
//-----------------------------------------------------------------------------
// dzwei, 2014-8-1
//=============================================================================
int check_frame_validation(void)
{
	unsigned char *ptr;
	unsigned int i,frame_checksum;

	// Before calling this subroutine, we have checked if the data in 
	// the receive buffer is equal to frame_len or not. 
	ptr = uart3_buf.buf[uart3_buf.w_idx];
	frame_checksum = (ptr[uart3_buf.idx_in_buf-1]<<8) + ptr[uart3_buf.idx_in_buf-2];
	m_checksum_calc -= ptr[uart3_buf.idx_in_buf-1] + ptr[uart3_buf.idx_in_buf-2];
	
	if (frame_checksum == m_checksum_calc) 
	{
		uart3_buf.buf_len[uart3_buf.w_idx] = uart3_buf.idx_in_buf;
	
		if (uart3_buf.valid_buf_num+1 == UART_BUF_NUM) 
		{
			// all the buffer is occupied, print error info
			// and do nothing. This means we discard the last frame.
			printk(KERN_INFO "uart3_buf.valid_buf_num == UART_BUF_NUM \n");
		} 
		else 
		{
			// update buffer index of writting
			uart3_buf.valid_buf_num += 1;
			uart3_buf.w_idx += 1;
			uart3_buf.w_idx &= UART_BUF_NUM - 1;
		}
	
		return 0;		// good frame
	} 
	else 
	{
		// frame_len or frame_checksum error
		check_false_count++;
		printk(KERN_INFO "checksum error check_false_count=%d\n",check_false_count);
		for(i=0; i<uart3_buf.idx_in_buf; i++) 
		{
			printk(KERN_INFO "--frame error ptr[%02x]=%02x-- \n",i,ptr[i]);
		}
		return -1;		// bad frame
	}
}

// the code for receiving chars int the interrupt handle
void receive_int_char(unsigned char ch)
{
	unsigned char *ptr;

	if (uart3_buf.flag == 0) 
	{
		if (ch == 0xFA)
		{
			uart3_buf.flag = 1;
		}
	} 
	else if (uart3_buf.flag == 1) 
	{
		if (ch == 0xFA) 
		{
			// 一些变量的初始化在这个阶段进行,因为这个阶段是数据帧判别的必经
			// 路径,因此不需要在其他地方对这些变量进行初始化
			uart3_buf.buf[uart3_buf.w_idx][0] = 0xFA;
			uart3_buf.buf[uart3_buf.w_idx][1] = 0xFA;
			uart3_buf.flag = 2;
			uart3_buf.idx_in_buf = 2;
			m_checksum_calc = 0xFA+0xFA;
		} 
		else 
		{
			uart3_buf.flag = 0;
			sencond_notfa_count++;
			printk(KERN_INFO "-- sencond_notfa_count=%d--\n",sencond_notfa_count);
		}
	} 
	else if (uart3_buf.flag == 2) 
	{
		uart3_buf.buf[uart3_buf.w_idx][uart3_buf.idx_in_buf++] = ch;
		m_checksum_calc += ch;

		// when we have received 4 chars, we can get frame length from 
		// the frame data
		if (uart3_buf.idx_in_buf == 4) 
		{
			ptr = uart3_buf.buf[uart3_buf.w_idx];
			uart3_buf.cur_frame_len = (ptr[3]<<8) + ptr[2];

			// if the frame is too large or to small, just discard the frame
			if ((uart3_buf.cur_frame_len > UART_BUF_SIZE)
				|| (uart3_buf.cur_frame_len < 7))
			{
				uart3_buf.flag = 0;
			}
			else
			{
				uart3_buf.flag = 3;
			}
		}
	}
	else if(uart3_buf.flag == 3)
	{
		// save the char in the frame data buffer
		// check the validation of idx_in_buf
		uart3_buf.buf[uart3_buf.w_idx][uart3_buf.idx_in_buf++] = ch;
		m_checksum_calc += ch;

		// if the data received is more than frame length
		if (uart3_buf.cur_frame_len == uart3_buf.idx_in_buf)
		{
			check_frame_validation();
			uart3_buf.flag = 0;
		}
	} 
	else 		// will never enter this branch
	{
		// error state
		//while(1)
		printk(KERN_INFO "----error state----\n");
	}
}
#endif

extern int fill_canbus_buf(unsigned char *buf, unsigned int buf_len);

int bonovo_parse_canbus_data(unsigned char *buf, unsigned long frame_len)
{
	// now this function is only supporting volkaswagen universal machine
	// If we will support other vendor vehicle, we need check the vehicle
	// type before parse the canbus data frame.
	// dzwei, 2014-8-6

	int i;
	unsigned char checksum_calculated = 0;
	int error_code = 0;
	
	//---------------------------------------------------------------
	// step: check canbus data frame validation
	//---------------------------------------------------------------
	if (frame_len < 5)
	{
		error_code = -1;		// frame length is not enough
	}
	else if (buf[0] != 0x2E)
	{
		error_code = -2;		// bad frame head
	}
	else
	{
		for (i=1; i<frame_len-1; i++)
		{
			checksum_calculated += buf[i];
		}
		checksum_calculated = checksum_calculated^0xFF;
		if (checksum_calculated != buf[frame_len-1])
		{
			error_code = -3;	// checksum error
		}
	}
	if (error_code)
	{
		return error_code;
	}

	// process steering wheel key
	if (buf[1] == 0x23)
	{
        bonovo_deal_advance_key(0x060000 + (buf[3]<<8), buf[4]);	
	}
	else		// other information will pass the char device driver
	{
		fill_canbus_buf(buf, frame_len);
	}
	return 0;
}

int read_serial_frame(void * pport)
{
	unsigned char *ptr;
	unsigned int i;
	unsigned int brightness, sys_vol;
    unsigned int keycode;
	unsigned int data_len;		// the actual data length in frame
	struct uart_rk_port *up = (struct uart_rk_port *)pport;

	// first checking the frame length
	while (uart3_buf.valid_buf_num) {
		ptr = uart3_buf.buf[uart3_buf.r_idx];
		bonovo_printk(KERN_INFO "---- valid_buf_numd=%d r_idx=%d ptr[4]=0x%08x----\n",uart3_buf.valid_buf_num,uart3_buf.r_idx,ptr[4]);
		switch (ptr[4]) {
		case POINT_TOUCH_DATA:	//down or up
			bonovo_printk("-------- %s:suspend_status:%d\n", __FILE__, bonovo_get_suspend_status());
			if (bonovo_get_suspend_status()) {
				rk28_send_wakeup_key();
			}
			/*
			curpos = 5;
			ts_data.touch_point = ptr[curpos++];
			bonovo_printk(KERN_INFO "---- ts_data.touch_point=%d ----\n",ts_data.touch_point);
			for(i=0; i<ts_data.touch_point; i++) {
				ts_data.point[i].id = ptr[curpos++];
				ts_data.point[i].status = ptr[curpos++];
				ts_data.point[i].x = ptr[curpos]+(ptr[curpos+1]<<8);
				curpos += 2;
				ts_data.point[i].y = ptr[curpos]+(ptr[curpos+1]<<8);
				curpos += 3;
				//if(ts_data.point[i].status == 0)
				//printk(KERN_INFO "--id=%d status=%d x=%d y=%d--\n",
				//ts_data.point[i].id,ts_data.point[i].status,ts_data.point[i].x,ts_data.point[i].y);
			}
			ft5x0x_report_bonovo_value(&ts_data);
			*/
			data_len = uart3_buf.buf_len[uart3_buf.r_idx]-7;
			ft5x0x_report_bonovo_touch_event(&ptr[5], data_len);
			break;
		case POINT_WINCE_BACKLIGHT:
			brightness = ptr[5];
			bonovo_printk(KERN_INFO "----%s POINT_WINCE_BACKLIGHT brightness=%d----\n",__func__,brightness);
			if (brightness < 20)
				brightness = 20;
			bonovo_set_brightness_leven(brightness);
			bonovo_update_brightness_key();
			rk28_send_wakeup_key();
			break;
		case POINT_SYS_VOL:
			sys_vol = ptr[5];
			bonovo_set_sys_vol(sys_vol);
			bonovo_update_sys_vol_key();
			break;
		case POINT_GPS_DATA:	//gps
			spin_lock(&(up->read_lock));
			tty_insert_flip_string(up->port.state->port.tty, &ptr[5], uart3_buf.buf_len[uart3_buf.r_idx]-7);
			tty_flip_buffer_push(up->port.state->port.tty);
			spin_unlock(&(up->read_lock));
			break;
		case POINT_BOX_WAKEUP:
			rk28_send_wakeup_key();
			break;
		case POINT_WINCE_KEYDOWN:
			//bonovo_wince_key(ptr[5] + (ptr[6] << 8), 1);
			bonovo_deal_board_key(ptr[5] + (ptr[6] << 8), 1);
			break;
		case POINT_WINCE_KEYUP:
			//bonovo_wince_key(ptr[5] + (ptr[6] << 8), 0);
			bonovo_deal_board_key(ptr[5] + (ptr[6] << 8), 0);
			break;
        case POINT_ADVANCE_KEY:
            keycode = ptr[6] + ((ptr[7]&0x00FF)<<8)
                + ((ptr[8]&0x00FF)<<16) + ((ptr[9]&0x00FF)<<24);
            bonovo_deal_advance_key(keycode, ptr[5]);
            break;
		case POINT_DATE_TIME:	//set date time
			rtc_current_rtc_time.tm_sec = ptr[12];
			rtc_current_rtc_time.tm_min = ptr[11];
			rtc_current_rtc_time.tm_hour = ptr[10];
			rtc_current_rtc_time.tm_mday = ptr[9];
			rtc_current_rtc_time.tm_wday = ptr[8];
			rtc_current_rtc_time.tm_mon = ptr[7]-1;
			rtc_current_rtc_time.tm_year = (ptr[5]+(ptr[6]<<8))-1900;
			bonovo_printk(KERN_INFO "----%s rtc_current_time s,m,h,d,w,m,y %d,%d,%d,%d,%d,%d,%d----\n",
									__func__,
									rtc_current_rtc_time.tm_sec,
									rtc_current_rtc_time.tm_min,
									rtc_current_rtc_time.tm_hour,
									rtc_current_rtc_time.tm_mday,
									rtc_current_rtc_time.tm_wday,
									rtc_current_rtc_time.tm_mon,
									rtc_current_rtc_time.tm_year);
			queue_work(up->uart_wq_point, &up->uart_work_point);
			break;
		case POINT_POWER_OFF:	//android box power off
			//rk29_send_power_key(1);
			bonovo_send_power_key(1);
			cePowerOffFlag = 1;
			break;
		case POINT_SOUND_CHANNEL:	//update sound channel
			//bonovo_set_wince_volume(ptr[5]); //0:Stereo 1:Left channel
			//bonovo_update_volume_key();
			break;
		case POINT_BOX_ALIVE:
			//send_ack(up, start_complete, ARRAY_SIZE(start_complete));
			bonovo_set_android_status(ptr[6]);
			//to do if android box is alive
			break;
		case POINT_FM_STATUS:
			switch (ptr[5])
			{
			case 0:
				bonovo_set_radio_status((unsigned char)1);
				break;
			case 1:
				bonovo_set_radio_status((unsigned char)0);
				break;
			case 2:
				//printk("KERN_INFO fm:%d valid:0x%02X\n", ptr[6] + (ptr[7]<<8), ptr[8]);
				bonovo_write_radio_buff(ptr[6], ptr[7], ptr[8]);
				break;
			default:
				//printk(KERN_INFO "Error radio command.\n");
				break;
			}
			break;
		case POINT_BLUETOOTH_INFO:
			//i = (ptr[3]<<8) + ptr[2] - 7;
			data_len = uart3_buf.buf_len[uart3_buf.r_idx]-7;
			bonovo_write_bt_buff(&ptr[5], data_len);
			break;
		case POINT_DVD_INFO:
			break;
		case POINT_CAN_INFO:
			//bonovo_parse_canbus_data(&ptr[5], uart3_buf.buf_len[uart3_buf.r_idx]-7);
			fill_canbus_buf(&ptr[5], uart3_buf.buf_len[uart3_buf.r_idx]-7);
			break;
		case POINT_MCU_SERAIL:
			//i = (ptr[3]<<8) + ptr[2] - 7;
			data_len = uart3_buf.buf_len[uart3_buf.r_idx]-7;
			virtual_serial_write_buff(&ptr[5], data_len);
			break;
        case POINT_MCU_STATUS:
            //i = (ptr[3]<<8) + ptr[2] - 7;
			data_len = uart3_buf.buf_len[uart3_buf.r_idx]-7;
			bonovo_mcu_status(&ptr[5], data_len);
			bonovo_light_state_key();
            break;
        case POINT_MCU_FLASH_STATUS:
        case POINT_MCU_REQUEST:
	 case POINT_MCU_VERSION:
            i = (ptr[3]<<8) + ptr[2] - 7;
            write_mcu_buff(&ptr[5], i);
            break;
		default:
			break;
		}
		uart3_buf.valid_buf_num -= 1;
		uart3_buf.r_idx += 1;
		uart3_buf.r_idx &= UART_BUF_NUM - 1;
	}
	mod_timer(&up->rx_point_timer, jiffies + msecs_to_jiffies(READ_BUFF_TIME));
	return 0;
}
#endif

extern int serial_send_ack(char * data, int len);
int isCePowerOff()
{
	return cePowerOffFlag;
}

//can't set time in interrupt, so in workquene
void serial_rk_point(struct work_struct *work)
{
	//printk(KERN_INFO "----bonovo serial_rk_point----\n");
	rtc_tm_to_time(&rtc_current_rtc_time, &tv.tv_sec);

	bonovo_set_rtc(tv);

	bonovo_update_time_key();
}

unsigned int calculateSum(unsigned char* cmdBuf, int size)
{
    unsigned int temp = 0;
    int i;
    for(i=0; i < size; i++){
        temp += cmdBuf[i];
    }
    return temp;
}

void sync_mcu_time(struct timespec time)
{
    struct rtc_time rtc_new_rtc_time;
    unsigned int check_sum;
    unsigned char buf[15] = {0xFA, 0xFA, 0x0F, 0x00, 0xA4};

    rtc_time_to_tm(time.tv_sec, &rtc_new_rtc_time);
    buf[5] = (rtc_new_rtc_time.tm_year+1900) & 0x00FF;
    buf[6] = ((rtc_new_rtc_time.tm_year+1900) >> 8) & 0x00FF;
    buf[7] = rtc_new_rtc_time.tm_mon+1;
    buf[8] = rtc_new_rtc_time.tm_wday;
    buf[9] = rtc_new_rtc_time.tm_mday;
    buf[10]= rtc_new_rtc_time.tm_hour;
    buf[11]= rtc_new_rtc_time.tm_min;
    buf[12]= rtc_new_rtc_time.tm_sec;

    check_sum = calculateSum(buf, 13);
    buf[13]= check_sum & 0x00FF;
    buf[14]= (check_sum >> 8) & 0x00FF;
    serial_send_ack(buf, sizeof(buf));
}

static void serial_rk_point_timeout(unsigned long uart)
{
	struct uart_rk_port *up = (struct uart_rk_port *)uart;

	//spin_lock(&(up->read_lock));
	//read_serial_buffer(up);
	//spin_unlock(&(up->read_lock));
#if USE_ONE_BUFF
	read_serial_buffer(up);
#else
	read_serial_frame(up);
#endif
}
//*************************************************************************//


static void
receive_chars(struct uart_rk_port *up, unsigned int *status)
{
	struct tty_struct *tty = up->port.state->port.tty;
	unsigned char ch, lsr = *status;
	int max_count = 256;
	char flag;
//****************************************************************
//* Add by zbiao for android box
//****************************************************************
	//int count=0;
	//int ret_val;
//****************************************************************

	do {
		if (likely(lsr & UART_LSR_DR)){
			ch = serial_in(up, UART_RX);
		}
		else
			/*
			 * Intel 82571 has a Serial Over Lan device that will
			 * set UART_LSR_BI without setting UART_LSR_DR when
			 * it receives a break. To avoid reading from the
			 * receive buffer without UART_LSR_DR bit set, we
			 * just force the read character to be 0
			 */
			ch = 0;

		flag = TTY_NORMAL;
		up->port.icount.rx++;

		lsr |= up->lsr_saved_flags;
		up->lsr_saved_flags = 0;

		if (unlikely(lsr & UART_LSR_BRK_ERROR_BITS)) {
			/*
			 * For statistics only
			 */
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE)
				up->port.icount.parity++;
			else if (lsr & UART_LSR_FE)
				up->port.icount.frame++;
			if (lsr & UART_LSR_OE)
				up->port.icount.overrun++;


			/*
			 * Mask off conditions which should be ignored.
			 */
			lsr &= up->port.read_status_mask;

			if (lsr & UART_LSR_BI) {
				DEBUG_INTR("handling break....");
				flag = TTY_BREAK;
			} else if (lsr & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}
		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;

//****************************************************************
//* Add by zbiao for android box
//****************************************************************
		if(up->port.line == 3){
			#if USE_ONE_BUFF		//by dm
				m_receive_buffer[count++] = ch;
			#else
				receive_int_char(ch);
			#endif
		}else{
//****************************************************************	

		uart_insert_char(&up->port, lsr, UART_LSR_OE, ch, flag);

//* Add by zbiao for android box
		}
//****************************************************************
ignore_char:
		lsr = serial_in(up, UART_LSR);
	} while ((lsr & (UART_LSR_DR | UART_LSR_BI)) && (max_count-- > 0));
//****************************************************************
//* Add by zbiao for android box
//****************************************************************
	if(up->port.line != 3){
//****************************************************************
	spin_unlock(&up->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&up->port.lock);
//****************************************************************
//* Add by zbiao for android box
//****************************************************************
	}else{
	#if USE_ONE_BUFF
		write_serial_buffer(count); 	//by dm
	#else
	/*
	// When we using frame length as the only standard to judge if we have
	// received one whole frame, we don't need to call check_frame_validation()
	// here. If we using double 0xFA as the terminal of previous frame, we must
	// call check_frame_validation() here, because maybe we have already 
	// received one whole frame, but have not received double 0xFA yet when return
	// from the interrupt routine.
		ret_val = check_frame_validation();
		if (ret_val == 0 || ret_val == -1)
		{
			//printk("----receive char uart3_buf.flag = %d ----\n",uart3_buf.flag);
			uart3_buf.flag = 0;
		}
		else
		{
			//printk("--ret_val== -2 len=%d idx=%d --\n",uart3_buf.cur_frame_len, uart3_buf.idx_in_buf);
		}
	*/
	#endif
	}
//****************************************************************
	*status = lsr;
}

static void transmit_chars(struct uart_rk_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count;

	if (up->port.x_char) {
		serial_out(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_tx_stopped(&up->port)) {
		__stop_tx(up);
		return;
	}
	if (uart_circ_empty(xmit)) {
		__stop_tx(up);
		return;
	}
#if USE_DMA
	//hhb
	if(up->dma->use_dma & TX_DMA){
		if(SERIAL_CIRC_CNT_TO_END(xmit) >= DMA_TX_TRRIGE_LEVEL){
			serial_rk_start_tx_dma(&up->port);
			return;
		}
	}
#endif
	count = up->port.fifosize - serial_in(up , 0x20);
//****************************************************************
//* Add by zbiao for android box
//****************************************************************
	if(up->port.line == 3){
		spin_lock(&up->write_lock);

#if UART3_SEND_LOG
		printk(KERN_INFO "get the bonovo_tx_lock  in transmit_chars function.\n");
#endif
	}
//****************************************************************
	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);
//****************************************************************
//* Add by zbiao for android box
//****************************************************************
	if(up->port.line == 3){
		spin_unlock(&up->write_lock);

#if UART3_SEND_LOG
		printk(KERN_INFO "release the bonovo_tx_lock in transmit_chars function.\n");
#endif
	}
//****************************************************************

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	DEBUG_INTR("THRE...");
#if USE_DMA
	up->port_activity = jiffies;
#endif
	if (uart_circ_empty(xmit))
		__stop_tx(up);
}

static unsigned int check_modem_status(struct uart_rk_port *up)
{
	unsigned int status = serial_in(up, UART_MSR);

#if 0
	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;
	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.state != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

		wake_up_interruptible(&up->port.state->port.delta_msr_wait);
	}
#endif

	return status;
}


/*
 * This handles the interrupt from one port.
 */
static void serial_rk_handle_port(struct uart_rk_port *up)
{
	unsigned int status;
	unsigned long flags;
	spin_lock_irqsave(&up->port.lock, flags);

	/* reading UART_LSR can automatically clears PE FE OE bits, except receive fifo error bit*/
	status = serial_in(up, UART_LSR);

	DEBUG_INTR("status = %x...\n", status);
#if USE_DMA
	/* DMA mode enable */
	if(up->dma->use_dma) {

		if (status & UART_LSR_RFE) {
			if(up->port.line != DBG_PORT){
				dev_info(up->port.dev, "error:lsr=0x%x\n", status);
				status = serial_in(up, UART_LSR);
				DEBUG_INTR("error:lsr=0x%x\n", status);
			}
		}

		if (status & 0x02) {
			if(up->port.line != DBG_PORT){
				dev_info(up->port.dev, "error:lsr=0x%x\n", status);
				status = serial_in(up, UART_LSR);
				DEBUG_INTR("error:lsr=0x%x\n", status);
			}
		}

		if(!(up->dma->use_dma & RX_DMA)) {
			if (status & (UART_LSR_DR | UART_LSR_BI)) {
				receive_chars(up, &status);
			} else if ((up->iir & 0x0f) == 0x0c) {
            	serial_in(up, UART_RX);
        	}
		}

		if ((up->iir & 0x0f) == 0x02) {
			transmit_chars(up);
		}
	} else 
#endif
	{   //dma mode disable

		/*
		 * when uart receive a serial of data which doesn't have stop bit and so on, that causes frame error,and
		 * set UART_LSR_RFE to one,what is worse,we couldn't read the data in the receive fifo. So if
		 * wo don't clear this bit and reset the receive fifo, the received data available interrupt would
		 * occur continuously.  added by hhb@rock-chips.com 2011-08-05
		 */

		if (status & UART_LSR_RFE) {
			if(up->port.line != DBG_PORT){
				dev_info(up->port.dev, "error:lsr=0x%x\n", status);
				status = serial_in(up, UART_LSR);
				DEBUG_INTR("error:lsr=0x%x\n", status);
				rk29_uart_dump_register(up);
			}
		}

		if (status & (UART_LSR_DR | UART_LSR_BI)) {
			receive_chars(up, &status);
		} else if ((up->iir & 0x0f) == 0x0c) {
            serial_in(up, UART_RX);
        }
		check_modem_status(up);
		//hhb@rock-chips.com when FIFO and THRE mode both are enabled,and FIFO TX empty trigger is set to larger than 1,
		//,we need to add ((up->iir & 0x0f) == 0x02) to transmit_chars,because when entering interrupt,the FIFO and THR
		//might not be 1.
		if ((up->iir & 0x0f) == 0x02) {
			transmit_chars(up);
		}
	}
	spin_unlock_irqrestore(&up->port.lock, flags);
}

/*
 * This is the serial driver's interrupt routine.
 */

static irqreturn_t serial_rk_interrupt(int irq, void *dev_id)
{
	struct uart_rk_port *up = dev_id;
	int handled = 0;
	unsigned int iir;

	iir = serial_in(up, UART_IIR);

	DEBUG_INTR("%s(%d) iir = 0x%02x\n", __func__, irq, iir);
	up->iir = iir;

	if (!(iir & UART_IIR_NO_INT)) {
		serial_rk_handle_port(up);
		handled = 1;
	} else if ((iir & UART_IIR_BUSY) == UART_IIR_BUSY) {

		/* The DesignWare APB UART has an Busy Detect (0x07)
		 * interrupt meaning an LCR write attempt occured while the
		 * UART was busy. The interrupt must be cleared by reading
		 * the UART status register (USR) and the LCR re-written. */

		if(!(serial_in(up, UART_USR) & UART_USR_BUSY)){
			serial_out(up, UART_LCR, up->lcr);
		}
		handled = 1;
		dbg("the serial.%d is busy\n", up->port.line);
	}
	DEBUG_INTR("end(%d).\n", handled);

	return IRQ_RETVAL(handled);
}

static unsigned int serial_rk_tx_empty(struct uart_port *port)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);
	unsigned long flags;
	unsigned int lsr;

	dev_dbg(port->dev, "%s\n", __func__);
	spin_lock_irqsave(&up->port.lock, flags);
	lsr = serial_in(up, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return (lsr & UART_LSR_TEMT) == UART_LSR_TEMT ? TIOCSER_TEMT : 0;
}

static unsigned int serial_rk_get_mctrl(struct uart_port *port)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);
	unsigned int status;
	unsigned int ret;

	status = check_modem_status(up);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	dev_dbg(port->dev, "%s 0x%08x\n", __func__, ret);
	return ret;
}

static void serial_rk_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);
	unsigned char mcr = 0;

	dev_dbg(port->dev, "+%s\n", __func__);
	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr |= up->mcr;

	serial_out(up, UART_MCR, mcr);
	dev_dbg(port->dev, "-serial.%d %s mcr: 0x%02x\n", port->line, __func__, mcr);
}

static void serial_rk_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);
	unsigned long flags;

	dev_dbg(port->dev, "+%s\n", __func__);
	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_lcr_write(up, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
	dev_dbg(port->dev, "-%s lcr: 0x%02x\n", __func__, up->lcr);
}

#if defined(CONFIG_SERIAL_RK_CONSOLE) || defined(CONFIG_CONSOLE_POLL)
/*
 *	Wait for transmitter & holding register to empty
 */
static void wait_for_xmitr(struct uart_rk_port *up, int bits)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	for (;;) {
		status = serial_in(up, UART_LSR);

		up->lsr_saved_flags |= status & LSR_SAVE_FLAGS;

		if ((status & bits) == bits)
			break;
		if (--tmout == 0)
			break;
		udelay(1);
	}
}
#endif

#ifdef CONFIG_CONSOLE_POLL
/*
 * Console polling routines for writing and reading from the uart while
 * in an interrupt or debug context.
 */

static int serial_rk_get_poll_char(struct uart_port *port)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);
	unsigned char lsr = serial_in(up, UART_LSR);

	while (!(lsr & UART_LSR_DR))
		lsr = serial_in(up, UART_LSR);

	return serial_in(up, UART_RX);
}

static void serial_rk_put_poll_char(struct uart_port *port,
			 unsigned char c)
{
	unsigned int ier;
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);

	/*
	 *	First save the IER then disable the interrupts
	 */
	ier = serial_in(up, UART_IER);
	serial_out(up, UART_IER, 0);

	wait_for_xmitr(up, UART_LSR_TEMT);
	/*
	 *	Send the character out.
	 *	If a LF, also do CR...
	 */
	serial_out(up, UART_TX, c);
	if (c == 10) {
		wait_for_xmitr(up, UART_LSR_TEMT);
		serial_out(up, UART_TX, 13);
	}

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	wait_for_xmitr(up, UART_LSR_TEMT);
	serial_out(up, UART_IER, ier);
}

#endif /* CONFIG_CONSOLE_POLL */

static int serial_rk_startup(struct uart_port *port)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);
	unsigned long flags;
	int retval, fifosize = 0;
	

	dev_dbg(port->dev, "%s\n", __func__);

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(up->port.irq, serial_rk_interrupt, up->port.irqflags,
				up->name, up);
	if (retval)
		return retval;

	up->mcr = 0;

	clk_enable(up->pclk);
	clk_enable(up->clk);  // enable the config uart clock

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial_rk_clear_fifos(up);

	//read uart fifo size  hhb@rock-chips.com
	fifosize = __raw_readl(up->port.membase + 0xf4);
	up->port.fifosize = ((fifosize >> 16) & 0xff) << 4;
	if(up->port.fifosize <= 0)
		up->port.fifosize = 32;
	//printk("fifo size:%d :%08x\n", up->port.fifosize, fifosize);

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);
	(void) serial_in(up, UART_USR);

	/*
	 * Now, initialize the UART
	 */
	serial_lcr_write(up, UART_LCR_WLEN8 | UART_LCR_EPAR);

	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Most PC uarts need OUT2 raised to enable interrupts.
	 */
//	up->port.mctrl |= TIOCM_OUT2;

	serial_rk_set_mctrl(&up->port, up->port.mctrl);

	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Clear the interrupt registers again for luck, and clear the
	 * saved flags to avoid getting false values from polling
	 * routines or the previous session.
	 */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);
	(void) serial_in(up, UART_USR);
	up->lsr_saved_flags = 0;
#if 0
	up->msr_saved_flags = 0;
#endif
#if USE_DMA
	if (up->dma->use_dma & TX_DMA) {
		if(up->port.state->xmit.buf != up->dma->tx_buffer){
			free_page((unsigned long)up->port.state->xmit.buf);
			up->port.state->xmit.buf = up->dma->tx_buffer;
		}
	} else 
#endif
	{
		up->ier = 0;
		serial_out(up, UART_IER, up->ier);
	}

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */

	return 0;
}


static void serial_rk_shutdown(struct uart_port *port)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);
	unsigned long flags;

	dev_dbg(port->dev, "%s\n", __func__);
	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_out(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
//	up->port.mctrl &= ~TIOCM_OUT2;
	serial_rk_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_lcr_write(up, serial_in(up, UART_LCR) & ~UART_LCR_SBC);
	serial_rk_clear_fifos(up);

	/*
	 * Read data port to reset things, and then free the irq
	 */
	(void) serial_in(up, UART_RX);
#if USE_DMA
	if (up->dma->use_dma & TX_DMA)
		up->port.state->xmit.buf = NULL;
#endif
	free_irq(up->port.irq, up);
	clk_disable(up->clk);
	clk_disable(up->pclk);
}

static void
serial_rk_set_termios(struct uart_port *port, struct ktermios *termios,
		      struct ktermios *old)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);
	unsigned char cval = 0, fcr = 0, mcr = 0;
	unsigned long flags;
	unsigned int baud, quot;
	dev_dbg(port->dev, "+%s\n", __func__);

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	case CS8:
	default:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB){
		cval |= UART_LCR_STOP;
	}
	if (termios->c_cflag & PARENB){
		cval |= UART_LCR_PARITY;
	}
	if (!(termios->c_cflag & PARODD)){
		cval |= UART_LCR_EPAR;
	}
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif


	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 16 / 0xffff,
				  port->uartclk / 16);

	quot = uart_get_divisor(port, baud);
	//dev_info(up->port.dev, "uartclk:%d\n", port->uartclk/16);
	//dev_info(up->port.dev, "baud:%d\n", baud);
	//dev_info(up->port.dev, "quot:%d\n", quot);

	if (baud < 2400){
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
	}
	else{
		fcr = UART_FCR_ENABLE_FIFO;
#if USE_DMA
		//added by hhb@rock-chips.com
		if(up->dma->use_dma & TX_DMA){
			fcr |= UART_FCR_T_TRIG_01;
		} else
#endif
		{
			fcr |= UART_FCR_T_TRIG_01;
		}

#if USE_DMA
		//added by hhb@rock-chips.com
		if(up->dma->use_dma & RX_DMA){	
			fcr |= UART_FCR_R_TRIG_00;
		} else
#endif
		{
			if (termios->c_cflag & CRTSCTS)
				fcr |= UART_FCR_R_TRIG_11;
			else
				fcr |= UART_FCR_R_TRIG_00;
		}
	}

	/*
	 * MCR-based auto flow control.  When AFE is enabled, RTS will be
	 * deasserted when the receive FIFO contains more characters than
	 * the trigger, or the MCR RTS bit is cleared.  In the case where
	 * the remote UART is not using CTS auto flow control, we must
	 * have sufficient FIFO entries for the latency of the remote
	 * UART to respond.  IOW, at least 32 bytes of FIFO.
	 */
	up->mcr &= ~UART_MCR_AFE;
	if (termios->c_cflag & CRTSCTS){
		up->mcr |= UART_MCR_AFE;
	}

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
#if 0
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;
#endif

	//to avoid uart busy when set baud rate  hhb@rock-chips.com
	serial_out(up, UART_SRR, UART_RESET);
	mcr = serial_in(up, UART_MCR);
	serial_out(up, UART_MCR, mcr | 0x10);  //loopback mode
	
	up->lcr = cval;				/* Save LCR */
	/* set DLAB */
	if(serial_lcr_write(up, cval | UART_LCR_DLAB)) {
		if(up->port.line != DBG_PORT)
			dbg("serial.%d set DLAB fail\n", up->port.line);
		serial_out(up, UART_SRR, UART_RESET);
		goto fail;
	}

	/* set uart baud rate */
	if(serial_dl_write(up, quot)) {
		if(up->port.line != DBG_PORT)
			dbg("serial.%d set dll fail\n", up->port.line);
		serial_out(up, UART_SRR, UART_RESET);
		goto fail;
	}

	/* reset DLAB */
	if(serial_lcr_write(up, cval)) {
		if(up->port.line != DBG_PORT)
			dbg("serial.%d reset DLAB fail\n", up->port.line);
		serial_out(up, UART_SRR, UART_RESET);
		goto fail;
	}
	else {
		serial_rk_set_mctrl(&up->port, up->port.mctrl);
		up->fcr = fcr;
		serial_out(up, UART_FCR, up->fcr);		/* set fcr */
		up->ier = 0;
		//start serial receive data
#if USE_DMA
		if (up->dma->use_dma) {
			up->ier |= UART_IER_RLSI;
			up->ier |= UART_IER_PTIME;   //Programmable THRE Interrupt Mode Enable
			if (up->dma->use_dma & RX_DMA)
				serial_rk_start_rx_dma(&up->port);
			else
				up->ier |= UART_IER_RDI;
		} else
#endif
		{
			//  not use dma receive
			up->ier |= UART_IER_RDI;
			up->ier |= UART_IER_RLSI;
			if(up->port.line != DBG_PORT)
				up->ier |= UART_IER_PTIME;   //Programmable THRE Interrupt Mode Enable

		}
		serial_out(up, UART_IER, up->ier);
	}
	
	spin_unlock_irqrestore(&up->port.lock, flags);

	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);
	dev_dbg(port->dev, "-%s baud %d\n", __func__, baud);
	return;

fail:
	spin_unlock_irqrestore(&up->port.lock, flags);

}

#if 0
static void
serial_rk_set_ldisc(struct uart_port *port, int new)
{
	if (new == N_PPS) {
		port->flags |= UPF_HARDPPS_CD;
		serial_rk_enable_ms(port);
	} else
		port->flags &= ~UPF_HARDPPS_CD;
}
#endif

static void
serial_rk_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);

	dev_dbg(port->dev, "%s: %s\n", __func__, state ? "disable" : "enable");
	if (state) {
		clk_disable(up->clk);
		clk_disable(up->pclk);
	} else {
		clk_enable(up->pclk);
		clk_enable(up->clk);
	}
}

static void serial_rk_release_port(struct uart_port *port)
{
	dev_dbg(port->dev, "%s\n", __func__);
}

static int serial_rk_request_port(struct uart_port *port)
{
	dev_dbg(port->dev, "%s\n", __func__);
	return 0;
}

static void serial_rk_config_port(struct uart_port *port, int flags)
{
	dev_dbg(port->dev, "%s\n", __func__);
	port->type = PORT_RK;
}

static int
serial_rk_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	dev_dbg(port->dev, "%s\n", __func__);
	return -EINVAL;
}

static const char *
serial_rk_type(struct uart_port *port)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);

	dev_dbg(port->dev, "%s: %s\n", __func__, up->name);
	return up->name;
}

static struct uart_ops serial_rk_pops = {
	.tx_empty	= serial_rk_tx_empty,
	.set_mctrl	= serial_rk_set_mctrl,
	.get_mctrl	= serial_rk_get_mctrl,
	.stop_tx	= serial_rk_stop_tx,
	.start_tx	= serial_rk_start_tx,
	.stop_rx	= serial_rk_stop_rx,
	.enable_ms	= serial_rk_enable_ms,
	.break_ctl	= serial_rk_break_ctl,
	.startup	= serial_rk_startup,
	.shutdown	= serial_rk_shutdown,
	.set_termios	= serial_rk_set_termios,
#if 0
	.set_ldisc	= serial_rk_set_ldisc,
#endif
	.pm		= serial_rk_pm,
	.type		= serial_rk_type,
	.release_port	= serial_rk_release_port,
	.request_port	= serial_rk_request_port,
	.config_port	= serial_rk_config_port,
	.verify_port	= serial_rk_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char = serial_rk_get_poll_char,
	.poll_put_char = serial_rk_put_poll_char,
#endif
};

#ifdef CONFIG_SERIAL_RK_CONSOLE

static struct uart_rk_port *serial_rk_console_ports[UART_NR];

static void serial_rk_console_putchar(struct uart_port *port, int ch)
{
	struct uart_rk_port *up =
		container_of(port, struct uart_rk_port, port);

	wait_for_xmitr(up, UART_LSR_THRE);
	serial_out(up, UART_TX, ch);
}

/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void
serial_rk_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_rk_port *up = serial_rk_console_ports[co->index];
	unsigned long flags;
	unsigned int ier;
	int locked = 1;

	touch_nmi_watchdog();

	local_irq_save(flags);
	if (up->port.sysrq) {
		/* serial_rk_handle_port() already took the lock */
		locked = 0;
	} else if (oops_in_progress) {
		locked = spin_trylock(&up->port.lock);
	} else
		spin_lock(&up->port.lock);

	/*
	 *	First save the IER then disable the interrupts
	 */
	ier = serial_in(up, UART_IER);

	serial_out(up, UART_IER, 0);

	uart_console_write(&up->port, s, count, serial_rk_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	wait_for_xmitr(up, UART_LSR_TEMT);
	serial_out(up, UART_IER, ier);

#if 0
	/*
	 *	The receive handling will happen properly because the
	 *	receive ready bit will still be set; it is not cleared
	 *	on read.  However, modem control will not, we must
	 *	call it if we have saved something in the saved flags
	 *	while processing with interrupts off.
	 */
	if (up->msr_saved_flags)
		check_modem_status(up);
#endif

	if (locked)
		spin_unlock(&up->port.lock);
	local_irq_restore(flags);
}

static int __init serial_rk_console_setup(struct console *co, char *options)
{
	struct uart_rk_port *up;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (unlikely(co->index >= UART_NR || co->index < 0))
		return -ENODEV;

	if (serial_rk_console_ports[co->index] == NULL)
		return -ENODEV;
	up = serial_rk_console_ports[co->index];

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&up->port, co, baud, parity, bits, flow);
}

static struct console serial_rk_console = {
	.name		= "ttyS",
	.write		= serial_rk_console_write,
	.device		= uart_console_device,
	.setup		= serial_rk_console_setup,
	.flags		= CON_PRINTBUFFER | CON_ANYTIME,
	.index		= -1,
	.data		= &serial_rk_reg,
};

static void serial_rk_add_console_port(struct uart_rk_port *up)
{
	serial_rk_console_ports[up->pdev->id] = up;
}

#define SERIAL_CONSOLE	&serial_rk_console
#else
#define SERIAL_CONSOLE	NULL

static inline void serial_rk_add_console_port(struct uart_rk_port *up)
{}

#endif

static struct uart_driver serial_rk_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "rk29_serial",
	.dev_name		= "ttyS",
	.major			= TTY_MAJOR,
	.minor			= 64,
	.cons			= SERIAL_CONSOLE,
	.nr			= UART_NR,
};
#if USE_WAKEUP
static irqreturn_t serial_rk_wakeup_handler(int irq, void *dev) {
	struct uart_rk_port *up = dev;
	struct uart_wake_up *wakeup = up->wakeup;
	if(wakeup->enable == 1) {
		iomux_set(wakeup->rx_mode);
		wake_lock_timeout(&wakeup->wakelock, 3 * HZ);
	}    
	return 0;
}

static int serial_rk_setup_wakeup_irq(struct uart_rk_port *up)
{
	int ret = 0;
	struct uart_wake_up *wakeup = up->wakeup;

	if(wakeup->enable == 1) {
		memset(wakeup->wakelock_name, 0, 32);
		sprintf(wakeup->wakelock_name, "serial.%d_wakelock", up->port.line);
		wake_lock_init(&wakeup->wakelock, WAKE_LOCK_SUSPEND, wakeup->wakelock_name);
		memset(wakeup->rx_pin_name, 0, 32);		
		sprintf(wakeup->rx_pin_name, "UART%d_SIN", up->port.line);
		wakeup->rx_pin = iomux_mode_to_gpio(wakeup->rx_mode);
		ret = gpio_request(wakeup->rx_pin, wakeup->rx_pin_name);
		if (ret) {
			printk("request %s fail ! \n", wakeup->rx_pin_name);
		    return ret;
		}
		gpio_direction_input(wakeup->rx_pin);
		wakeup->rx_irq = gpio_to_irq(wakeup->rx_pin);
		memset(wakeup->rx_irq_name, 0, 32);
		sprintf(wakeup->rx_irq_name, "serial.%d_wake_up_irq", up->port.line);
		ret = request_irq(wakeup->rx_irq, serial_rk_wakeup_handler, IRQF_TRIGGER_FALLING, wakeup->rx_irq_name, up);
		if(ret < 0) {
			printk("%s request fail\n", wakeup->rx_irq_name);
		    return ret;
	 	}
		disable_irq_nosync(wakeup->rx_irq);
		enable_irq_wake(wakeup->rx_irq);
		iomux_set(wakeup->rx_mode);
	}
	return ret;
}

static int serial_rk_enable_wakeup_irq(struct uart_rk_port *up) {
	struct uart_wake_up *wakeup = up->wakeup;
	if(wakeup->enable == 1) {
		iomux_set(wakeup->rx_mode & 0xfff0);
		enable_irq(wakeup->rx_irq);
	}
    return 0;
}

static int serial_rk_disable_wakeup_irq(struct uart_rk_port *up) {
	struct uart_wake_up *wakeup = up->wakeup;
	if(wakeup->enable == 1) {
		disable_irq_nosync(wakeup->rx_irq);
		iomux_set(wakeup->rx_mode);
	}
    return 0;
}

static int serial_rk_remove_wakeup_irq(struct uart_rk_port *up) {
	struct uart_wake_up *wakeup = up->wakeup;
	if(wakeup->enable == 1) {
		//disable_irq_nosync(wakeup->rx_irq);
		free_irq(wakeup->rx_irq, NULL);
		gpio_free(wakeup->rx_pin);
		wake_lock_destroy(&wakeup->wakelock);
	}
    return 0;
}
#endif



static int __devinit serial_rk_probe(struct platform_device *pdev)
{
	struct uart_rk_port	*up;
	struct resource		*mem;
	int irq;
	int ret = -ENOSPC;
	
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq;
	}

	if (!request_mem_region(mem->start, (mem->end - mem->start) + 1,
				pdev->dev.driver->name)) {
		dev_err(&pdev->dev, "memory region already claimed\n");
		return -EBUSY;
	}

	up = kzalloc(sizeof(*up), GFP_KERNEL);
	if (up == NULL) {
		ret = -ENOMEM;
		goto do_release_region;
	}

	sprintf(up->name, "rk29_serial.%d", pdev->id);
	up->pdev = pdev;
	up->pclk = clk_get(&pdev->dev, "pclk_uart");
	up->clk = clk_get(&pdev->dev, "uart");
	if (unlikely(IS_ERR(up->clk))) {
		ret = PTR_ERR(up->clk);
		goto do_free;
	}
	up->tx_loadsz = 30;
#if USE_DMA
	up->dma = &rk29_uart_ports_dma[pdev->id];
#endif
#if USE_WAKEUP
	up->wakeup = &rk29_uart_ports_wakeup[pdev->id];
#endif
	up->port.dev = &pdev->dev;
	up->port.type = PORT_RK;
	up->port.irq = irq;
	up->port.iotype = UPIO_DWAPB;

	up->port.regshift = 2;
	//fifo size default is 32, but it will be updated later when start_up
	up->port.fifosize = 32;
	up->port.ops = &serial_rk_pops;
	up->port.line = pdev->id;
	up->port.iobase = mem->start;
	up->port.membase = ioremap_nocache(mem->start, mem->end - mem->start + 1);
	if (!up->port.membase) {
		ret = -ENOMEM;
		goto do_put_clk;
	}
	up->port.mapbase = mem->start;
	up->port.irqflags = IRQF_DISABLED;
	up->port.uartclk = clk_get_rate(up->clk);

#if USE_DMA
	/* set dma config */
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	if(up->dma->use_dma & RX_DMA) {
		//timer
		up->dma->use_timer = USE_TIMER;
		up->dma->rx_timer.function = serial_rk_report_dma_rx;
		up->dma->rx_timer.data = (unsigned long)up;
		up->dma->rx_timeout = 10;
		up->dma->rx_timer.expires = jiffies + msecs_to_jiffies(up->dma->rx_timeout);
		init_timer(&up->dma->rx_timer);

		//rx buffer
		up->dma->rb_size =  UART_XMIT_SIZE*2;
		up->dma->rx_buffer = dmam_alloc_coherent(up->port.dev, up->dma->rb_size,
				&up->dma->rx_phy_addr, DMA_MEMORY_MAP);
		up->dma->rb_tail = 0;

		if(!up->dma->rx_buffer){
			dev_info(up->port.dev, "dmam_alloc_coherent dma_rx_buffer fail\n");
		}
		else {
			dev_info(up->port.dev, "dma_rx_buffer 0x%08x\n", (unsigned) up->dma->rx_buffer);
			dev_info(up->port.dev, "dma_rx_phy 0x%08x\n", (unsigned)up->dma->rx_phy_addr);
		}

		// work queue
		//INIT_WORK(&up->uart_work, serial_rk_report_revdata_workfunc);
		//INIT_WORK(&up->uart_work_rx, serial_rk_start_dma_rx);
		//up->uart_wq = create_singlethread_workqueue("uart_workqueue");
		up->dma->rx_dma_used = 0;
		spin_lock_init(&(up->dma->rx_lock));
		serial_rk_init_dma_rx(&up->port);
	}

	if(up->dma->use_dma & TX_DMA){
		//tx buffer
		up->dma->tb_size = UART_XMIT_SIZE;
		up->dma->tx_buffer = dmam_alloc_coherent(up->port.dev, up->dma->tb_size,
				&up->dma->tx_phy_addr, DMA_MEMORY_MAP);
		if(!up->dma->tx_buffer){
			dev_info(up->port.dev, "dmam_alloc_coherent dma_tx_buffer fail\n");
		}
		else{
			dev_info(up->port.dev, "dma_tx_buffer 0x%08x\n", (unsigned) up->dma->tx_buffer);
			dev_info(up->port.dev, "dma_tx_phy 0x%08x\n", (unsigned) up->dma->tx_phy_addr);
		}
		spin_lock_init(&(up->dma->tx_lock));
		serial_rk_init_dma_tx(&up->port);
	}

#endif
	//************* bonovo android box ***************//
	if(up->port.line == 3){
		INIT_WORK(&up->uart_work_point, serial_rk_point);
		up->uart_wq_point = create_singlethread_workqueue("uart_workqueue_point");

		//bonovo timer by dm
		#if !defined(USE_ONE_BUFF)
		init_uart3_buf_ctrl();
		#endif
		up->rx_point_timer.function = serial_rk_point_timeout;
		up->rx_point_timer.data = (unsigned long)up;
		up->rx_point_timer.expires = jiffies + msecs_to_jiffies(200);
		init_timer(&up->rx_point_timer);
		add_timer(&up->rx_point_timer);
		
		spin_lock_init(&(up->read_lock));
	}

	serial_rk_add_console_port(up);
	ret = uart_add_one_port(&serial_rk_reg, &up->port);
	if (ret != 0)
		goto do_iounmap;

	platform_set_drvdata(pdev, up);
	dev_info(&pdev->dev, "membase 0x%08x\n", (unsigned) up->port.membase);
#if USE_WAKEUP
	serial_rk_setup_wakeup_irq(up); 
#endif
	return 0;

do_iounmap:
	iounmap(up->port.membase);
	up->port.membase = NULL;
do_put_clk:
	clk_put(up->clk);
	clk_put(up->pclk);
do_free:
	kfree(up);
do_release_region:
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return ret;
}

static int __devexit serial_rk_remove(struct platform_device *pdev)
{
	struct uart_rk_port *up = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	if (up) {
		struct resource	*mem;
		destroy_workqueue(up->uart_wq);
		uart_remove_one_port(&serial_rk_reg, &up->port);
		mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		iounmap(up->port.membase);
		up->port.membase = NULL;
		clk_put(up->clk);
		clk_put(up->pclk);
		kfree(up);
		release_mem_region(mem->start, (mem->end - mem->start) + 1);
	}
#if USE_WAKEUP
    serial_rk_remove_wakeup_irq(up);
#endif
	return 0;
}

static int serial_rk_suspend(struct platform_device *dev, pm_message_t state)
{
	struct uart_rk_port *up = platform_get_drvdata(dev);

	if (up && up->port.line != DBG_PORT && POWER_MANEGEMENT){
		uart_suspend_port(&serial_rk_reg, &up->port);
	}
	if(up->port.line == DBG_PORT && POWER_MANEGEMENT){
		serial_rk_pm(&up->port, 1, 0);
	}
#if USE_WAKEUP
    serial_rk_enable_wakeup_irq(up);
#endif
	return 0;
}

static int serial_rk_resume(struct platform_device *dev)
{
	struct uart_rk_port *up = platform_get_drvdata(dev);
#if USE_WAKEUP
    serial_rk_disable_wakeup_irq(up);
#endif
	if (up && up->port.line != DBG_PORT && POWER_MANEGEMENT){
		uart_resume_port(&serial_rk_reg, &up->port);
	}
	if(up->port.line == DBG_PORT && POWER_MANEGEMENT){
		serial_rk_pm(&up->port, 0, 1);
	}
	return 0;
}

static struct platform_driver serial_rk_driver = {
	.probe		= serial_rk_probe,
	.remove		= __devexit_p(serial_rk_remove),
	.suspend	= serial_rk_suspend,
	.resume		= serial_rk_resume,
	.driver		= {
#if defined(CONFIG_ARCH_RK29)
		.name	= "rk29_serial",
#elif defined(CONFIG_SERIAL_RK2818)
		.name	= "rk2818_serial",
#else
		.name	= "rk_serial",
#endif
		.owner	= THIS_MODULE,
	},
};

static int __init serial_rk_init(void)
{
	int ret;
	//hhb@rock-chips.com
	printk("%s\n", VERSION_AND_TIME);

	ret = uart_register_driver(&serial_rk_reg);
	if (ret)
		return ret;

	ret = platform_driver_register(&serial_rk_driver);
	if (ret != 0)
		uart_unregister_driver(&serial_rk_reg);

	return ret;
}

static void __exit serial_rk_exit(void)
{
	platform_driver_unregister(&serial_rk_driver);
	uart_unregister_driver(&serial_rk_reg);
}

module_init(serial_rk_init);
module_exit(serial_rk_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RK UART driver");

