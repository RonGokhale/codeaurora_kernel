/* drivers/serial/msm_serial.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#if defined(CONFIG_SERIAL_MSM_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#undef SUPPORT_SYSRQ

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <linux/irq.h>
#include "msm_serial.h"

#define UART_NR		3
#define SERIAL_MSM_MAJOR	204
#define SERIAL_MSM_MINOR	16

#define uwr(v, a) writel(v, port->membase + (a))
#define urd(a) readl(port->membase + (a))

/* XXX this is totally disgusting */
#define IMR (port->unused[0])

static void serial_init(struct uart_port *port)
{
#if !defined(CONFIG_SERIAL_MSM_NOINIT)
	printk(KERN_INFO "serial_init() configuring UART port @ %p\n", port);

	/* disable all interrupts */
	uwr(0, UART_IMR);

	uwr(0x30, UART_CR);  /* reset error status */
	uwr(0x10, UART_CR);  /* reset receiver */
	uwr(0x20, UART_CR);  /* reset transmitter */

	/* these should not be different... */
	if (port->line == 2) {
		/* configuration for 19.2MHz TCXO */
		uwr(0x06, UART_MREG);
		uwr(0xF1, UART_NREG);
		uwr(0x0F, UART_DREG);
		uwr(0x1A, UART_MNDREG);
	} else {
		/* valid settings for 115.2 on 3080 SURF build... */
		uwr(0xC0, UART_MREG);
		uwr(0xAF, UART_NREG);
		uwr(0x80, UART_DREG);
		uwr(0x19, UART_MNDREG);
	}

	uwr(0x15, UART_CR);  /* reset RX */
	uwr(0x25, UART_CR);  /* reset TX */
	uwr(0x35, UART_CR);  /* reset error status */
	uwr(0x45, UART_CR);  /* reset RX break */
	uwr(0x75, UART_CR);  /* rest? */
	uwr(0xD5, UART_CR);  /* reset */

	uwr(0x7BF, UART_IPR); /* stale timeout = 630 * bitrate */
	uwr(0, UART_IMR);
	uwr(10, UART_TFWR);  /* TX watermark */

	uwr(0, UART_RFWR); /* no RX watermark -- flag us whenever there is data */

	uwr(UART_CSR_115200, UART_CSR);
	uwr(0, UART_IRDA);
	uwr(0x1E, UART_HCR);

	if (port->line == 0)
		uwr(0x7F4, UART_MR1); /* RFS/ CTS/ 500chr RFR */
	else
		uwr(16, UART_MR1);
	uwr(0x34, UART_MR2); /* 8N1 */

	uwr(0x05, UART_CR); /* enable TX & RX */
#else
	printk(KERN_INFO "serial_init() trusting the bootloader\n");

	uwr(0x15, UART_CR);  /* reset RX */
	uwr(0x25, UART_CR);  /* reset TX */
	uwr(0x35, UART_CR);  /* reset error status */
	uwr(0x45, UART_CR);  /* reset RX break */
	uwr(0x75, UART_CR);  /* rest? */
	uwr(0xD5, UART_CR);  /* reset */

	uwr(0x7BF, UART_IPR); /* stale timeout = 630 * bitrate */
	uwr(0, UART_IMR);    /* no RX watermark -- flag us whenever there is data */
	uwr(10, UART_TFWR);  /* TX watermark */

	uwr(0, UART_RFWR);

       /* don't do a port init */
	uwr(0x05, UART_CR); /* enable TX & RX */
#endif
	IMR = 0;
}


static void msm_stop_tx(struct uart_port *port)
{
	unsigned flags;
	IMR &= ~UART_IMR_TXLEV;
	uwr(IMR, UART_IMR);
}

static void msm_start_tx(struct uart_port *port)
{
	unsigned flags;
	IMR |= UART_IMR_TXLEV;
	uwr(IMR, UART_IMR);
}

static void msm_stop_rx(struct uart_port *port)
{
}

static void msm_enable_ms(struct uart_port *port)
{
}

static void handle_rx(struct uart_port *port)
{
	struct tty_struct *tty = port->info->tty;
	unsigned int status, ch, maxwork = 256;

	for (;;) {
		status = urd(UART_SR);
		if (!(status & UART_SR_RX_READY)) break;
		if (maxwork-- == 0) break;

		ch = urd(UART_RF);
		port->icount.rx++;

#ifdef SUPPORT_SYSRQ
		if (ch == '`') {
			/* let double-taps of ` pass through as a regular ` */
			if (port->sysrq) {
				port->sysrq = 0;
			} else {
				uart_handle_break(port);
				continue;
			}
		}
		if (uart_handle_sysrq_char(port, ch))
			continue;
#endif

		tty_insert_flip_char(tty, ch, TTY_NORMAL);
	}
	tty_flip_buffer_push(tty);
}

static void handle_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->info->xmit;
	unsigned int status, maxwork = 256;

	for (;;) {
		status = urd(UART_SR);

		if (!(status & UART_SR_TX_READY)) break;
		if (maxwork-- == 0) break;

		if (port->x_char) {
			uwr(port->x_char, UART_TF);
			port->icount.tx++;
			port->x_char = 0;
			continue;
		}

		if (uart_circ_empty(xmit)) {
			/* stop tx interrupts */
			IMR &= (~UART_IMR_TXLEV);
			uwr(IMR, UART_IMR);
			break;
		}

		uwr(xmit->buf[xmit->tail], UART_TF);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static irqreturn_t msm_int(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	unsigned sr;

	spin_lock(&port->lock);
	sr = urd(UART_SR);
	if (sr & UART_SR_RX_READY)
		handle_rx(port);
	if (sr & UART_SR_TX_READY)
		if (IMR & UART_IMR_TXLEV)
			handle_tx(port);
	spin_unlock(&port->lock);
	return IRQ_HANDLED;
}

static unsigned int msm_tx_empty(struct uart_port *port)
{
	return TIOCSER_TEMT;
}

static unsigned int msm_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_RTS | TIOCM_DSR | TIOCM_CAR;
}

static void msm_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static void msm_break_ctl(struct uart_port *port, int break_state)
{
}

static int msm_startup(struct uart_port *port)
{
	int retval;

	char port_name[8];
	sprintf(port_name, "uart%d", port->irq);

	retval = request_irq(port->irq, msm_int, 0, port_name, port);
	if (retval) return retval;

	/* enable rx */
	IMR = UART_IMR_RXLEV;
	uwr(IMR, UART_IMR);
	return 0;
}

static void msm_shutdown(struct uart_port *port)
{
	free_irq(port->irq, port);
}

static void
msm_set_termios(struct uart_port *port, struct ktermios *termios,
		struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud, quot;

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = uart_get_divisor(port, baud);

	termios->c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD | CREAD);
	termios->c_cflag |= CS8;

	termios->c_iflag &= ~(INPCK | IGNPAR | IGNBRK | BRKINT);

	spin_lock_irqsave(&port->lock, flags);

	uart_update_timeout(port, termios->c_cflag, baud);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *msm_type(struct uart_port *port)
{
	return port->type == PORT_MSM_SERIAL ? "MSM" : NULL;
}

static void msm_release_port(struct uart_port *port)
{
}

static int msm_request_port(struct uart_port *port)
{
	return 0;
}

/*
 * Configure/autoconfigure the port.
 */
static void msm_config_port(struct uart_port *port, int flags)
{
	printk(KERN_INFO "msm_config_port @ %p\n", port);
	if (flags & UART_CONFIG_TYPE) {
		serial_init(port);
		port->type = PORT_MSM_SERIAL;
		msm_request_port(port);
	}
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int msm_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_MSM_SERIAL)
		ret = -EINVAL;
	if (ser->irq < 0 || ser->irq >= NR_IRQS)
		ret = -EINVAL;
	if (ser->baud_base < 9600)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops msm_uart_pops = {
	.tx_empty	= msm_tx_empty,
	.set_mctrl	= msm_set_mctrl,
	.get_mctrl	= msm_get_mctrl,
	.stop_tx	= msm_stop_tx,
	.start_tx	= msm_start_tx,
	.stop_rx	= msm_stop_rx,
	.enable_ms	= msm_enable_ms,
	.break_ctl	= msm_break_ctl,
	.startup	= msm_startup,
	.shutdown	= msm_shutdown,
	.set_termios	= msm_set_termios,
	.type		= msm_type,
	.release_port	= msm_release_port,
	.request_port	= msm_request_port,
	.config_port	= msm_config_port,
	.verify_port	= msm_verify_port,
};

static struct uart_port msm_ports[UART_NR] = {
	{
		.membase = (void *) MSM_UART1_BASE,
		.mapbase = MSM_UART1_PHYS,
		.irq = INT_UART1,
		.iotype = SERIAL_IO_MEM,
		.uartclk = 14745600,
		.ops = &msm_uart_pops,
		.flags = ASYNC_BOOT_AUTOCONF,
		.line = 0,
	},
	{
		.membase = (void *) MSM_UART2_BASE,
		.mapbase = MSM_UART2_PHYS,
		.irq = INT_UART2,
		.iotype = SERIAL_IO_MEM,
		.uartclk = 14745600,
		.ops = &msm_uart_pops,
		.flags = ASYNC_BOOT_AUTOCONF,
		.line = 1,
	},
	{
		.membase = (void *) MSM_UART3_BASE,
		.mapbase = MSM_UART3_PHYS,
		.irq = INT_UART3,
		.iotype = SERIAL_IO_MEM,
		.uartclk = 14745600,
		.ops = &msm_uart_pops,
		.flags = ASYNC_BOOT_AUTOCONF,
		.line = 2,
	},
};


#ifdef CONFIG_SERIAL_MSM_CONSOLE

static void msm_console_putchar(struct uart_port *port, int ch)
{
	while (!(urd(UART_SR) & UART_SR_TX_READY)) ;
	uwr(ch, UART_TF);
}

static void
msm_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_port *port = &msm_ports[co->index];

	spin_lock(&port->lock);
	uart_console_write(port, s, count, msm_console_putchar);
	spin_unlock(&port->lock);

}

static void __init
msm_console_get_options(struct uart_port *port, int *baud,
			int *parity, int *bits)
{
	*baud = 115200;
	*parity = 'n';
	*bits = 8;
}

static int __init msm_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index >= UART_NR)
		return -ENODEV;

	port = &msm_ports[co->index];

	if (port->membase == 0)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		msm_console_get_options(port, &baud, &parity, &bits);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver msm_uart_driver;

static struct console msm_console = {
	.name		= "ttyMSM",
	.write		= msm_console_write,
	.device		= uart_console_device,
	.setup		= msm_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &msm_uart_driver,
};

#define MSM_CONSOLE	&msm_console
#else
#define MSM_CONSOLE	NULL
#endif

static struct uart_driver msm_uart_driver = {
	.owner          = THIS_MODULE,
	.driver_name    = "ttyMSM",
	.dev_name       = "ttyMSM",
	.major          = SERIAL_MSM_MAJOR,
	.minor          = SERIAL_MSM_MINOR,
	.nr             = UART_NR,
	.cons           = MSM_CONSOLE,
};

static int __devinit msm_serial_probe(struct platform_device *pdev)
{
	int ret;

	if ((pdev->id < 0) || (pdev->id >= UART_NR))
		return -ENODEV;

	return uart_add_one_port(&msm_uart_driver, &msm_ports[pdev->id]);
}

static struct platform_driver msm_serial_driver = {
	.probe = msm_serial_probe,
	.driver = {
		.name = "msm_serial",
		.owner = THIS_MODULE,
	},
};

static int __init msm_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&msm_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&msm_serial_driver);
	if (ret)
		uart_unregister_driver(&msm_uart_driver);

	return ret;
}

module_init(msm_serial_init);
