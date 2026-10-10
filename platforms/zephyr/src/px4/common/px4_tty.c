/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


/**
 * @file px4_tty.c
 *
 * Device files for the UARTs of the px4,serial-ports devicetree node, which
 * PX4 opens with open() and configures with termios, as on NuttX and Linux.
 * open() is wrapped at link time to find them, and the other calls go through
 * the Zephyr file descriptor table.
 *
 * The UART interrupt callback moves the data between the UART and a receive
 * and a transmit ring buffer, and raises the poll signals that read(),
 * write() and poll() wait on.
 */

#include <px4_tty.h>

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <string.h>
#include <sys/termios.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_PX4_SERIAL)

#define PX4_SERIAL_PORTS DT_COMPAT_GET_ANY_STATUS_OKAY(px4_serial_ports)
#define PX4_TTY_UART(node) DT_PHANDLE(node, uart)

struct px4_tty {
	const char *path;
	const struct device *uart;
	struct ring_buf *rx;
	struct ring_buf *tx;
	speed_t default_speed;
	bool flow_control;
	bool usb;

	struct k_spinlock lock;
	struct k_poll_signal readable;
	struct k_poll_signal writable;
	struct k_mutex *fd_lock;
	struct termios termios;
	atomic_t open;
	bool nonblock;

	/* receive statistics, for the px4_tty shell command */
	uint32_t rx_bytes;
	uint32_t rx_dropped;
	uint32_t rx_overruns;
	uint32_t rx_framing_errors;
	uint32_t rx_noise_errors;
	uint32_t rx_parity_errors;
};

#define PX4_TTY_RING_BUF(name, size) RING_BUF_DECLARE(name, size);

#define PX4_TTY_BUFFERS(node)                                                                      \
	PX4_TTY_RING_BUF(_CONCAT(px4_tty_rx_, DT_DEP_ORD(node)), DT_PROP(node, rx_buffer_size))    \
	PX4_TTY_RING_BUF(_CONCAT(px4_tty_tx_, DT_DEP_ORD(node)), DT_PROP(node, tx_buffer_size))

DT_FOREACH_CHILD_STATUS_OKAY(PX4_SERIAL_PORTS, PX4_TTY_BUFFERS)

#define PX4_TTY(node)                                                                              \
	{                                                                                          \
		.path = DT_PROP(node, path),                                                       \
			.uart = DEVICE_DT_GET(PX4_TTY_UART(node)),                                         \
				.rx = &_CONCAT(px4_tty_rx_, DT_DEP_ORD(node)),                                     \
				      .tx = &_CONCAT(px4_tty_tx_, DT_DEP_ORD(node)),                                     \
					    .default_speed = DT_PROP_OR(PX4_TTY_UART(node), current_speed, 115200),            \
							    .flow_control = DT_PROP_OR(PX4_TTY_UART(node), hw_flow_control, false),            \
									    .usb = DT_NODE_HAS_COMPAT(PX4_TTY_UART(node), zephyr_cdc_acm_uart),                \
	},

static struct px4_tty px4_ttys[] = {DT_FOREACH_CHILD_STATUS_OKAY(PX4_SERIAL_PORTS, PX4_TTY)};

#if DT_HAS_CHOSEN(zephyr_console)
#define PX4_TTY_NOT_CONSOLE(node)                                                                  \
	BUILD_ASSERT(!DT_SAME_NODE(PX4_TTY_UART(node), DT_CHOSEN(zephyr_console)),                 \
		     "the console UART cannot be a PX4 serial port");
DT_FOREACH_CHILD_STATUS_OKAY(PX4_SERIAL_PORTS, PX4_TTY_NOT_CONSOLE)
#endif

static void px4_tty_receive(struct px4_tty *tty)
{
	k_spinlock_key_t key = k_spin_lock(&tty->lock);

	for (;;) {
		uint8_t *data;
		uint32_t size = ring_buf_put_ptr(tty->rx, &data, 0);
		int len;

		if (size == 0) {
			/* the receive buffer is full: drop the data */
			uint8_t discard[16];

			len = uart_fifo_read(tty->uart, discard, sizeof(discard));
			tty->rx_dropped += MAX(len, 0);

			if (len < (int)sizeof(discard)) {
				break;
			}

		} else {
			len = uart_fifo_read(tty->uart, data, size);
			ring_buf_commit(tty->rx, MAX(len, 0));
			tty->rx_bytes += MAX(len, 0);

			if (len < (int)size) {
				break;
			}
		}
	}

	if (!ring_buf_is_empty(tty->rx)) {
		k_poll_signal_raise(&tty->readable, 0);
	}

	k_spin_unlock(&tty->lock, key);
}

static void px4_tty_transmit(struct px4_tty *tty)
{
	k_spinlock_key_t key = k_spin_lock(&tty->lock);
	uint8_t *data;
	uint32_t size = ring_buf_get_ptr(tty->tx, &data, 0);

	if (size == 0) {
		uart_irq_tx_disable(tty->uart);

	} else {
		int len = uart_fifo_fill(tty->uart, data, size);

		ring_buf_consume(tty->tx, MAX(len, 0));
		k_poll_signal_raise(&tty->writable, 0);
	}

	k_spin_unlock(&tty->lock, key);
}

static void px4_tty_isr(const struct device *dev, void *user_data)
{
	struct px4_tty *tty = user_data;

	uart_irq_update(dev);

	/* an overrun stops the reception of some UARTs, such as the NXP LPUART, until it is cleared */
	const int err = uart_err_check(dev);

	if (err > 0) {
		tty->rx_overruns += (err & UART_ERROR_OVERRUN) ? 1 : 0;
		tty->rx_framing_errors += (err & UART_ERROR_FRAMING) ? 1 : 0;
		tty->rx_noise_errors += (err & UART_ERROR_NOISE) ? 1 : 0;
		tty->rx_parity_errors += (err & UART_ERROR_PARITY) ? 1 : 0;
	}

	if (uart_irq_rx_ready(dev) > 0) {
		px4_tty_receive(tty);
	}

	if (uart_irq_tx_ready(dev) > 0) {
		px4_tty_transmit(tty);
	}
}

static size_t px4_tty_get(struct px4_tty *tty, void *buf, size_t size)
{
	k_spinlock_key_t key = k_spin_lock(&tty->lock);
	size_t len = ring_buf_get(tty->rx, buf, size);

	if (ring_buf_is_empty(tty->rx)) {
		k_poll_signal_reset(&tty->readable);
	}

	k_spin_unlock(&tty->lock, key);
	return len;
}

static size_t px4_tty_put(struct px4_tty *tty, const void *buf, size_t size)
{
	k_spinlock_key_t key = k_spin_lock(&tty->lock);
	size_t len = ring_buf_put(tty->tx, buf, size);

	if (ring_buf_space_get(tty->tx) == 0) {
		k_poll_signal_reset(&tty->writable);
	}

	k_spin_unlock(&tty->lock, key);

	/* outside of the lock, as the driver may call the callback */
	if (len > 0) {
		uart_irq_tx_enable(tty->uart);
	}

	return len;
}

static int px4_tty_wait(struct px4_tty *tty, struct k_poll_signal *signal)
{
	struct k_poll_event event =
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, signal);

	/* read() and write() hold the lock of the file descriptor: release it
	 * while waiting, not to block the other direction */
	if (tty->fd_lock != NULL) {
		k_mutex_unlock(tty->fd_lock);
	}

	(void)k_poll(&event, 1, K_FOREVER);

	if (tty->fd_lock != NULL) {
		(void)k_mutex_lock(tty->fd_lock, K_FOREVER);
	}

	return atomic_get(&tty->open) ? 0 : -EBADF;
}

/* a USB port sends nothing while the host has not opened it */
static bool px4_tty_connected(struct px4_tty *tty)
{
#if defined(CONFIG_UART_LINE_CTRL)
	uint32_t dtr;

	if (tty->usb && (uart_line_ctrl_get(tty->uart, UART_LINE_CTRL_DTR, &dtr) == 0)) {
		return dtr != 0;
	}

#endif

	return true;
}

static ssize_t px4_tty_read(void *obj, void *buf, size_t size, size_t offset)
{
	struct px4_tty *tty = obj;

	ARG_UNUSED(offset);

	for (;;) {
		size_t len = px4_tty_get(tty, buf, size);

		if ((len > 0) || (size == 0)) {
			return len;
		}

		if (tty->nonblock) {
			errno = EAGAIN;
			return -1;
		}

		int ret = px4_tty_wait(tty, &tty->readable);

		if (ret < 0) {
			errno = -ret;
			return -1;
		}
	}
}

static ssize_t px4_tty_write(void *obj, const void *buf, size_t size, size_t offset)
{
	struct px4_tty *tty = obj;
	const uint8_t *data = buf;
	size_t written = 0;

	ARG_UNUSED(offset);

	if (!px4_tty_connected(tty)) {
		return size;
	}

	for (;;) {
		written += px4_tty_put(tty, data + written, size - written);

		if ((written == size) || (tty->nonblock && (written > 0))) {
			return written;
		}

		if (tty->nonblock) {
			errno = EAGAIN;
			return -1;
		}

		int ret = px4_tty_wait(tty, &tty->writable);

		if (ret < 0) {
			errno = -ret;
			return -1;
		}
	}
}

static int px4_tty_close(void *obj)
{
	struct px4_tty *tty = obj;

	uart_irq_rx_disable(tty->uart);
	uart_irq_tx_disable(tty->uart);
	atomic_clear(&tty->open);

	/* wake up the threads waiting on the port */
	k_poll_signal_raise(&tty->readable, 0);
	k_poll_signal_raise(&tty->writable, 0);

	return 0;
}

static int px4_tty_poll_prepare(struct px4_tty *tty, struct zvfs_pollfd *pfd,
				struct k_poll_event **pev, struct k_poll_event *pev_end)
{
	if (pfd->events & ZVFS_POLLIN) {
		if (*pev == pev_end) {
			return -ENOMEM;
		}

		k_poll_event_init(*pev, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &tty->readable);
		(*pev)++;
	}

	if (pfd->events & ZVFS_POLLOUT) {
		if (*pev == pev_end) {
			return -ENOMEM;
		}

		k_poll_event_init(*pev, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &tty->writable);
		(*pev)++;
	}

	return 0;
}

static int px4_tty_poll_update(struct px4_tty *tty, struct zvfs_pollfd *pfd,
			       struct k_poll_event **pev)
{
	k_spinlock_key_t key = k_spin_lock(&tty->lock);

	if (pfd->events & ZVFS_POLLIN) {
		if (!ring_buf_is_empty(tty->rx)) {
			pfd->revents |= ZVFS_POLLIN;
		}

		(*pev)++;
	}

	if (pfd->events & ZVFS_POLLOUT) {
		if (ring_buf_space_get(tty->tx) > 0) {
			pfd->revents |= ZVFS_POLLOUT;
		}

		(*pev)++;
	}

	k_spin_unlock(&tty->lock, key);
	return 0;
}

static int px4_tty_ioctl(void *obj, unsigned int request, va_list args)
{
	struct px4_tty *tty = obj;

	switch (request) {
	case ZFD_IOCTL_SET_LOCK:
		tty->fd_lock = va_arg(args, struct k_mutex *);
		return 0;

	case ZFD_IOCTL_POLL_PREPARE: {
			struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
			struct k_poll_event **pev = va_arg(args, struct k_poll_event **);
			struct k_poll_event *pev_end = va_arg(args, struct k_poll_event *);

			return px4_tty_poll_prepare(tty, pfd, pev, pev_end);
		}

	case ZFD_IOCTL_POLL_UPDATE: {
			struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
			struct k_poll_event **pev = va_arg(args, struct k_poll_event **);

			return px4_tty_poll_update(tty, pfd, pev);
		}

	case F_GETFL:
		return O_RDWR | (tty->nonblock ? O_NONBLOCK : 0);

	case F_SETFL:
		tty->nonblock = (va_arg(args, int) & O_NONBLOCK) != 0;
		return 0;

	case ZFD_IOCTL_FIONBIO:
		tty->nonblock = *va_arg(args, int *) != 0;
		return 0;

	case ZFD_IOCTL_FIONREAD:
		*va_arg(args, int *) = ring_buf_size_get(tty->rx);
		return 0;

	case PX4_TTY_FIONSPACE:
		*va_arg(args, int *) = ring_buf_space_get(tty->tx);
		return 0;

	default:
		errno = EOPNOTSUPP;
		return -1;
	}
}

static const struct fd_op_vtable px4_tty_vtable = {
	.read_offs = px4_tty_read,
	.write_offs = px4_tty_write,
	.close = px4_tty_close,
	.ioctl = px4_tty_ioctl,
};

static void px4_tty_from_uart_config(const struct uart_config *config, struct termios *termios_p)
{
	static const tcflag_t data_bits[] = {
		[UART_CFG_DATA_BITS_5] = CS5,
		[UART_CFG_DATA_BITS_6] = CS6,
		[UART_CFG_DATA_BITS_7] = CS7,
		[UART_CFG_DATA_BITS_8] = CS8,
	};

	termios_p->c_cflag = CREAD | CLOCAL;
	termios_p->c_cflag |= (config->data_bits < ARRAY_SIZE(data_bits)) ? data_bits[config->data_bits] : CS8;
	termios_p->c_cflag |= (config->parity != UART_CFG_PARITY_NONE) ? PARENB : 0;
	termios_p->c_cflag |= (config->parity == UART_CFG_PARITY_ODD) ? PARODD : 0;
	termios_p->c_cflag |= (config->stop_bits == UART_CFG_STOP_BITS_2) ? CSTOPB : 0;
	termios_p->c_cflag |= (config->flow_ctrl == UART_CFG_FLOW_CTRL_RTS_CTS) ? CRTSCTS : 0;
	termios_p->c_ispeed = config->baudrate;
	termios_p->c_ospeed = config->baudrate;
}

static void px4_tty_to_uart_config(const struct termios *termios_p, struct uart_config *config)
{
	switch (termios_p->c_cflag & CSIZE) {
	case CS5: config->data_bits = UART_CFG_DATA_BITS_5; break;

	case CS6: config->data_bits = UART_CFG_DATA_BITS_6; break;

	case CS7: config->data_bits = UART_CFG_DATA_BITS_7; break;

	default: config->data_bits = UART_CFG_DATA_BITS_8; break;
	}

	if (termios_p->c_cflag & PARENB) {
		config->parity = (termios_p->c_cflag & PARODD) ? UART_CFG_PARITY_ODD : UART_CFG_PARITY_EVEN;

	} else {
		config->parity = UART_CFG_PARITY_NONE;
	}

	config->stop_bits = (termios_p->c_cflag & CSTOPB) ? UART_CFG_STOP_BITS_2 : UART_CFG_STOP_BITS_1;
	config->flow_ctrl = (termios_p->c_cflag & CRTSCTS) ? UART_CFG_FLOW_CTRL_RTS_CTS : UART_CFG_FLOW_CTRL_NONE;
	config->baudrate = termios_p->c_ospeed;
}

static struct px4_tty *px4_tty_find(const char *path)
{
	ARRAY_FOR_EACH_PTR(px4_ttys, tty) {
		if ((path != NULL) && (strcmp(tty->path, path) == 0)) {
			return tty;
		}
	}

	return NULL;
}

static int px4_tty_open(struct px4_tty *tty, int flags)
{
	if (!device_is_ready(tty->uart)) {
		errno = ENODEV;
		return -1;
	}

	if (!atomic_cas(&tty->open, 0, 1)) {
		errno = EBUSY;
		return -1;
	}

	int fd = zvfs_reserve_fd();

	if (fd < 0) {
		atomic_clear(&tty->open);
		return -1;
	}

	struct uart_config config;

	memset(&tty->termios, 0, sizeof(tty->termios));

	if (uart_config_get(tty->uart, &config) == 0) {
		px4_tty_from_uart_config(&config, &tty->termios);

	} else {
		tty->termios.c_cflag = CS8 | CREAD | CLOCAL;
		tty->termios.c_ispeed = tty->default_speed;
		tty->termios.c_ospeed = tty->default_speed;
	}

	ring_buf_reset(tty->rx);
	ring_buf_reset(tty->tx);
	k_poll_signal_reset(&tty->readable);
	k_poll_signal_raise(&tty->writable, 0);
	tty->nonblock = (flags & O_NONBLOCK) != 0;

	uart_irq_callback_user_data_set(tty->uart, px4_tty_isr, tty);
	uart_irq_rx_enable(tty->uart);

	zvfs_finalize_typed_fd(fd, tty, &px4_tty_vtable, ZVFS_MODE_IFCHR);
	return fd;
}

int __real_open(const char *name, int flags, ...);

int __wrap_open(const char *name, int flags, ...)
{
	struct px4_tty *tty = px4_tty_find(name);

	if (tty != NULL) {
		return px4_tty_open(tty, flags);
	}

	int mode = 0;

	if (flags & O_CREAT) {
		va_list args;

		va_start(args, flags);
		mode = va_arg(args, int);
		va_end(args);
	}

	return __real_open(name, flags, mode);
}

bool px4_tty_exists(const char *path)
{
	return px4_tty_find(path) != NULL;
}

static struct px4_tty *px4_tty_get_fd(int fd)
{
	return zvfs_get_fd_obj(fd, &px4_tty_vtable, ENOTTY);
}

int tcgetattr(int fd, struct termios *termios_p)
{
	struct px4_tty *tty = px4_tty_get_fd(fd);

	if (tty == NULL) {
		return -1;
	}

	*termios_p = tty->termios;
	return 0;
}

int tcflush(int fd, int queue_selector)
{
	struct px4_tty *tty = px4_tty_get_fd(fd);

	if (tty == NULL) {
		return -1;
	}

	if ((queue_selector != TCIFLUSH) && (queue_selector != TCOFLUSH) && (queue_selector != TCIOFLUSH)) {
		errno = EINVAL;
		return -1;
	}

	k_spinlock_key_t key = k_spin_lock(&tty->lock);

	if (queue_selector != TCOFLUSH) {
		ring_buf_reset(tty->rx);
		k_poll_signal_reset(&tty->readable);
	}

	if (queue_selector != TCIFLUSH) {
		ring_buf_reset(tty->tx);
		k_poll_signal_raise(&tty->writable, 0);
	}

	k_spin_unlock(&tty->lock, key);
	return 0;
}

int tcdrain(int fd)
{
	struct px4_tty *tty = px4_tty_get_fd(fd);

	if (tty == NULL) {
		return -1;
	}

	/* give up after a second, when nothing sends the data, e.g. a disconnected USB port */
	for (int i = 0; (i < 1000) && !ring_buf_is_empty(tty->tx); i++) {
		k_msleep(1);
	}

	return 0;
}

int tcsetattr(int fd, int optional_actions, const struct termios *termios_p)
{
	struct px4_tty *tty = px4_tty_get_fd(fd);

	if (tty == NULL) {
		return -1;
	}

	if ((termios_p->c_cflag & CRTSCTS) && !tty->flow_control) {
		/* the UART has no RTS and CTS pins */
		errno = EINVAL;
		return -1;
	}

	if ((optional_actions == TCSADRAIN) || (optional_actions == TCSAFLUSH)) {
		(void)tcdrain(fd);
	}

	struct uart_config config;

	int ret = uart_config_get(tty->uart, &config);

	if (ret == 0) {
		px4_tty_to_uart_config(termios_p, &config);
		ret = uart_configure(tty->uart, &config);

		/* some UARTs, such as the NXP LPUART, are reset with their interrupts when configured */
		if (ret == 0) {
			uart_irq_rx_enable(tty->uart);

			k_spinlock_key_t key = k_spin_lock(&tty->lock);

			if (!ring_buf_is_empty(tty->tx)) {
				uart_irq_tx_enable(tty->uart);
			}

			k_spin_unlock(&tty->lock, key);
		}
	}

	/* the line of emulated and USB UARTs has no settings */
	if ((ret == -ENOSYS) || (ret == -ENOTSUP)) {
		ret = 0;
	}

	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	if (optional_actions == TCSAFLUSH) {
		(void)tcflush(fd, TCIFLUSH);
	}

	tty->termios = *termios_p;
	return 0;
}

static int px4_tty_init(void)
{
	ARRAY_FOR_EACH_PTR(px4_ttys, tty) {
		k_poll_signal_init(&tty->readable);
		k_poll_signal_init(&tty->writable);
	}

	return 0;
}

SYS_INIT(px4_tty_init, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);

#if defined(CONFIG_SHELL)
static int cmd_px4_tty(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (size_t i = 0; i < ARRAY_SIZE(px4_ttys); i++) {
		const struct px4_tty *tty = &px4_ttys[i];

		shell_print(sh, "%s: rx %u, dropped %u, overruns %u, framing %u, noise %u, parity %u", tty->path,
			    tty->rx_bytes, tty->rx_dropped, tty->rx_overruns, tty->rx_framing_errors,
			    tty->rx_noise_errors, tty->rx_parity_errors);
	}

	return 0;
}

SHELL_CMD_REGISTER(px4_tty, NULL, "Receive statistics of the PX4 serial ports", cmd_px4_tty);
#endif

#else

bool px4_tty_exists(const char *path)
{
	ARG_UNUSED(path);
	return false;
}

int tcgetattr(int fd, struct termios *termios_p)
{
	errno = ENOTTY;
	return -1;
}

int tcflush(int fd, int queue_selector)
{
	errno = ENOTTY;
	return -1;
}

int tcdrain(int fd)
{
	errno = ENOTTY;
	return -1;
}

int tcsetattr(int fd, int optional_actions, const struct termios *termios_p)
{
	errno = ENOTTY;
	return -1;
}


#endif

speed_t cfgetispeed(const struct termios *termios_p)
{
	return termios_p->c_ispeed;
}

speed_t cfgetospeed(const struct termios *termios_p)
{
	return termios_p->c_ospeed;
}

int cfsetispeed(struct termios *termios_p, speed_t speed)
{
	termios_p->c_ispeed = speed;
	return 0;
}

int cfsetospeed(struct termios *termios_p, speed_t speed)
{
	termios_p->c_ospeed = speed;
	return 0;
}
