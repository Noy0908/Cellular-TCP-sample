/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/drivers/uart.h>
#include <hal/nrf_uarte.h>
#include <hal/nrf_gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_handler, CONFIG_TCP_LOG_LEVEL);
#include "slip/slip.h"
#include "app.h"


#define UART_RX_TIMEOUT_US					50000
#define UART_ERROR_DELAY_MS					500
#define UART_RX_EVENT_COUNT_FOR_BUF 		20

#define UART_SLIP_MTU						512

#define UART_SLAB_BLOCK_SIZE 				sizeof(struct rx_buf_t)
#define UART_SLAB_BLOCK_COUNT 				3


const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart2));
uint32_t uart_baudrate;

static struct k_work_delayable rx_process_work;

struct rx_buf_t {
	atomic_t ref_counter;
	size_t len;
	uint8_t buf[512];
};

struct rx_event_t {
	uint8_t *buf;
	size_t len;
};

enum uart_recovery_state {
	RECOVERY_IDLE,
	RECOVERY_ONGOING,
	RECOVERY_DISABLED
};

static atomic_t recovery_state;

static uint8_t slip_buffer[UART_SLIP_MTU];
static slip_t m_slip = {
	.state = SLIP_STATE_DECODING, 
	.p_buffer = slip_buffer,
	.current_index = 0,
	.buffer_len = sizeof(slip_buffer)};

K_MEM_SLAB_DEFINE(rx_slab, UART_SLAB_BLOCK_SIZE, UART_SLAB_BLOCK_COUNT, 4);

K_SEM_DEFINE(tx_done_sem, 0, 1);

K_MSGQ_DEFINE(rx_event_queue, sizeof(struct rx_event_t), 8, 4);

RING_BUF_DECLARE(tx_buf, 256);

/* Protects the tx_buf from multiple writes. */
K_MUTEX_DEFINE(mutex_tx_put); 

static inline struct rx_buf_t *block_start_get(uint8_t *buf)
{
	size_t block_num;

	/* blocks are fixed size units from a continuous memory slab: */
	/* round down to the closest unit size to find beginning of block. */

	block_num = (((size_t)buf - (size_t)rx_slab.buffer) / UART_SLAB_BLOCK_SIZE);

	return (struct rx_buf_t *) &rx_slab.buffer[block_num * UART_SLAB_BLOCK_SIZE];
}

static struct rx_buf_t *rx_buf_alloc(void)
{
	struct rx_buf_t *buf;
	int err;

	/* Async UART driver returns pointers to received data as */
	/* offsets from beginning of RX buffer block. */
	/* This code uses a reference counter to keep track of the number of */
	/* references within a single RX buffer block */

	err = k_mem_slab_alloc(&rx_slab, (void **) &buf, K_NO_WAIT);
	if (err) {
		return NULL;
	}

	atomic_set(&buf->ref_counter, 1);

	return buf;
}

static void rx_buf_ref(void *buf)
{
	atomic_inc(&(block_start_get(buf)->ref_counter));
}

static void rx_buf_unref(void *buf)
{
	struct rx_buf_t *uart_buf = block_start_get(buf);
	atomic_t ref_counter = atomic_dec(&uart_buf->ref_counter);

	/* ref_counter is the uart_buf->ref_counter value prior to decrement */
	if (ref_counter == 1) {
		k_mem_slab_free(&rx_slab, (void *)uart_buf);
	}
}

static int rx_enable(void)
{
	struct rx_buf_t *buf;
	int ret;

	buf = rx_buf_alloc();
	if (!buf) {
		LOG_ERR("UART RX failed to allocate buffer");
		return -ENOMEM;
	}

	ret = uart_rx_enable(uart_dev, buf->buf, sizeof(buf->buf), UART_RX_TIMEOUT_US);
	if (ret) {
		LOG_ERR("UART RX enable failed: %d", ret);
		return ret;
	}

	return 0;
}

static int rx_disable(void)
{
	int err;

	/* Wait for possible rx_enable to complete. */
	if (atomic_set(&recovery_state, RECOVERY_DISABLED) == RECOVERY_ONGOING) {
		k_sleep(K_MSEC(10));
	}

	err = uart_rx_disable(uart_dev);
	if (err) {
		LOG_ERR("UART RX disable failed: %d", err);
		atomic_set(&recovery_state, RECOVERY_IDLE);
		return err;
	}

	return 0;
}

static void rx_recovery(void)
{
	int err;

	if (atomic_get(&recovery_state) != RECOVERY_ONGOING) {
		return;
	}

	err = rx_enable();
	if (err) {
		k_work_schedule(&rx_process_work, K_MSEC(20));
		return;
	}

	atomic_cas(&recovery_state, RECOVERY_ONGOING, RECOVERY_IDLE);
}


static void on_packet_received(uint8_t *buffer, uint16_t length)
{
	int err;
	socket_data_t send_buffer = {0};
	
	send_buffer.data = k_calloc(length+1, sizeof(uint8_t));
	if (send_buffer.data != NULL) 
	{
		memcpy(send_buffer.data, buffer, length);		
		send_buffer.length = length;

		if(0 == k_msgq_num_free_get(&tx_send_queue))
		{
			/* message queue is full ,we have to delete the oldest data to reserve room for the new data */
			socket_data_t data_buffer = {0};
			k_msgq_get(&tx_send_queue, &data_buffer, K_NO_WAIT);
			LOG_DBG("Droped data:[%d]:%s\n",data_buffer.length, data_buffer.data);
			k_free(data_buffer.data);
		}

		/** send the uart data to tcp server*/
		err = k_msgq_put(&tx_send_queue, &send_buffer, K_NO_WAIT);
		if (err) {
			LOG_ERR("Message sent error: %d", err);
			k_free(send_buffer.data);
		}
	} 
	else 
	{
		LOG_ERR("Memory not allocated!\n");
	}
}


/** Before uart send data, the data should be enpacket by SLIP protocol */
static void uart_send_data(uint8_t *buffer, uint16_t length)
{
	static uint8_t send_buffer[UART_SLIP_MTU] = {0};
	uint32_t send_length = 0;

	slip_encode(send_buffer, buffer, length, &send_length);
	uart_tx_write(send_buffer, send_length);
}


static void rx_process(struct k_work *work)
{
	uint16_t i = 0;
	struct rx_event_t rx_event;
	int ret_code = 0;

	while (k_msgq_get(&rx_event_queue, &rx_event, K_NO_WAIT) == 0) 
	{
		// LOG_INF("Uart received:[%d]:%s\n",rx_event.len, rx_event.buf);
		for(i=0; i<rx_event.len; i++)
		{
			ret_code = slip_decode_add_byte(&m_slip, rx_event.buf[i]);
			switch (ret_code)
			{
			case NRF_SUCCESS:
				/** decode uart data success, now put it to message queue */
				on_packet_received(m_slip.p_buffer, m_slip.current_index);
				uart_send_data(m_slip.p_buffer, m_slip.current_index);		//uart send back the received data with lwip protocal, just for test
				
				memset(m_slip.p_buffer, 0, m_slip.buffer_len);
				m_slip.current_index = 0;
        		m_slip.state = SLIP_STATE_DECODING;
				break;
			// fall through
			case ERROR_NO_MEM:
				m_slip.current_index = 0;
				m_slip.state = SLIP_STATE_DECODING;
				break;
			default:
				break;
			}
		}

		rx_buf_unref(rx_event.buf);
	}

	rx_recovery();
}

static int tx_start(void)
{
	uint8_t *buf;
	size_t len;
	int err;
	enum pm_device_state state = PM_DEVICE_STATE_OFF;

	pm_device_state_get(uart_dev, &state);
	if (state != PM_DEVICE_STATE_ACTIVE) {
		return 1;
	}

	len = ring_buf_get_claim(&tx_buf, &buf, ring_buf_capacity_get(&tx_buf));
	err = uart_tx(uart_dev, buf, len, SYS_FOREVER_US);
	if (err) {
		LOG_ERR("UART TX error: %d", err);
		ring_buf_get_finish(&tx_buf, 0);
		return err;
	}
	// else
	// {
	// 	LOG_ERR("UART TX [%d] bytes Succeed!", len);
	// }

	return 0;
}

static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	struct rx_buf_t *buf;
	struct rx_event_t rx_event;
	int err;

	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case UART_TX_DONE:
	case UART_TX_ABORTED:
		err = ring_buf_get_finish(&tx_buf, evt->data.tx.len);
		if (err) {
			LOG_ERR("UART_TX_%s failure: %d",
				(evt->type == UART_TX_DONE) ? "DONE" : "ABORTED", err);
		}
		if (ring_buf_is_empty(&tx_buf) == false) {
			tx_start();
		} 
		
		k_sem_give(&tx_done_sem);
		break;
	case UART_RX_RDY:
		rx_buf_ref(evt->data.rx.buf);		
		rx_event.buf = &evt->data.rx.buf[evt->data.rx.offset];
		rx_event.len = evt->data.rx.len;
		err = k_msgq_put(&rx_event_queue, &rx_event, K_NO_WAIT);
		if (err) {
			LOG_ERR("UART_RX_RDY failure: %d, dropped: %d", err, evt->data.rx.len);
			rx_buf_unref(evt->data.rx.buf);
			break;
		}
		(void)k_work_submit((struct k_work *)&rx_process_work);
		break;
	case UART_RX_BUF_REQUEST:
		buf = rx_buf_alloc();
		if (!buf) {
			LOG_WRN("Disabling UART RX: No free buffers.");
			break;
		}
		err = uart_rx_buf_rsp(uart_dev, buf->buf, sizeof(buf->buf));
		if (err) {
			LOG_WRN("Disabling UART RX: %d", err);
			rx_buf_unref(buf);
		}
		break;
	case UART_RX_BUF_RELEASED:
		if (evt->data.rx_buf.buf) {
			rx_buf_unref(evt->data.rx_buf.buf);
		}
		break;
	case UART_RX_DISABLED:
		if (atomic_cas(&recovery_state, RECOVERY_IDLE, RECOVERY_ONGOING)) {
			k_work_submit((struct k_work *)&rx_process_work);
		}
		break;
	default:
		break;
	}
}

/* Write the data to tx_buffer and trigger sending. */
int uart_tx_write(const uint8_t *data, size_t len)
{
	size_t ret;
	size_t sent = 0;
	int err;

	k_mutex_lock(&mutex_tx_put, K_FOREVER);
	while (sent < len) {
		ret = ring_buf_put(&tx_buf, data + sent, len - sent);
		if (ret) {
			sent += ret;
		} else {
			/* Buffer full, block and start TX. */
			k_sem_take(&tx_done_sem, K_FOREVER);
			err = tx_start();
			if (err) {
				LOG_ERR("TX buf overflow, %d dropped. Unable to send: %d",
					len - sent,
					err);
				k_sem_give(&tx_done_sem);
				k_mutex_unlock(&mutex_tx_put);
				return err;
			}
		}
	}
	k_mutex_unlock(&mutex_tx_put);

	if (k_sem_take(&tx_done_sem, K_NO_WAIT) == 0) {
		err = tx_start();
		if (err == 1) {
			k_sem_give(&tx_done_sem);
			return 0;
		} else if (err) {
			LOG_ERR("TX start failed: %d", err);
			k_sem_give(&tx_done_sem);
			return err;
		}
	} else {
		/* TX already in progress. */
	}

	return 0;
}

int uart_handler_init(void)
{
	int err;
	uint32_t start_time;
	struct uart_config cfg;

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	err = uart_config_get(uart_dev, &cfg);
	if (err) {
		LOG_ERR("uart_config_get: %d", err);
		return err;
	}

	uart_baudrate = cfg.baudrate;
	LOG_INF("UART baud: %d d/p/s-bits: %d/%d/%d HWFC: %d",
		cfg.baudrate, cfg.data_bits, cfg.parity,
		cfg.stop_bits, cfg.flow_ctrl);

	/* Wait for the UART line to become valid */
	start_time = k_uptime_get_32();
	do {
		err = uart_err_check(uart_dev);
		if (err) {
			uint32_t now = k_uptime_get_32();

			if (now - start_time > UART_ERROR_DELAY_MS) {
				LOG_ERR("UART check failed: %d", err);
				return -EIO;
			}
			k_sleep(K_MSEC(10));
		}
	} while (err);
	err = uart_callback_set(uart_dev, uart_callback, NULL);
	if (err) {
		LOG_ERR("Cannot set callback: %d", err);
		return -EFAULT;
	}
	(void)atomic_set(&recovery_state, RECOVERY_IDLE);
	err = rx_enable();
	if (err) {
		return -EFAULT;
	}

	k_work_init_delayable(&rx_process_work, rx_process);

	k_sem_give(&tx_done_sem);

	return 0;
}

