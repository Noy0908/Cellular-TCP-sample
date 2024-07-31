/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef __APP_H_
#define __APP_H_


typedef struct {
	uint8_t *data;
	uint16_t length;
} socket_data_t;


extern const k_tid_t tcp_thread; 

extern struct k_sem lte_connected_sem;

extern struct k_sem modem_shutdown_sem;

extern struct k_msgq tx_send_queue;

extern struct k_msgq rx_event_queue;

extern bool reconnect;


extern int uart_tx_write(const uint8_t *data, size_t len);


extern int uart_handler_init(void);



#endif /* _APP_H */