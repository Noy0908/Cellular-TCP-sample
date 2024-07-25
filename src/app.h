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


extern struct k_sem lte_connected_sem;

extern struct k_sem modem_shutdown_sem;

extern struct k_msgq tx_send_queue;

extern int event_fd;

extern bool reconnect;

#endif /* _APP_H */