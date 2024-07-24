/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef __APP_H_
#define __APP_H_


extern struct k_sem lte_connected_sem;

extern struct k_sem modem_shutdown_sem;

extern struct k_msgq tx_send_queue;

#endif /* _APP_H */