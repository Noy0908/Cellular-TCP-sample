/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/drivers/uart.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_at.h>

#include "app.h"


#define FW_VERSION			"1.4.7"


#define TX_QUEUE_COUNT		20


K_SEM_DEFINE(lte_connected_sem, 0, 1);
K_SEM_DEFINE(modem_shutdown_sem, 0, 1);


/** Below code can be work as a timer */
#if 0
K_MSGQ_DEFINE(tx_send_queue, sizeof(socket_data_t), TX_QUEUE_COUNT, 4);
static struct k_work_delayable socket_transmission_work;
static int data_upload_iterations = CONFIG_TCP_DATA_UPLOAD_ITERATIONS;

static void socket_transmission_work_fn(struct k_work *work)
{
	int err;
	socket_data_t send_buffer = {0};
	// char *send_buffer = NULL;
	
	send_buffer.data = k_malloc(256);
	if (send_buffer.data != NULL) 
	{
		memset(send_buffer.data,data_upload_iterations,256);		//just test
		send_buffer.length = 256;

		if(0 == k_msgq_num_free_get(&tx_send_queue))
		{
			/* message queue is full ,we have to delete the oldest data to reserve room for the new data */
			socket_data_t data_buffer = {0};
			k_msgq_get(&tx_send_queue, &data_buffer, K_NO_WAIT);
			printf("Droped data:[%d]:%d-%d-%d\n",data_buffer.length, data_buffer.data[1],data_buffer.data[2],data_buffer.data[3]);
			k_free(data_buffer.data);
		}

		/** send the uart data to tcp server*/
		err = k_msgq_put(&tx_send_queue, &send_buffer, K_NO_WAIT);
		if (err) {
			printf("Message sent error: %d", err);
		}
	} 
	else 
	{
		printf("Memory not allocated!\n");
	}

	printk("socket_transmission_work_fn[%d] been triggeded at %lld,\r\n ",data_upload_iterations,k_ticks_to_ms_floor64(k_uptime_ticks()));

	/* Transmit a limited number of times and then shutdown. */
	if (data_upload_iterations > 0) {
		data_upload_iterations--;
	} else if (data_upload_iterations == 0) {
		k_sem_give(&modem_shutdown_sem);
		/* No need to schedule work if we're shutting down. */
		return;
	}

	/* Schedule work if we're either transmitting indefinitely or
	 * there are more iterations left.
	 */
	k_work_schedule(&socket_transmission_work,
			K_SECONDS(CONFIG_TCP_DATA_UPLOAD_FREQUENCY_SECONDS));
}


static void work_init(void)
{
	k_work_init_delayable(&socket_transmission_work, socket_transmission_work_fn);
}

#endif

static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
		    (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
			break;
		}

		printk("Network registration status: %s\n",
		       evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ?
		       "Connected - home" : "Connected - roaming");
		k_sem_give(&lte_connected_sem);

		// k_work_schedule(&socket_transmission_work, K_SECONDS(5));
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		printk("PSM parameter update: TAU: %d s, Active time: %d s\n",
		       evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE:
		printk("eDRX parameter update: eDRX: %.2f s, PTW: %.2f s\n",
		       (double)evt->edrx_cfg.edrx, (double)evt->edrx_cfg.ptw);
		break;
	case LTE_LC_EVT_RRC_UPDATE:
		printk("RRC mode: %s\n",
		       evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle\n");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		printk("LTE cell changed: Cell ID: %d, Tracking area: %d\n",
		       evt->cell.id, evt->cell.tac);
		break;
	default:
		break;
	}
}


static void modem_connect(void)
{
	int ret = 0;

	do {
		printk("\nConnecting to network.This may take several minutes.\n");

		ret = lte_lc_connect();
		if (ret < 0) {
			printk("Failed to establish LTE connection (%d).", ret);
			printk("Will retry in a minute.");
			lte_lc_offline();
			k_sleep(K_SECONDS(60));
		} else {
			enum lte_lc_lte_mode mode;

			lte_lc_lte_mode_get(&mode);
			if (mode == LTE_LC_LTE_MODE_NBIOT) {
				printk("Connected to NB-IoT network.\n");
			} else if (mode == LTE_LC_LTE_MODE_LTEM) {
				printk("Connected to LTE network.\n");
			} else  {
				printk("Connected to unknown network.\n");
			}
			k_sem_give(&lte_connected_sem);
		}
	} while (ret < 0);
}


int main(void)
{
	int err;

	printk("\nTCP sample has started, version is %s.\n\n",FW_VERSION);

	// work_init();

	err = nrf_modem_lib_init();
	if (err) {
		printk("Failed to initialize modem library, error: %d\n", err);
		return -1;
	}

	err = lte_lc_connect_async(lte_handler);
	if (err) {
		printk("Failed to connect to LTE network, error: %d\n", err);
		return -1;
	}

	uart_handler_init();
	
	while (true) 
	{
		/* Set network state to start for blocking LTE */
		if (reconnect) {
			reconnect = false;

			printk("LTE connection error. The sample will try to re-establish network connection.\n");

			/* Try to reconnect to the network. */
			err = lte_lc_offline();
			if (err < 0) {
				printk("Failed to put LTE link in offline state (%d)\n", err);
			}
			modem_connect();
		}

		/** This part code may not excute, just for low power use api */
		if( 0 == k_sem_take(&modem_shutdown_sem, K_SECONDS(2)))
		{
			/** we should suspend the tcp thread */
			// k_thread_suspend(tcp_thread);

			err = nrf_modem_lib_shutdown();
			if (err) {
				return -1;
			}	
		}
	}	

	return 0;
}
