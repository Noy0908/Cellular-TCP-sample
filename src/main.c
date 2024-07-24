/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_at.h>


#define FW_VERSION			"1.1.0"

#define UDP_IP_HEADER_SIZE 	28

#define TX_QUEUE_COUNT		20

static int client_fd;
static struct sockaddr_storage host_addr;
static struct k_work_delayable socket_transmission_work;
static int data_upload_iterations = CONFIG_TCP_DATA_UPLOAD_ITERATIONS;


K_SEM_DEFINE(lte_connected_sem, 0, 1);
K_SEM_DEFINE(modem_shutdown_sem, 0, 1);

K_MSGQ_DEFINE(tx_send_queue, 4, TX_QUEUE_COUNT, 4);

static void socket_transmission_work_fn(struct k_work *work)
{
	int err;
	char *send_buffer = NULL;
	
	send_buffer = k_malloc(256);
	if (send_buffer != NULL) 
	{
		memset(send_buffer,data_upload_iterations,256);		//just test
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


#if 0

#if defined(CONFIG_TCP_RAI_LAST)
	/* Let the modem know that this is the last packet for now and we do not
	 * wait for a response.
	 */
	int rai = RAI_LAST;

	err = setsockopt(client_fd, SOL_SOCKET, SO_RAI, &rai, sizeof(rai));
	if (err) {
		printk("Failed to set socket option, error: %d\n", errno);
	}
#endif

#if defined(CONFIG_TCP_RAI_ONGOING)
	/* Let the modem know that we expect to keep the network up longer.
	 */
	int rai = RAI_ONGOING;

	err = setsockopt(client_fd, SOL_SOCKET, SO_RAI, &rai, sizeof(rai));
	if (err) {
		printk("Failed to set socket option, error: %d\n", errno);
	}
#endif

	// err = send(client_fd, buffer, sizeof(buffer), 0);
	// if (err < 0) {
	// 	printk("Failed to transmit UDP packet, error: %d\n", errno);
	// }

#if defined(CONFIG_UDP_RAI_NO_DATA)
	/* Let the modem know that there will be no upcoming data transmission anymore.
	 */
	int rai = RAI_NO_DATA;

	err = setsockopt(client_fd, SOL_SOCKET, SO_RAI, &rai, sizeof(rai));
	if (err) {
		printk("Failed to set socket option, error: %d\n", errno);
	}
#endif

#endif

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

// static int socket_connect(void)
// {
// 	int err;
// 	struct sockaddr_in *server4 = ((struct sockaddr_in *)&host_addr);

// 	server4->sin_family = AF_INET;
// 	server4->sin_port = htons(CONFIG_UDP_SERVER_PORT);

// 	inet_pton(AF_INET, CONFIG_UDP_SERVER_ADDRESS_STATIC, &server4->sin_addr);

// 	client_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
// 	if (client_fd < 0) {
// 		printk("Failed to create UDP socket, error: %d\n", errno);
// 		err = -errno;
// 		return err;
// 	}

// 	err = connect(client_fd, (struct sockaddr *)&host_addr, sizeof(struct sockaddr_in));
// 	if (err < 0) {
// 		printk("Failed to connect socket, error: %d\n", errno);
// 		close(client_fd);
// 		return err;
// 	}

// 	return 0;
// }


int main(void)
{
	int err;

	printk("\nTCP sample has started, version is %s.\n\n",FW_VERSION);

	work_init();

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

	// k_sem_take(&lte_connected_sem, K_FOREVER);

	// err = socket_connect();
	// if (err) {
	// 	return -1;
	// }

	// start_tcp_task();

	k_work_schedule(&socket_transmission_work, K_NO_WAIT);

	k_sem_take(&modem_shutdown_sem, K_FOREVER);

	err = nrf_modem_lib_shutdown();
	if (err) {
		return -1;
	}

	return 0;
}
