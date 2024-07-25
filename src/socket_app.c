/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <stdlib.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/net/socket.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tcp_sample, CONFIG_TCP_LOG_LEVEL);

#include "app.h"

#ifdef CONFIG_NET_IPV6
#define USE_IPV6
#endif

#define TCP_THREAD_STACK_SIZE               4096
#define TCP_THREAD_PRIORITY 				5

/* For Zephyr, keep max number of fd's in sync with max poll() capacity */
#ifdef CONFIG_NET_SOCKETS_POLL_MAX
#define NUM_FDS 	CONFIG_NET_SOCKETS_POLL_MAX
#else
#define NUM_FDS 	5
#endif

#define BIND_PORT 	4242

int event_fd = 0;

#if 0
void send_message_queue(void)
{
	
		LOG_INF("TCP wait for poll event at %lld.\n",k_ticks_to_ms_floor64(k_uptime_ticks()));

		socket_data_t data_buffer = {0};
		// if(k_msgq_get(&tx_send_queue, &data_buffer, K_SECONDS(5)) == 0)
		if (k_msgq_peek(&tx_send_queue, &data_buffer) == 0) 
		{
			LOG_INF("%d-%d-%d\n",data_buffer.data[1],data_buffer.data[2],data_buffer.data[3]);
			// k_free(data_buffer.data);
			// ret = do_tcp_send(client_socket, data_buffer.data, data_buffer.length);
			// {
			// 	if(ret == data_buffer.length)	
			// 	{
			// 		/* send successfully, delete the queue message and free memory */
					k_msgq_get(&tx_send_queue, &data_buffer, K_NO_WAIT);
					k_free(data_buffer.data);
			// 	}
			// }
		}
		else
		{
			k_sleep(K_SECONDS(5));
		}

		// char *data_buffer = NULL;

		// if(k_msgq_get(&tx_send_queue, &data_buffer, K_SECONDS(5)) == 0)
		// {
		// 	LOG_INF("%d-%d-%d\n",data_buffer[1],data_buffer[2],data_buffer[3]);
		// 	k_free(data_buffer);
		// }
	
}
#endif



static int do_tcp_send(int sock, const uint8_t *data, int datalen)
{
	int ret = 0;
	uint32_t offset = 0;

	while (offset < datalen) {
		ret = send(sock, data + offset, datalen - offset, 0);
		if (ret < 0) {
			LOG_ERR("send() failed: %d, sent: %d", -errno, offset);
			ret = -errno;
			break;
		} else {
			offset += ret;
		}
	}

	return ret;
}


/* tcp client task */
static void tcp_thread_fn(void)
{
	int ret = 0;
	struct pollfd fds[2];
	int client_socket = 0;
	uint8_t rec_buf[512] = {0};  /* For socket receive data. */
	
	k_sem_take(&lte_connected_sem, K_FOREVER);
	printf("LTE connected successfully, now we satrt tcp task!!!\n");

#if 1
retry:

#if !defined(USE_IPV6)
	struct sockaddr_in server_addr = {0};

	client_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (client_socket < 0) {
		printf("error: socket(AF_INET6): %d\n", errno);
		goto error_exit;
	}
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(CONFIG_TCP_SERVER_PORT);
	inet_pton(AF_INET, CONFIG_TCP_SERVER_ADDRESS_STATIC,&server_addr.sin_addr);

	ret = connect(client_socket, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in));
#else
	struct sockaddr_in6 server_addr = {0};

	client_socket = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
	if (client_socket < 0) {
		printf("error: socket(AF_INET6): %d\n", errno);
		goto error_exit;
	}
	server_addr.sin6_family = AF_INET6;
	server_addr.sin6_port = htons(CONFIG_TCP_SERVER_PORT);
	inet_pton(AF_INET6, CONFIG_TCP_SERVER_ADDRESS_STATIC,&server_addr.sin6_addr);

	// ret = connect(client_socket, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in6));
#endif
	if (ret) {
		LOG_ERR("connect() failed: %d", -errno);
		goto error_exit;
	}

	fds[0].fd = client_socket;
	fds[0].events = POLLIN;
#endif

	event_fd = eventfd(0, 0);
	fds[1].fd = event_fd;
	fds[1].events = POLLIN;

	while (1) {
		ret = poll(fds, ARRAY_SIZE(fds), MSEC_PER_SEC * CONFIG_TCP_POLL_TIME);
		if (ret < 0) {
			LOG_WRN("poll() error: %d", ret);
			break;
		}
		if (ret == 0) {
			/* timeout */
			LOG_INF("TCP wait for poll event at %lld.\n",k_ticks_to_ms_floor64(k_uptime_ticks()));
			continue;
		}

		LOG_DBG("[%d]sock events 0x%08x:%08x", ret,fds[0].revents,fds[1].revents);
		if ((fds[0].revents & POLLIN) != 0) {
			/* receive data from server */
			ret = recv(fds[0].fd, (void *)rec_buf, sizeof(rec_buf), MSG_DONTWAIT);
			if (ret < 0 && errno != EAGAIN) {
				LOG_WRN("recv() error: %d", -errno);
			} else if (ret > 0) {
				// LOG_DBG(rec_buf, ret);
				//receive_data_handle(rec_buf, ret);
			}
		}
		if ((fds[0].revents & POLLERR) != 0) {
			LOG_WRN("SOCK (%d): POLLERR", fds[0].fd);
			break;
		}
		if ((fds[0].revents & POLLNVAL) != 0) {
			LOG_WRN("SOCK (%d): POLLNVAL", fds[0].fd);
			break;
		}
		if ((fds[0].revents & POLLHUP) != 0) {
			/* Lose LTE connection / remote end close */
			LOG_WRN("SOCK (%d): POLLHUP", fds[0].fd);
			// k_sem_take(&lte_connected_sem, K_FOREVER);
			break;
		}

		/***************** Events to send message queue. *****************************/
		if ((fds[1].revents & POLLIN) != 0) {
			int64_t value;
			eventfd_read(fds[1].fd, &value);		//delete event

			socket_data_t data_buffer = {0};
			if (k_msgq_peek(&tx_send_queue, &data_buffer) == 0) 
			{
				LOG_INF("%d-%d-%d\n",data_buffer.data[1],data_buffer.data[2],data_buffer.data[3]);
				// ret = do_tcp_send(client_socket, data_buffer.data, data_buffer.length);
				{
					// if(ret == data_buffer.length)	
					{
						/* send successfully, delete the queue message and free memory */
						k_msgq_get(&tx_send_queue, &data_buffer, K_NO_WAIT);
						k_free(data_buffer.data);
					}
				}
			}
		}
	}

error_exit:
	if(client_socket)
	{
		close(client_socket);
	}
			
	goto retry;
}




K_THREAD_DEFINE(tcp_thread, TCP_THREAD_STACK_SIZE,
		tcp_thread_fn, NULL, NULL, NULL,
		TCP_THREAD_PRIORITY, 0, 0);