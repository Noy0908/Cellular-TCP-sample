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

// #ifdef CONFIG_NET_IPV6
// #define USE_IPV6
// #endif

#define TCP_THREAD_STACK_SIZE               4096
#define TCP_THREAD_PRIORITY 				5

#define TX_QUEUE_COUNT						20


K_MSGQ_DEFINE(tx_send_queue, sizeof(socket_data_t), TX_QUEUE_COUNT, 4);

bool reconnect = false;


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
			// LOG_INF("Socket send data length = %d\n", ret);
			offset += ret;
		}
	}

	return ret;
}


static void receive_data_handle(uint8_t *data, uint16_t length)
{
	LOG_INF("Received %d bytes data from tcp server, data_segment:%d", length, *(data+0));
}


/* tcp client task */
static void tcp_thread_fn(void)
{
	int ret = 0;
	struct pollfd fds[2];
	int client_socket = 0;
	uint8_t rec_buf[512] = {0};  /* For socket receive data. */
	
	k_sem_take(&lte_connected_sem, K_FOREVER);

retry:
	LOG_WRN("LTE connected successfully, now we start tcp task!!!\n");

#if !defined(USE_IPV6)
	struct sockaddr_in server_addr = {0};

	client_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (client_socket < 0) {
		printf("error: socket(AF_INET): %d\n", errno);
		goto error_exit;
	}
	struct timeval socket_timeout = {.tv_sec = 3};

	ret = setsockopt(client_socket, SOL_SOCKET, SO_SNDTIMEO, &socket_timeout, sizeof(socket_timeout));
	if (ret) {
		LOG_ERR("setsockopt(%d) error: %d", SO_SNDTIMEO, -errno);
		ret = -errno;
		goto error_exit;
	}

	
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(CONFIG_TCP_SERVER_PORT);
	inet_pton(AF_INET, CONFIG_TCP_SERVER_ADDRESS_STATIC,&server_addr.sin_addr);

	ret = connect(client_socket, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in));
	if (ret) {
		LOG_ERR("AF_INET connect() failed: %d", -errno);
		goto error_exit;
	}
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

	ret = connect(client_socket, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in6));
	if (ret) {
		LOG_ERR("AF_INET6 connect() failed: %d", -errno);
		goto error_exit;
	}
#endif
	
	fds[0].fd = client_socket;
	fds[0].events = POLLIN;

	while (1) {
		// ret = poll(fds, ARRAY_SIZE(fds), MSEC_PER_SEC * CONFIG_TCP_POLL_TIME);
		ret = poll(fds, ARRAY_SIZE(fds), 200);
		if (ret < 0) {
			LOG_WRN("poll() error: %d", ret);
			break;
		}
		else if (ret == 0) {
			/* timeout */
			// LOG_INF("TCP wait for poll event timeout at %lld.\n",k_ticks_to_ms_floor64(k_uptime_ticks()));
		}
		else{
			// LOG_DBG("[%d]sock events 0x%08x at %lld.\n", ret,fds[0].revents,k_ticks_to_ms_floor64(k_uptime_ticks()));
			if ((fds[0].revents & POLLIN) != 0) {
				/* receive data from server */
				ret = recv(fds[0].fd, (void *)rec_buf, sizeof(rec_buf), MSG_DONTWAIT);
				if (ret < 0 && errno != EAGAIN) {
					LOG_WRN("recv() error: %d", -errno);
				} else if (ret > 0) {
					/** handle the received data from tcp server */
					receive_data_handle(rec_buf, ret);
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
				reconnect = true;
				k_sem_take(&lte_connected_sem, K_FOREVER);
				break;
			}
		}

		/***************** Events to send message queue. *****************************/
		{
			static uint8_t resend_count = 0;
			socket_data_t send_packet = {0};
			if (k_msgq_peek(&tx_send_queue, &send_packet) == 0)  
			{
				// LOG_INF("[%d]:%s\n",send_packet.length, send_packet.data);
				ret = do_tcp_send(client_socket, send_packet.data, send_packet.length);
				if(ret == send_packet.length)
				{
					/* send successfully, delete the queue message and free memory */
					k_msgq_get(&tx_send_queue, &send_packet, K_NO_WAIT);
					k_free(send_packet.data);
					resend_count = 0;
					LOG_DBG("Socket send [%d] successfully , the data is: \"%s\"\n",ret, send_packet.data);
				}
				else if(ret <= 0)
				{
					if(resend_count++ >= 3)
					{
						LOG_WRN("Socket has something wrong, we'd better reconnect it.\n");
						break;
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
	k_sleep(K_SECONDS(3));	
	goto retry;
}




K_THREAD_DEFINE(tcp_thread, TCP_THREAD_STACK_SIZE,
		tcp_thread_fn, NULL, NULL, NULL,
		TCP_THREAD_PRIORITY, 0, 0);