#include <stdio.h>
#include <string.h>
#include <lwip/api.h>
#include <lwip/opt.h>
#include <lwip/arch.h>
#include <lwip/api.h>
#include <lwip/inet.h>
#include <lwip/sockets.h>
#include <globdefs.h>
#include <tcpip.h>
#include <main.h>
#include <FreeRTOS.h>
#include "udpecho.h"


#define 	TASK_NAME		"eth task"
#define 	ETH_TASK_PRIORITY		(tskIDLE_PRIORITY + 1)

static void eth_task(void *p);

int eth_create_task(void)
{
    xTaskHandle task = NULL;

    xTaskCreate(eth_task, TASK_NAME, configMINIMAL_STACK_SIZE, NULL, ETH_TASK_PRIORITY, &task);
    if (task == NULL) {
	printf("ERROR: Create %s\r\n", TASK_NAME);
	vTaskDelete(task);
	configASSERT(task);
    }
    printf("SUCCESS: Create %s\r\n", TASK_NAME);

    return 0;
}


/**
 * Задача
 */
static void eth_task(void *p)
{
    int sockfd;			/* Сокет */
    struct sockaddr_in local;
    u32 addr_len = sizeof(struct sockaddr);
    fd_set readfds, rfds;
    int i, maxfd = 0, n, k, z = 0;
    static char buf[1024];
    struct timeval t0;
    long opt;


    FD_ZERO(&readfds);
    FD_ZERO(&rfds);

    /* Создаем сокет UDP с этими настойками */
    local.sin_port = htons(7);
    local.sin_family = PF_INET;
    local.sin_addr.s_addr = INADDR_ANY;

    /* Создадим сокет */
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd < 0) {
	printf("ERROR: create socket\r\n");
    }
    printf("SUCCESS: create socket\r\n");

    /* Неблокирующий режим */
    opt = 1;
    ioctlsocket(sockfd, FIONBIO, &opt);

    /* Находим максимум для селекта */
    if (sockfd > maxfd) {
	maxfd = sockfd;
    }

    /* bind to port at own address */
    if (bind(sockfd, (struct sockaddr *) &local, addr_len) < 0) {
	log_printf("ERROR: bind\r\n");
    }
    log_printf("SUCCESS: bind\r\n");

    /* Делаем набор дескрипторов для отслеживания */
    FD_SET(sockfd, &readfds);


    /*  Ждем входящих сообщений */
    while (1) {
	/**
         * Кто то готов - разбираем  таймаут ставим в ноль, иначе функция select не работает! 
         * В это случае сразу будет выходить если нет готового дескриптора
         * если передать NULL - то работает неправильно!!! 
         * не стал исправлять lwip_select()!
         * Время не ставить!!!
         */
	t0.tv_sec = 0;
	t0.tv_usec = 1000;
	memcpy(&rfds, &readfds, sizeof(rfds));
	n = -1;
	if (select(maxfd + 1, &rfds, NULL, NULL, &t0) > 0) {
	    if (FD_ISSET(sockfd, &rfds)) {
		n = 1;
	    }
	}

	/* Принимаем данные и кладем в кольцевой буфер вверх */
	if (n >= 0) {
	    memset(buf, 0, sizeof(buf));
	    k = recvfrom(sockfd, buf, 1024, 0, (struct sockaddr *) &local, (u32_t *) & addr_len);

	    if (k > 0) {
		buf[k] = 0;
		if (sendto(sockfd, buf, k, 0, (struct sockaddr *) &local, addr_len) < 0) {
		    printf("Error sendto\r\n");
		}
	    }
	}
    }
}
