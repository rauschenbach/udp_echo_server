#include <stats.h>
#include <snmp.h>
#include <string.h>
#include "etharp.h"
#include "ppp_oe.h"
#include "opt.h"
#include "def.h"
#include "mem.h"
#include "pbuf.h"
#include "main.h"
#include "ethernetif.h"
#include "enc28j60.h"

#define 	ETH_PHY_ADDR			0x1C
#define 	ETH_MDIO_CTRL_DIV		1
#define 	ETH_MODE			ETH_PHY_CONTROL_MODE_AUTO	/*ETH_PHY_CONTROL_MODE_100M_FD */


/* Define those to better describe your network interface. */
#define 	IFNAME0 			'e'
#define 	IFNAME1 			'1'


/* MAC ADDRESS*/
#define       FULL_MAC                        "Казань"

#define MAC_ADDR0   FULL_MAC[0]
#define MAC_ADDR1   FULL_MAC[1]
#define MAC_ADDR2   FULL_MAC[2]
#define MAC_ADDR3   FULL_MAC[3]
#define MAC_ADDR4   FULL_MAC[4]
#define MAC_ADDR5   FULL_MAC[5]



static struct netif *s_pxNetIf = NULL;
static xSemaphoreHandle s_xSemaphore  = NULL;	/* Семафор для ожидающей задачи приема */
static u16 rx_reg  = 0;	/* Биты регистров по чтению */

#pragma pack(4)
static u8 EthFrame[ETH_FRAME_SIZE] = { 0 };	/* Intermediate buffer */

/* Forward declarations. */
static void eth1_input(struct netif *netif);
static void FlushFIFO(void);
static uint16_t EthReadFrame(u8 *);
static int8_t EthWriteFrame(u8 *, uint16_t);
void eth1_periodic_task(void *);
static void arp_timer(void *);



/**
 * Таймер для ARP кеша
 */
static void arp_timer(void *arg)
{
    etharp_tmr();
    sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);
}

/**
 * ARP таймер для всех интерфесов
 * Запускаем таймер для arp кеша
 */
void eth_arp_timer_start(void)
{
    etharp_init();
    sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);
}


/* Чтение кадра ethernet */
static u16 EthReadFrame(u8 * Frame)
{
	u16 len;
        
        len = enc28j60_recv_packet(Frame, ETH_FRAME_SIZE);
	return len;
}

/* Передача кадра ethernet */
static int8_t EthWriteFrame(u8 * Frame, uint16_t Len)
{
        enc28j60_send_packet(Frame, Len);
	return ERR_OK;
}

/**
 * Здесь функции используемые lwIP
 */
static void low_level_init(struct netif *netif)
{
	struct ethernetif *eth = netif->state;
	xTaskHandle task = NULL;

         enc28j60_init(FULL_MAC);

	eth->link = false;
	eth->phy_ok = true;

	/* set MAC hardware address length */
	netif->hwaddr_len = ETHARP_HWADDR_LEN;

	/* set MAC hardware address */
#if 1
	netif->hwaddr[0] = MAC_ADDR0;
	netif->hwaddr[1] = MAC_ADDR1;
	netif->hwaddr[2] = MAC_ADDR2;
	netif->hwaddr[3] = MAC_ADDR3;
	netif->hwaddr[4] = MAC_ADDR4;
	netif->hwaddr[5] = MAC_ADDR5;
#else
	netif->hwaddr[5] = MAC_ADDR0;
	netif->hwaddr[4] = MAC_ADDR1;
	netif->hwaddr[3] = MAC_ADDR2;
	netif->hwaddr[2] = MAC_ADDR3;
	netif->hwaddr[1] = MAC_ADDR4;
	netif->hwaddr[0] = MAC_ADDR5;
#endif
	/* initialize MAC address in ethernet MAC */

	/* maximum transfer unit */
	netif->mtu = 1500;

	/* device capabilities */
	/* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
	netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

	s_pxNetIf = netif;

	/* Делаем счетный семафор! */
	if (s_xSemaphore == NULL) {
		s_xSemaphore = xSemaphoreCreateCounting(10, 0);
		//s_xSemaphore = xSemaphoreCreateBinary();
	}

	/* Создадим задачу, чтобы она опрашивала eth1 */
        xTaskCreate(eth1_periodic_task, "eth1 periodic", netifINTERFACE_TASK_STACK_SIZE, s_pxNetIf, netifINTERFACE_TASK_PRIORITY + 3, &task);              
	if (task == NULL) {
		printf("ERROR: Create eth1 periodic Task\r\n");
		vTaskDelete(task);
	}
	printf("SUCCESS: Create eth1 periodic Task. Mac addr: %02X-%02X-%02X-%02X-%02X-%02X\r\n",
		   FULL_MAC[0], FULL_MAC[1], FULL_MAC[2], FULL_MAC[3], FULL_MAC[4], FULL_MAC[5]);

	/* Приемник и передатчик вкл */
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
	struct pbuf *q;
	int framelen = 0;

	err_t Err;

#if ETH_PAD_SIZE
	pbuf_header(p, -ETH_PAD_SIZE);	/* drop the padding word */
#endif

	for (q = p; q != NULL; q = q->next) {
		/* Send the data from the pbuf to the interface, one pbuf at a
		   time. The size of the data in each pbuf is kept in the ->len
		   variable. */
		memcpy(&EthFrame[framelen], q->payload, q->len);
		framelen += q->len;
	}
	/* The above memcpy() reduces the system performance, but 
	   it has to be done, as the RTE ethernet driver expects only
	   one and continuous packet data buffer. */
	Err = EthWriteFrame(EthFrame, framelen);

#if ETH_PAD_SIZE
	pbuf_header(p, ETH_PAD_SIZE);	/* reclaim the padding word */
#endif

	LINK_STATS_INC(link.xmit);

	return Err;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *low_level_input(struct netif *netif)
{
	struct pbuf *p, *q;
	int len, framelen;

	/* Obtain the size of the packet and put it into the "len"   variable. */
	len = EthReadFrame(EthFrame);
//  if (len > 1514) {
//    /* Drop oversized packet */
//    eth->mac->ReadFrame (NULL, 0);
//    return NULL;
//  }

#if ETH_PAD_SIZE
	len += ETH_PAD_SIZE;	/* allow room for Ethernet padding */
#endif

	/* We allocate a pbuf chain of pbufs from the pool. */
	p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
	if (p != NULL) {

#if ETH_PAD_SIZE
		pbuf_header(p, -ETH_PAD_SIZE);	/* drop the padding word */
#endif

		/* Copy the data to intermediate buffer. This is required because
		   the RTE ethernet driver copies all the data to one
		   continuous packet data buffer. */
		framelen = 0;
		/* We iterate over the pbuf chain until we have read the entire
		 * packet into the pbuf. */
		for (q = p; q != NULL; q = q->next) {
			/* Read enough bytes to fill this pbuf in the chain. The
			 * available data in the pbuf is given by the q->len
			 * variable.
			 * This does not necessarily have to be a memcpy, you can also preallocate
			 * pbufs for a DMA-enabled MAC and after receiving truncate it to the
			 * actually received size. In this case, ensure the tot_len member of the
			 * pbuf is the sum of the chained pbuf len members.
			 */
			memcpy(q->payload, &EthFrame[framelen], q->len);
			framelen += q->len;
		}

#if ETH_PAD_SIZE
		pbuf_header(p, ETH_PAD_SIZE);	/* reclaim the padding word */
#endif

		LINK_STATS_INC(link.recv);
	} else {
		/* drop packet(); */
		LINK_STATS_INC(link.memerr);
		LINK_STATS_INC(link.drop);
	}

	return p;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
static void eth1_input(struct netif *netif)
{
	struct eth_hdr *ethhdr;
	struct pbuf *p;

	/* move received packet into a new pbuf */
	p = low_level_input(netif);

	/* no packet could be read, silently ignore this */
	if (p == NULL)
		return;
	/* points to packet payload, which starts with an Ethernet header */
	ethhdr = p->payload;

	switch (htons(ethhdr->type)) {
		/* IP or ARP packet? */
	case ETHTYPE_IP:
	case ETHTYPE_ARP:
#if PPPOE_SUPPORT
		/* PPPoE packet? */
	case ETHTYPE_PPPOEDISC:
	case ETHTYPE_PPPOE:
#endif				/* PPPOE_SUPPORT */
		/* full packet send to tcpip_thread to process */
		if (netif->input(p, netif) != ERR_OK) {
			LWIP_DEBUGF(NETIF_DEBUG, ("eth1_input: IP input error\n"));
			pbuf_free(p);
			p = NULL;
		}
		break;

	default:
		pbuf_free(p);
		p = NULL;
		break;
	}
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t eth1_init(struct netif *netif)
{
	struct ethernetif *eth;

	LWIP_ASSERT("netif != NULL", (netif != NULL));

	eth = (struct ethernetif *) mem_malloc(sizeof(struct ethernetif));
	if (eth == NULL) {
		LWIP_DEBUGF(NETIF_DEBUG, ("eth1_init: out of memory\n"));
		return ERR_MEM;
	}
#if LWIP_NETIF_HOSTNAME
	/* Initialize interface hostname */
	netif->hostname = MY_HOSTNAME;
#endif				/* LWIP_NETIF_HOSTNAME */

	/*
	 * Initialize the snmp variables and counters inside the struct netif.
	 * The last argument should be replaced with your link speed, in units
	 * of bits per second.
	 */
/*    NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_NETIF_IN_BPS); */

	netif->state = eth;
	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;
	/* We directly use etharp_output() here to save a function call.
	 * You can instead declare your own function an call etharp_output()
	 * from it if you have to do some checks before sending (e.g. if link
	 * is available...) */
	netif->output = etharp_output;
	netif->linkoutput = low_level_output;

	/* initialize the hardware */
	low_level_init(netif);

	return ERR_OK;
}

void eth1_poll(struct netif *netif)
{
    struct ethernetif *eth = netif->state;

 //   if (!eth->phy_ok || eth->link == false)
//	return;

	eth1_input(netif);
}



/**
 * Вызывается каждый раз в бесконечном цикле
 * Здесь определяем задачи которые должны выполняться
 */
void eth1_periodic_task(void *par)
{
    while (1) {
	eth1_poll(s_pxNetIf);	/* check if any packet received */
        vTaskDelay(5);
    }
}
