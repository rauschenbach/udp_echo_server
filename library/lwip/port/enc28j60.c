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
#include "spi.h"
#include "enc28j60.h"


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


/*
 * SPI
 */
#define enc28j60_select() ETH_CS_LOW()
#define enc28j60_release() ETH_CS_HI()

#define enc28j60_rx() spi_in_out(0xff)
#define enc28j60_tx(data) spi_in_out(data)


static struct netif *s_pxNetIf = NULL;
static xSemaphoreHandle s_xSemaphore = NULL;	/* Семафор для ожидающей задачи приема */
static uint8_t enc28j60_current_bank = 0;
static uint16_t enc28j60_rxrdpt = 0;


#pragma pack(4)
static u8 EthFrame[ETH_FRAME_SIZE] = { 0 };	/* Intermediate buffer */
static u8 rx_reg = 0;		/* Биты регистров прерывания по чтению */

/* Forward declarations. */
static void enc28j60_input(struct netif *netif);
static uint16_t enc28j60_read_frame(u8 *);
static int8_t enc28j60_write_frame(u8 *, uint16_t);
static void enc28j60_periodic_task(void *);

static void print_regs(void);


/* Generic SPI read command */
static uint8_t enc28j60_read_op(uint8_t cmd, uint8_t adr)
{
    uint8_t data;

    enc28j60_select();
    enc28j60_tx(cmd | (adr & ENC28J60_ADDR_MASK));
    if (adr & 0x80)		// throw out dummy byte 
	enc28j60_rx();		// when reading MII/MAC register
    data = enc28j60_rx();
    enc28j60_release();
    return data;
}

/* Generic SPI write command */
static void enc28j60_write_op(uint8_t cmd, uint8_t adr, uint8_t data)
{
    enc28j60_select();
    enc28j60_tx(cmd | (adr & ENC28J60_ADDR_MASK));
    enc28j60_tx(data);
    enc28j60_release();
}

/* Initiate software reset */
static void enc28j60_soft_reset()
{
    enc28j60_select();
    enc28j60_tx(ENC28J60_SPI_SC);
    enc28j60_release();

    enc28j60_current_bank = 0;
//    vTaskDelay(1);            // Wait until device initializes
    for (int i = 0; i < 10000; i++);	// не в контексте ОС
}


/*
 * Memory access
 */
/* Set register bank */
static void enc28j60_set_bank(uint8_t adr)
{
    uint8_t bank;

    if ((adr & ENC28J60_ADDR_MASK) < ENC28J60_COMMON_CR) {
	bank = (adr >> 5) & 0x03;	//BSEL1|BSEL0=0x03
	if (bank != enc28j60_current_bank) {
	    enc28j60_write_op(ENC28J60_SPI_BFC, ECON1, 0x03);
	    enc28j60_write_op(ENC28J60_SPI_BFS, ECON1, bank);
	    enc28j60_current_bank = bank;
	}
    }
}

/* Read register */
static uint8_t enc28j60_rcr(uint8_t adr)
{
    enc28j60_set_bank(adr);
    return enc28j60_read_op(ENC28J60_SPI_RCR, adr);
}

/* Read register pair */
static uint16_t enc28j60_rcr16(uint8_t adr)
{
    enc28j60_set_bank(adr);
    return enc28j60_read_op(ENC28J60_SPI_RCR, adr) | (enc28j60_read_op(ENC28J60_SPI_RCR, adr + 1) << 8);
}

/* Write register */
static void enc28j60_wcr(uint8_t adr, uint8_t arg)
{
    enc28j60_set_bank(adr);
    enc28j60_write_op(ENC28J60_SPI_WCR, adr, arg);
}

/* Write register pair */
static void enc28j60_wcr16(uint8_t adr, uint16_t arg)
{
    enc28j60_set_bank(adr);
    enc28j60_write_op(ENC28J60_SPI_WCR, adr, arg);
    enc28j60_write_op(ENC28J60_SPI_WCR, adr + 1, arg >> 8);
}

/* Clear bits in register (reg &= ~mask) */
static void enc28j60_bfc(uint8_t adr, uint8_t mask)
{
    enc28j60_set_bank(adr);
    enc28j60_write_op(ENC28J60_SPI_BFC, adr, mask);
}

/* Set bits in register (reg |= mask) */
static void enc28j60_bfs(uint8_t adr, uint8_t mask)
{
    enc28j60_set_bank(adr);
    enc28j60_write_op(ENC28J60_SPI_BFS, adr, mask);
}

/* Read Rx/Tx buffer (at ERDPT) */
static void enc28j60_read_buffer(uint8_t * buf, uint16_t len)
{
    enc28j60_select();
    enc28j60_tx(ENC28J60_SPI_RBM);
    while (len--)
	*(buf++) = enc28j60_rx();
    enc28j60_release();
}

/* Write Rx/Tx buffer (at EWRPT) */
static void enc28j60_write_buffer(uint8_t * buf, uint16_t len)
{
    enc28j60_select();
    enc28j60_tx(ENC28J60_SPI_WBM);
    while (len--)
	enc28j60_tx(*(buf++));
    enc28j60_release();
}

/* Read PHY register */
static uint16_t enc28j60_read_phy(uint8_t adr)
{
    enc28j60_wcr(MIREGADR, adr);
    enc28j60_bfs(MICMD, MICMD_MIIRD);
    while (enc28j60_rcr(MISTAT) & MISTAT_BUSY);
    enc28j60_bfc(MICMD, MICMD_MIIRD);
    return enc28j60_rcr16(MIRD);
}

/* Write PHY register */
static void enc28j60_write_phy(uint8_t adr, uint16_t data)
{
    enc28j60_wcr(MIREGADR, adr);
    enc28j60_wcr16(MIWR, data);
    while (enc28j60_rcr(MISTAT) & MISTAT_BUSY);
}


/**
 * Установка обработчика прерываний на PA0 
 * чтобы можно было симулировать кнопкой
 */
static void exti0_irq_config(void)
{
    EXTI_InitTypeDef exti0;
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;

    /* Enable GPIOB clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Configure PA0 pin as input floating - подтянем вверх */
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &gpio);

    /* Connect EXTI Line0 to PA0 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    /* Configure EXTI Line0 */
    exti0.EXTI_Line = EXTI_Line0;
    exti0.EXTI_Mode = EXTI_Mode_Interrupt;
    exti0.EXTI_Trigger = EXTI_Trigger_Falling;
    exti0.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti0);

    /* Enable and set EXTI Line0 Interrupt */
    nvic.NVIC_IRQChannel = EXTI0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0x02;
    nvic.NVIC_IRQChannelSubPriority = 0x02;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

/* Чтение кадра ethernet */
static u16 enc28j60_read_frame(u8 * Frame)
{
    u16 len = 0, rxlen, status, temp;

    /* Если есть пакеты */
    if (enc28j60_rcr(EPKTCNT)) {
	enc28j60_wcr16(ERDPT, enc28j60_rxrdpt);

	enc28j60_read_buffer((void *) &enc28j60_rxrdpt, sizeof(enc28j60_rxrdpt));
	enc28j60_read_buffer((void *) &rxlen, sizeof(rxlen));
	enc28j60_read_buffer((void *) &status, sizeof(status));

	/* success */
	if (status & 0x80) {
	    len = rxlen - 4;	//throw out crc
	    if (len > ETH_FRAME_SIZE) {
		len = ETH_FRAME_SIZE;
            }
	    enc28j60_read_buffer(Frame, len);
	}
	/* Set Rx read pointer to next packet */
	temp = (enc28j60_rxrdpt - 1) & ENC28J60_BUFEND;
	enc28j60_wcr16(ERXRDPT, temp);

	/* Decrement packet counter */
	enc28j60_bfs(ECON2, ECON2_PKTDEC);
    }

    return len;
}

/**
 * Передача кадра ethernet 
 */
static int8_t enc28j60_write_frame(u8 * Frame, uint16_t Len)
{
    while (enc28j60_rcr(ECON1) & ECON1_TXRTS) {
	/* TXRTS may not clear - ENC28J60 bug. We must reset
	 * transmit logic in cause of Tx error
	 */
	if (enc28j60_rcr(EIR) & EIR_TXERIF) {
	    enc28j60_bfs(ECON1, ECON1_TXRST);
	    enc28j60_bfc(ECON1, ECON1_TXRST);
	}
    }

    enc28j60_wcr16(EWRPT, ENC28J60_TXSTART);
    enc28j60_write_buffer((uint8_t *) "\x00", 1);
    enc28j60_write_buffer(Frame, Len);

    enc28j60_wcr16(ETXST, ENC28J60_TXSTART);
    enc28j60_wcr16(ETXND, ENC28J60_TXSTART + Len);

    /* Request packet send */
    enc28j60_bfs(ECON1, ECON1_TXRTS);


    return ERR_OK;
}

/**
 * Здесь функции используемые lwIP
 */
static void low_level_init(struct netif *netif)
{
    struct ethernetif *eth = netif->state;
    xTaskHandle task = NULL;

    /* Initialize SPI */
    spi_init();

    /* Reset ENC28J60 */
    enc28j60_soft_reset();

    /* Setup Rx/Tx buffer */
    enc28j60_wcr16(ERXST, ENC28J60_RXSTART);
    enc28j60_wcr16(ERXRDPT, ENC28J60_RXSTART);
    enc28j60_wcr16(ERXND, ENC28J60_RXEND);


    enc28j60_rxrdpt = ENC28J60_RXSTART;

    /* Setup MAC */
    enc28j60_wcr(MACON1, MACON1_TXPAUS |	// Enable flow control
		 MACON1_RXPAUS | MACON1_MARXEN);	// Enable MAC Rx


    enc28j60_wcr(MACON2, 0);	// Clear reset
    enc28j60_wcr(MACON3, MACON3_PADCFG0 |	// Enable padding,
		 MACON3_TXCRCEN | MACON3_FRMLNEN | MACON3_FULDPX);	// Enable crc & frame len chk

    enc28j60_wcr16(MAMXFL, ENC28J60_MAXFRAME);
    enc28j60_wcr(MABBIPG, 0x15);	// Set inter-frame gap
    enc28j60_wcr(MAIPGL, 0x12);
    enc28j60_wcr(MAIPGH, 0x0c);

    /* Set MAC address */
    enc28j60_wcr(MAADR5, MAC_ADDR0);
    enc28j60_wcr(MAADR4, MAC_ADDR1);
    enc28j60_wcr(MAADR3, MAC_ADDR2);
    enc28j60_wcr(MAADR2, MAC_ADDR3);
    enc28j60_wcr(MAADR1, MAC_ADDR4);
    enc28j60_wcr(MAADR0, MAC_ADDR5);
    
    
    print_regs();

    
    /* Setup PHY */
    enc28j60_write_phy(PHCON1, PHCON1_PDPXMD);	// Force full-duplex mode
    enc28j60_write_phy(PHCON2, PHCON2_HDLDIS);	// Disable loopback
    enc28j60_write_phy(PHLCON, PHLCON_LACFG2 |	// Configure LED ctrl
		       PHLCON_LBCFG2 | PHLCON_LBCFG1 | PHLCON_LBCFG0 | PHLCON_LFRQ0 | PHLCON_STRCH);

    /* Enable Rx packets */
    enc28j60_bfs(ECON1, ECON1_RXEN);

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
	s_xSemaphore = xSemaphoreCreateCounting(20, 0);
	//s_xSemaphore = xSemaphoreCreateBinary();
    }

    /* Создадим задачу, чтобы она опрашивала eth1 */
    xTaskCreate(enc28j60_periodic_task, "enc28j60 periodic", netifINTERFACE_TASK_STACK_SIZE, s_pxNetIf,
		netifINTERFACE_TASK_PRIORITY, &task);
    if (task == NULL) {
	printf("ERROR: Create enc28j60 periodic Task\r\n");
	vTaskDelete(task);
    }
    printf("SUCCESS: Create enc28j60 periodic Task. Mac addr: %02X-%02X-%02X-%02X-%02X-%02X\r\n",
	   FULL_MAC[0], FULL_MAC[1], FULL_MAC[2], FULL_MAC[3], FULL_MAC[4], FULL_MAC[5]);

    /* Разрешим прерывания от платки ethernet */
    exti0_irq_config();

    enc28j60_wcr(EIE, EIE_INTIE | EIE_PKTIE);	/*  Глобальный флаг разрешения прерываний - разрешение выхода ~INT + pending Interrupt */
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
    static xSemaphoreHandle xTxSemaphore = NULL;
    err_t Err;

    /* семафор, чтобы не входить еще раз если предыдущие данные не переданы */
  if (xTxSemaphore == NULL) {
    vSemaphoreCreateBinary (xTxSemaphore);
  } 
    
   if (xSemaphoreTake(xTxSemaphore, netifGUARD_BLOCK_TIME)) {

  
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
    Err = enc28j60_write_frame(EthFrame, framelen);

#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE);	/* reclaim the padding word */
#endif

    LINK_STATS_INC(link.xmit);
    
    /* Отдаем семафор */
    xSemaphoreGive(xTxSemaphore);
   }
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
    len = enc28j60_read_frame(EthFrame);
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
static void enc28j60_input(struct netif *netif)
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
	    LWIP_DEBUGF(NETIF_DEBUG, ("enc28j60_input: IP input error\n"));
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
err_t enc28j60_init(struct netif *netif)
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


/**
 * Вызывается каждый раз в бесконечном цикле
 * Здесь определяем задачи которые должны выполняться
 */
static void enc28j60_periodic_task(void *par)
{
    while (1) {
	/* Пока есть ожидающие пакеты в буфере */
	if (xSemaphoreTake(s_xSemaphore, -1) == pdTRUE) {
	    do {
		enc28j60_input(s_pxNetIf);
		rx_reg = enc28j60_rcr(EIR);
                /* Очищаем флаг приема */
		enc28j60_bfc(EIR, EIR_PKTIF);                

		/* попробуем пока очистить */
		if (rx_reg & EIR_RXERIF) {
		    enc28j60_bfc(EIR, EIR_RXERIF);
		    printf("Rx overflow %02X\r\n", rx_reg);
                    print_regs();
                    
                    //enc28j60_wcr(EIE, EIE_INTIE | EIE_RXERIE);	/*  Глобальный флаг разрешения прерываний - разрешение выхода ~INT + pending Interrupt */                    
		}               
	    } while (rx_reg & EIR_PKTIF);
	}
    }
}


/**
 * Функция обработки прерываний платки ethernet
 */
void enc28j60_isr(void* par)
{
    rx_reg = enc28j60_rcr(EIR);

    /* Прием, переполнение или потеря пакета */
    if (rx_reg & EIR_PKTIF) {
	STM_EVAL_LEDToggle(LED6);
        xSemaphoreGiveFromISR(s_xSemaphore, (signed portBASE_TYPE *)par);
    }
}


void print_regs(void){
    u16 data;      
    data = enc28j60_rcr16(ERXST);
    printf("ERXST=%04X\r\n", data);
    data = enc28j60_rcr16(ERXRDPT);
    printf("ERXRDPT=%04X\r\n", data);
    data = enc28j60_rcr16(ERXND);
    printf("ERXND=%04X\r\n", data);
    data = enc28j60_rcr(MACON1);
    printf("MACON1=%02X\r\n", data);
    data = enc28j60_rcr(MACON2);
    printf("MACON2=%02X\r\n", data);
    data = enc28j60_rcr(MACON3);
    printf("MACON3=%02X\r\n", data);
    data = enc28j60_rcr16(MAMXFL);
    printf("MAMXFL=%04X\r\n", data);
    data = enc28j60_rcr(MABBIPG);
    printf("MABBIPG=%02X\r\n", data);
    data = enc28j60_rcr(MAIPGL);
    printf("MAIPGL=%02X\r\n", data);
    data = enc28j60_rcr(MAIPGH);
    printf("MAIPGH=%02X\r\n", data);
    data = enc28j60_rcr(MAADR5);
    printf("MAADR5=%02X\r\n", data);
    data = enc28j60_rcr(MAADR4);
    printf("MAADR4=%02X\r\n", data);
    data = enc28j60_rcr(MAADR3);
    printf("MAADR3=%02X\r\n", data);
    data = enc28j60_rcr(MAADR2);
    printf("MAADR2=%02X\r\n", data);
    data = enc28j60_rcr(MAADR1);
    printf("MAADR1=%02X\r\n", data);
    data = enc28j60_rcr(MAADR0);
    printf("MAADR0=%02X\r\n", data);
}
