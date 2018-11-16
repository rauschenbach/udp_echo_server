#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__

#include <inttypes.h>
#include <stdbool.h>
#include "lwip/err.h"
#include "lwip/netif.h"


#define	ETH_FRAME_SIZE				1514
#define netifMTU                                (1500)
#define netifINTERFACE_TASK_STACK_SIZE		(configMINIMAL_STACK_SIZE)
#define netifINTERFACE_TASK_PRIORITY		(tskIDLE_PRIORITY + 3)
#define netifGUARD_BLOCK_TIME			(250)


/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
struct ethernetif {
    bool link;			/* Ethernet Link State */
    bool phy_ok;		/* Phy initialized successfully */
    /* Add whatever per-interface state that is needed here. */
};


err_t eth1_init(struct netif *netif);



#endif 
