#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <portable.h>
#include <semphr.h>
#include <task.h>
#include <udpecho.h>

#include <stm32f4xx.h>
#include <enc28j60.h>

#include <stm32f4_discovery.h>   
#include <serial_debug.h>

   
   
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define USE_LCD        /* enable LCD  */  
#undef  USE_LCD        /* enable LCD  */  

//#define USE_DHCP       /* enable DHCP, if disabled static address is used*/
    

/* MAC ADDRESS*/
 
/*Static IP ADDRESS*/
#define IP_ADDR0   10
#define IP_ADDR1   58
#define IP_ADDR2   6
#define IP_ADDR3   101
   
/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0

/*Gateway Address*/
#define GW_ADDR0   10
#define GW_ADDR1   58
#define GW_ADDR2   6
#define GW_ADDR3   1  



void Time_Update(void);
void Delay(uint32_t nCount);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


