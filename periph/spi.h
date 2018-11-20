#ifndef _SPI_H
#define _SPI_H

#include "main.h"


/**
 * Дефиниции для ног на ETH
 */
/**** Пин RST на PB0 */     
#define ETH_SPI_RESET_PIN           	  GPIO_Pin_0	/* PB0 */
#define ETH_SPI_RESET_GPIO_PORT           GPIOB	/* GPIOB */
#define ETH_SPI_RESET_GPIO_CLK            RCC_AHB1Periph_GPIOB


#define ETH_SPI                           SPI2
#define ETH_SPI_CLK                       RCC_APB1Periph_SPI2
#define ETH_GPIO_AF	                  GPIO_AF_SPI2


/**** Пин SCK на PB10 */
#define ETH_SPI_SCK_PIN                   GPIO_Pin_10   /*PB10	 */
#define ETH_SPI_SCK_GPIO_PORT             GPIOB	/* GPIOB */
#define ETH_SPI_SCK_GPIO_PIN_SOURCE       GPIO_PinSource10	/* 10 */
#define ETH_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB

/* MISO на PB14 */
#define ETH_SPI_MISO_PIN                  GPIO_Pin_14	/* PB14 */
#define ETH_SPI_MISO_GPIO_PORT            GPIOB	/* GPIOB */
#define ETH_SPI_MISO_GPIO_PIN_SOURCE      GPIO_PinSource14	
#define ETH_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOB


/* MOSI на PB15 */
#define ETH_SPI_MOSI_PIN                  GPIO_Pin_15	/* PB15 */
#define ETH_SPI_MOSI_GPIO_PORT            GPIOB	/* GPIOB */
#define ETH_SPI_MOSI_GPIO_PIN_SOURCE      GPIO_PinSource15	/* 15 */
#define ETH_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOB


/* CS на PB12 */
#define ETH_SPI_CS_PIN                  GPIO_Pin_12	/* PB12 */
#define ETH_SPI_CS_GPIO_PORT            GPIOB		/* GPIOB */
#define ETH_SPI_CS_GPIO_PIN_SOURCE      GPIO_PinSource12	/* 12 */
#define ETH_SPI_CS_GPIO_CLK             RCC_AHB1Periph_GPIOB



#define ETH_RESET_LOW()    do { GPIO_ResetBits(ETH_SPI_RESET_GPIO_PORT, ETH_SPI_RESET_PIN); } while(0)
#define ETH_RESET_HI()     do { GPIO_SetBits(ETH_SPI_RESET_GPIO_PORT, ETH_SPI_RESET_PIN); } while(0)


#define ETH_CS_LOW()    do {  GPIO_ResetBits(ETH_SPI_CS_GPIO_PORT, ETH_SPI_CS_PIN); } while(0)
#define ETH_CS_HI()     do { GPIO_SetBits(ETH_SPI_CS_GPIO_PORT, ETH_SPI_CS_PIN); } while(0)


void spi_init(void);
u8   spi_in_out(u8 data);


#endif /* spi.h  */
