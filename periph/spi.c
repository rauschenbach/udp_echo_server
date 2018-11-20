#include "main.h"
#include "spi.h"


/**
 * setup hardware pins, prepare SPI subsystem 
 */
void spi_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	/*Разрешим тактирование ETH_SPI_CLK */
        RCC_APB1PeriphClockCmd(ETH_SPI_CLK, ENABLE);

	/* выводы MOSI MISO и прочие */
	RCC_AHB1PeriphClockCmd(ETH_SPI_RESET_GPIO_CLK | ETH_SPI_SCK_GPIO_CLK | ETH_SPI_CS_GPIO_CLK |
				ETH_SPI_MISO_GPIO_CLK | ETH_SPI_MOSI_GPIO_CLK, ENABLE);


	GPIO_PinAFConfig(ETH_SPI_SCK_GPIO_PORT, ETH_SPI_SCK_GPIO_PIN_SOURCE, ETH_GPIO_AF);
	GPIO_PinAFConfig(ETH_SPI_MOSI_GPIO_PORT, ETH_SPI_MOSI_GPIO_PIN_SOURCE, ETH_GPIO_AF);
	GPIO_PinAFConfig(ETH_SPI_MISO_GPIO_PORT, ETH_SPI_MISO_GPIO_PIN_SOURCE, ETH_GPIO_AF);


	/* Init GPIO */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	/*!< SPI SCK pin configuration - Альтернативная функция: SCK */
	GPIO_InitStructure.GPIO_Pin = ETH_SPI_SCK_PIN;
	GPIO_Init(ETH_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);


	/*!< SPI MOSI pin configuration - Альтернативная функция: MOSI */
	GPIO_InitStructure.GPIO_Pin = ETH_SPI_MOSI_PIN;
	GPIO_Init(ETH_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);


	/*!< SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin = ETH_SPI_MISO_PIN;
	GPIO_Init(ETH_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);


	/* SPI configuration */
	SPI_I2S_DeInit(ETH_SPI);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
        

	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(ETH_SPI, &SPI_InitStructure);
	SPI_Cmd(ETH_SPI, ENABLE);


	/* Configure PIN for RESET */
	GPIO_InitStructure.GPIO_Pin = ETH_SPI_RESET_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(ETH_SPI_RESET_GPIO_PORT, &GPIO_InitStructure);


	/* Configure CS */
	GPIO_InitStructure.GPIO_Pin = ETH_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ETH_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

	ETH_RESET_LOW();	/*  RESET hi */
        for(int i = 0; i < 10000; i++);
	ETH_RESET_HI();	/*  RESET hi */
}


/**
 * Отправить 8 бит по SPI
 */
u8 spi_in_out(u8 data)
{
	volatile int i;

	//for(i = 0; i < 100; i++);

	/* Loop while DR register in not empty */
	while (SPI_I2S_GetFlagStatus(ETH_SPI, SPI_I2S_FLAG_TXE) == RESET) {}

	/* Send a Byte through the SPI peripheral */
	SPI_I2S_SendData(ETH_SPI, data);


	/* Wait to receive a Byte */
	while (SPI_I2S_GetFlagStatus(ETH_SPI, SPI_I2S_FLAG_RXNE) == RESET) {}

	/* Return the Byte read from the SPI bus */
	data = (uint8_t) SPI_I2S_ReceiveData(ETH_SPI);
        
        /* Задержка. CS налезает на последний импульс */
	for(i = 0; i < 100; i++);
        return data;
}
         
