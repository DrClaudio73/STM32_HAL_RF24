/*
 * SPI.cpp
 *
 *  Created on: 29 dic 2020
 *      Author: ccattaneo
 */

#include "SPI_RF24.hpp"

#include "main.h"
#include "RF24_config.hpp"

#define RF24_SPIDEV_BITS 8

SPI_RF24::SPI_RF24()
        :fd(-1), _spi_speed(RF24_SPI_SPEED)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* Peripheral clock enable */
	__HAL_RCC_SPI1_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**SPI1 GPIO Configuration
	    PA5     ------> SPI1_SCK
	    PA6     ------> SPI1_MISO
	    PA7     ------> SPI1_MOSI
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SPI_RF24::begin(uint32_t spi_speed)
{

    if (this->spiIsInitialized) {
        return;
    }

    this->spiIsInitialized = true;
    init(spi_speed);
}

void SPI_RF24::init(uint32_t speed)
{
	MX_SPI1_Init();
}

uint8_t SPI_RF24::transfer(uint8_t tx)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 1000);

    return rx;
}

void SPI_RF24::transfernb(char* tbuf, char* rbuf, uint32_t len)
{
	uint8_t *txbuf;
	uint8_t *rxbuf;
	txbuf = (uint8_t*) tbuf;
	rxbuf = (uint8_t*) rbuf;
	uint16_t len2 = (uint16_t) len;

	HAL_SPI_TransmitReceive(&hspi1, txbuf, rxbuf, len2, 2000);
}

SPI_RF24::~SPI_RF24()
{
	__HAL_RCC_SPI1_CLK_DISABLE();

	/**SPI1 GPIO Configuration
	PA5     ------> SPI1_SCK
	PA6     ------> SPI1_MISO
	PA7     ------> SPI1_MOSI
	*/
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
}

void SPI_RF24::MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}
