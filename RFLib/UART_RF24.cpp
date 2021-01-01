/*
 * UART_RF24.cpp
 *
 *  Created on: 30 dic 2020
 *      Author: ccattaneo
 */


#include "UART_RF24.hpp"
#include "main.h"
#include "RF24_config.hpp"
#include <stdio.h>
#include <string.h>

UART_RF24::UART_RF24()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* Peripheral clock enable */
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART1 GPIO Configuration
	    PA9     ------> USART1_TX
	    PA10     ------> USART1_RX
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void UART_RF24::begin()
{

    if (this->uartIsInitialized) {
        return;
    }

    this->uartIsInitialized = true;
    init();
}

void UART_RF24::init()
{
	MX_USART1_UART_Init();
}

UART_RF24::~UART_RF24()
{
	__HAL_RCC_USART1_CLK_DISABLE();

	/**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
	 */
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
}

HAL_StatusTypeDef UART_RF24::transmit(char* tbuf, size_t len)
{
	uint8_t *txbuf;
	//uint8_t *rxbuf;
	txbuf = (uint8_t*) tbuf;
	//rxbuf = (uint8_t*) rbuf;
	uint16_t len2 = (uint16_t) len;

	return HAL_UART_Transmit(&huartRF24,txbuf, len2, 2000);
}

HAL_StatusTypeDef UART_RF24::receive(char* rbuf, size_t len)
{
	//uint8_t *txbuf;
	uint8_t *rxbuf;
	//txbuf = (uint8_t*) tbuf;
	rxbuf = (uint8_t*) rbuf;
	uint16_t len2 = (uint16_t) len;

	return HAL_UART_Receive(&huartRF24,rxbuf, len2, 2000);
}

void UART_RF24::print(char* format,char *param1){
	char loc_buf[555];
	sprintf(loc_buf, format, param1);
	transmit(loc_buf, strlen((char *) param1));
}

void UART_RF24::MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huartRF24.Instance = USART1;
  huartRF24.Init.BaudRate = 115200;
  huartRF24.Init.WordLength = UART_WORDLENGTH_8B;
  huartRF24.Init.StopBits = UART_STOPBITS_1;
  huartRF24.Init.Parity = UART_PARITY_NONE;
  huartRF24.Init.Mode = UART_MODE_TX_RX;
  huartRF24.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huartRF24.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huartRF24) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
