/*
 * UART_RF24.hpp
 *
 *  Created on: 30 dic 2020
 *      Author: ccattaneo
 */

#ifndef UART_RF24_HPP_
#define UART_RF24_HPP_

#include "RF24_config.hpp"

class UART_RF24 {

public:

    /**
    * TIMER_RF24 constructor
    */
	UART_RF24();

    /**
    * Start TIMER_RF24
    */
    void begin(void);


    ~UART_RF24();
    UART_HandleTypeDef huartRF24;
    HAL_StatusTypeDef transmit(char* tbuf, size_t len);
    HAL_StatusTypeDef receive(char* tbuf, size_t len);
    void print(char* format,char *param1);

private:
    bool uartIsInitialized = false;
    void init(void);
    void MX_USART1_UART_Init(void);
};

#endif /* UART_RF24_HPP_ */
