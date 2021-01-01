/*
 * TimerRF24.hpp
 *
 *  Created on: 29 dic 2020
 *      Author: ccattaneo
 */

#ifndef TIMERRF24_HPP_
#define TIMERRF24_HPP_
#include "RF24_config.hpp"

class TIMER_RF24 {

public:

    /**
    * TIMER_RF24 constructor
    */
	TIMER_RF24();

    /**
    * Start TIMER_RF24
    */
    void begin(void);
    void setCNT(uint16_t val);
    uint16_t getCNT(void);

    ~TIMER_RF24();
private:
    TIM_HandleTypeDef htim_us;

    bool timerIsInitialized = false;
    void init(void);
    void MX_TIM1_Init(void);
};


#endif /* TIMERRF24_HPP_ */
