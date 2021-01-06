/*
 * TimerRF24.cpp
 *
 *  Created on: 29 dic 2020
 *      Author: ccattaneo
 */


#include "TimerRF24.hpp"
#include "main.h"
#include "RF24_config.hpp"


TIMER_RF24::TIMER_RF24()
{
	__HAL_RCC_TIM1_CLK_ENABLE();
}

void TIMER_RF24::begin()
{

    if (this->timerIsInitialized) {
        return;
    }

    this->timerIsInitialized = true;
    //__HAL_RCC_TIM1_CLK_ENABLE();
    init();

    HAL_TIM_Base_Start(&htim_us);
}

void TIMER_RF24::setCNT(uint16_t val){
	__HAL_TIM_SET_COUNTER(&htim_us,val);  // set the counter value a 0
}

uint16_t TIMER_RF24::getCNT(void){
	return (__HAL_TIM_GET_COUNTER(&htim_us));  // get the counter value
}

void TIMER_RF24::init()
{
	MX_TIM1_Init();
}

TIMER_RF24::~TIMER_RF24()
{
	__HAL_RCC_TIM1_CLK_DISABLE();
}

void TIMER_RF24::MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim_us.Instance = TIM1;
  htim_us.Init.Prescaler = 72-1;
  htim_us.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_us.Init.Period = 65535-1;
  htim_us.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim_us.Init.RepetitionCounter = 0;
  htim_us.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim_us) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim_us, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim_us, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}
