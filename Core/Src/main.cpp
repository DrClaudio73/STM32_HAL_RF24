/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../RFLib/RF24.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
//SPI_HandleTypeDef hspi1; moved into SPIRF24 class

//TIM_HandleTypeDef htim1; moved into TimerRF24 class

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_SPI1_Init(void); moved into SPI class
//static void MX_USART1_UART_Init(void);
uint32_t getCurrentMicros(void);
static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void);
//static void MX_TIM1_Init(void); moved into TimerRF24 Class
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
#define NUM_BYTES 32
uint8_t tx[NUM_BYTES];

void printdata(uint8_t* dati){
	for (int i=0 ; i< NUM_BYTES; i++){
		printf(" %d",dati[i]);
		if (i<(NUM_BYTES-1)){
			printf(" -");
		}
	}
	printf("\r\n");
}

int getfromSerial(char* ptr) {
  HAL_StatusTypeDef hstatus;

    hstatus = HAL_UART_Receive(&huart1, (uint8_t *) ptr, 1, 0xFF);
    if (hstatus == HAL_OK){
    	return 1;
    }
    else
    {
    	*ptr=0;
    	return 0;
    }
}

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_SPI1_Init(); moved into SPIRFF24 class
  //MX_USART1_UART_Init();
  //MX_TIM1_Init(); moved into TimerRF24 class
  /* USER CODE BEGIN 2 */
	for (int i=0 ; i< NUM_BYTES; i++){
		tx[i]=0;
	}

  bool radioNumber = 1;
  RF24 radio(ce_pin_GPIO_Port, ce_pin_Pin, csn_pin_GPIO_Port, csn_pin_Pin);
  uint8_t addresses[][6] = {"1Node","2Node"};
  radio.begin();
  huart1 = radio.uartRF24.gethUART();

  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  //radio.enableAckPayload();               // Allow optional ack payloads
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setRetries(15, 15);                // Smallest time between retries, max no. of retries
  radio.setPayloadSize(NUM_BYTES);        // Here we are sending NUM_BYTES-bytes payloads to test the call-response speed

  if(radioNumber){
      radio.openWritingPipe(addresses[1]);
      radio.openReadingPipe(1,addresses[0]);
    }else{
      radio.openWritingPipe(addresses[0]);
      radio.openReadingPipe(1,addresses[1]);
    }

  radio.startListening();
  radio.printDetails();
  printf("RF24/examples/GettingStarted -- STM32\r\n");
  printf("*** PRESS 'T' to begin transmitting to the other node\r\n");
  radio.printPrettyDetails();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool role = 0;
  while (1)
  {
	  /*
	  char buf[1025],buf2[1025];
	  memset(buf,0,1025);
	  printf("\r\nYour name: ");
	  scanf("%s %s", buf,buf2);
	  printf("\r\nHello, %s %s!\r\n", buf,buf2);*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (role == 1)  {

		  radio.stopListening();                                    // First, stop listening so we can talk.

		  printf(" \r\nNow sending");
		  tx[0]=tx[0]+1;
		  for (int i= 1; i<NUM_BYTES; i++){
			  tx[i]=tx[i-1]+1;
		  }
		  printdata(tx);

		  unsigned long start_time = getCurrentMicros();                             // Take the time, and send it.  This will block until complete

		  if (!radio.write( &tx, NUM_BYTES )){
			  printf("failed\r\n");
		  } else {
			  printf("sent ok!\r\n");
		  }

		  radio.startListening();                                    // Now, continue listening

		  unsigned long started_waiting_at = getCurrentMicros();               // Set up a timeout period, get the current microseconds
		  unsigned long istante;
		  bool timeout = false;                                   // Set up a variable to indicate if a response was received or not

		  while ( ! radio.available() ){                             // While nothing is received
			  istante = getCurrentMicros();
			  if (istante - started_waiting_at > 200000 ){            // If waited longer than 200ms, indicate timeout and exit while loop
				  timeout = true;
				  break;
			  }
		  }

		  if ( timeout ){                                             // Describe the results
			  printf("Failed, response timed out. \r\n");
		  }else{
			  printf("Success, time out has not been triggered. \r\n");
		        uint8_t rx[NUM_BYTES];                                 // Grab the response, compare, and send to debugging spew
		        for (int i=0 ; i< NUM_BYTES; i++){
		          rx[i]=0;
		        }
		        radio.read( &rx, NUM_BYTES );
		        printf("Sent:\t\t");
		        printdata(tx);
		        printf("Got response:\t");
		        printdata(rx);
		        radio.flush_tx();
		        //printf(", Round-trip delay \r\n");
		        //printf(end_time-start_time);
		        //printfln(" microseconds");
		  }

		  // Try again 1s later
		  //delay(100);
	  }

	  /****************** Pong Back Role ***************************/

	  if ( role == 0 )
	  {
		  //unsigned long got_time;
		  uint8_t rx[NUM_BYTES];                                 // Grab the response, compare, and send to debugging spew
		  for (int i=0 ; i< NUM_BYTES; i++){
			  rx[i]=0;
		  }
		  if( radio.available()){
			  while (radio.available()) {             // While there is data ready
				  radio.read( &rx, NUM_BYTES );       // Get the payload
			  }
			  radio.stopListening();                                        // First, stop listening so we can talk
			  radio.write( &rx, NUM_BYTES );              // Send the final one back.
			  radio.startListening();                                       // Now, resume listening so we catch the next packets.
			  printf("Sent response: ");
			  printdata(rx);
			  //printf(got_time);
		  }
	  }

	  /****************** Change Roles via Serial Commands ***************************/
	  uint8_t c;
	  //scanf("%c", &c );
	  //c=getFromSerial();
	  c=getchar();
	  //printf("YOU TYPED %c %d\r\n",c,c);
	  if (1)
	  {
		  //uint8_t c = buf[0];
		  //char c = toupper(Serial.read());
		  if ( (c == 'T' || c=='t') && role == 0 ){
			  radio.flush_tx();
		  	  tx[0]=0;
		  	  printf("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK\r\n");
			  role = 1;                  // Become the primary transmitter (ping out)
			  c=0;

		  }else
			  if ( (c == 'R' || c == 'r') && role == 1 ){
				  radio.flush_tx();
			  	  tx[0]=0;
			  	  printf("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK\r\n");
				  role = 0;                // Become the primary receiver (pong back)
				  radio.startListening();
				  c=0;
			  }else {
				    if ( (c == 'P' || c == 'p')  ){
				      printf("*** PRINTIG DEBUG INFO ***\r\n");
				      radio.printDetails();                   // Dump the configuration of the rf unit for debugging
				      printf("*** ***** ***");
				      radio.printPrettyDetails();
				    }
			  }
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None

static void MX_SPI1_Init(void)
{
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
}*/

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
static void MX_TIM1_Init(void)
{


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}*/

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None

static void MX_USART1_UART_Init(void)
{

   huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}*/

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  //GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  //__HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(csn_pin_GPIO_Port, csn_pin_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(ce_pin_GPIO_Port, ce_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : csn_pin_Pin
  GPIO_InitStruct.Pin = csn_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(csn_pin_GPIO_Port, &GPIO_InitStruct);

  //Configure GPIO pin : ce_pin_Pin
  GPIO_InitStruct.Pin = ce_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ce_pin_GPIO_Port, &GPIO_InitStruct);
*/
}

/* USER CODE BEGIN 4 */
uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  LL_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (LL_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}

static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
