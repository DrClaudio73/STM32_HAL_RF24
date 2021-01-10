All merit goes to TMRh20 for the library relevant to nRF24L01 (with variants). Thanks to them for invaluable effort and great software product.

This is my personal attempt to port their library on the STM32 - HAL framework.
Initially, I considered to use a dedicated TIMER TIM1 to implement delayMicorseconds and getMicroseconds().
Eventually, I found the code to do that with sysTick and considered this to be optimal since it does not engage a dedicated timer and hence reduce possible conflicts with other libraries or user code needs.

The code adds to the RF24 library class an object of class UART24 and one of class SPIRF24.
These two classes take care of implementing the:

1. SPI communication with nRF24L01 
2. UART for debug purpose including printing out the received/sent packets

Nevertheless printf usage still needs to modify syscall.c: this might still be an optimization to be introduced.

