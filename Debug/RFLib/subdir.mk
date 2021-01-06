################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../RFLib/RF24.cpp \
../RFLib/SPI_RF24.cpp \
../RFLib/TimerRF24.cpp \
../RFLib/UART_RF24.cpp 

OBJS += \
./RFLib/RF24.o \
./RFLib/SPI_RF24.o \
./RFLib/TimerRF24.o \
./RFLib/UART_RF24.o 

CPP_DEPS += \
./RFLib/RF24.d \
./RFLib/SPI_RF24.d \
./RFLib/TimerRF24.d \
./RFLib/UART_RF24.d 


# Each subdirectory must supply rules for building sources it contributes
RFLib/RF24.o: ../RFLib/RF24.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"RFLib/RF24.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
RFLib/SPI_RF24.o: ../RFLib/SPI_RF24.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"RFLib/SPI_RF24.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
RFLib/TimerRF24.o: ../RFLib/TimerRF24.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"RFLib/TimerRF24.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
RFLib/UART_RF24.o: ../RFLib/UART_RF24.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"RFLib/UART_RF24.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

