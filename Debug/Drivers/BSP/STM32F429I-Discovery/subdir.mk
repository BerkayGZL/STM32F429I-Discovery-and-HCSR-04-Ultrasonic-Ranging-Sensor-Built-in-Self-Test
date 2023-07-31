################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery.c \
../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_eeprom.c \
../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_gyroscope.c \
../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_io.c \
../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.c \
../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_sdram.c \
../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_ts.c 

OBJS += \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery.o \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_eeprom.o \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_gyroscope.o \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_io.o \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.o \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_sdram.o \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_ts.o 

C_DEPS += \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery.d \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_eeprom.d \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_gyroscope.d \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_io.d \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.d \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_sdram.d \
./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_ts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32F429I-Discovery/%.o: ../Drivers/BSP/STM32F429I-Discovery/%.c Drivers/BSP/STM32F429I-Discovery/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_STM32F429I_DISCO -DUSE_HAL_DRIVER -DSTM32F439xx -c -I../Core/Inc -I../Drivers/BSP/STM32F429I-Discovery -I"C:/Users/BERKAY/STM32CubeIDE/workspace_1.8.0/BSP_SDRAM4/Drivers/BSP/STM32F429I-Discovery" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32F429I-2d-Discovery

clean-Drivers-2f-BSP-2f-STM32F429I-2d-Discovery:
	-$(RM) ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery.d ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery.o ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_eeprom.d ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_eeprom.o ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_gyroscope.d ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_gyroscope.o ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_io.d ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_io.o ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.d ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.o ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_sdram.d ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_sdram.o ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_ts.d ./Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_ts.o

.PHONY: clean-Drivers-2f-BSP-2f-STM32F429I-2d-Discovery

