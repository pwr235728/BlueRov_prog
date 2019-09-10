################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/frame_parser.c \
../Src/main.c \
../Src/rs485.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f1xx.c 

OBJS += \
./Src/frame_parser.o \
./Src/main.o \
./Src/rs485.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f1xx.o 

C_DEPS += \
./Src/frame_parser.d \
./Src/main.d \
./Src/rs485.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103xB -I"/home/proxima/workspace/BlueRov/Inc" -I"/home/proxima/workspace/BlueRov/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/proxima/workspace/BlueRov/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/proxima/workspace/BlueRov/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/proxima/workspace/BlueRov/Drivers/CMSIS/Include"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


