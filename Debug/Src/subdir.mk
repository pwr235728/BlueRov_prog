################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/frame_parser.c \
../Src/helpfer_functions.c \
../Src/main.c \
../Src/rs485.c \
../Src/stm32f1xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f1xx.c 

OBJS += \
./Src/frame_parser.o \
./Src/helpfer_functions.o \
./Src/main.o \
./Src/rs485.o \
./Src/stm32f1xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f1xx.o 

C_DEPS += \
./Src/frame_parser.d \
./Src/helpfer_functions.d \
./Src/main.d \
./Src/rs485.d \
./Src/stm32f1xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DUSE_FULL_LL_DRIVER '-DHSE_VALUE=8000000' '-DHSE_STARTUP_TIMEOUT=100' '-DLSE_STARTUP_TIMEOUT=5000' '-DLSE_VALUE=32768' '-DHSI_VALUE=8000000' '-DLSI_VALUE=40000' '-DVDD_VALUE=3300' '-DPREFETCH_ENABLE=1' '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DSTM32F103xB -I"C:/Users/Kurat/Documents/git_repos/BlueRov_prog/Inc" -I"C:/Users/Kurat/Documents/git_repos/BlueRov_prog/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Kurat/Documents/git_repos/BlueRov_prog/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Kurat/Documents/git_repos/BlueRov_prog/Drivers/CMSIS/Include"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


