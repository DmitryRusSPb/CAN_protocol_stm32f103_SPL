################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/can.c \
../src/delay.c \
../src/main.c \
../src/syscalls.c \
../src/system_stm32f10x.c 

OBJS += \
./src/can.o \
./src/delay.o \
./src/main.o \
./src/syscalls.o \
./src/system_stm32f10x.o 

C_DEPS += \
./src/can.d \
./src/delay.d \
./src/main.d \
./src/syscalls.d \
./src/system_stm32f10x.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F103RETx -DDEBUG -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -I"/home/user/workspaceSTM32/CAN_protocol_stm32f103_SPL/StdPeriph_Driver/inc" -I"/home/user/workspaceSTM32/CAN_protocol_stm32f103_SPL/inc" -I"/home/user/workspaceSTM32/CAN_protocol_stm32f103_SPL/CMSIS/device" -I"/home/user/workspaceSTM32/CAN_protocol_stm32f103_SPL/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


