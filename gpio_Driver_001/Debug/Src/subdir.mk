################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/main.o: ../Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/Src" -I"H:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/INC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/Src" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/INC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/SRC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/Src" -I"H:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/INC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/Src" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/INC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/SRC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/Src" -I"H:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/INC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/Src" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/INC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/SRC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

