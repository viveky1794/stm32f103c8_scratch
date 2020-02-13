################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../myDriver_001/SRC/clock_Driver.c \
../myDriver_001/SRC/gpio_Driver.c 

OBJS += \
./myDriver_001/SRC/clock_Driver.o \
./myDriver_001/SRC/gpio_Driver.o 

C_DEPS += \
./myDriver_001/SRC/clock_Driver.d \
./myDriver_001/SRC/gpio_Driver.d 


# Each subdirectory must supply rules for building sources it contributes
myDriver_001/SRC/clock_Driver.o: ../myDriver_001/SRC/clock_Driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"H:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/INC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/INC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"myDriver_001/SRC/clock_Driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
myDriver_001/SRC/gpio_Driver.o: ../myDriver_001/SRC/gpio_Driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"H:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/INC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001/INC" -I"E:/udmey Content/STM32F103C8 Driver/gpio_Driver_001/myDriver_001" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"myDriver_001/SRC/gpio_Driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

