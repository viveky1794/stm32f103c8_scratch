################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include myDriver_001/SRC/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
gpio_Driver_001.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
gpio_Driver_001.list \


# All Target
all: gpio_Driver_001.elf secondary-outputs

# Tool invocations
gpio_Driver_001.elf: $(OBJS) $(USER_OBJS) E:\udmey\ Content\STM32F103C8\ Driver\gpio_Driver_001\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "gpio_Driver_001.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"E:\udmey Content\STM32F103C8 Driver\gpio_Driver_001\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="gpio_Driver_001.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

gpio_Driver_001.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "gpio_Driver_001.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
