################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
stm32f103rbt_adc.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
stm32f103rbt_adc.list \

OBJCOPY_HEX += \
stm32f103rbt_adc.hex \

OBJCOPY_BIN += \
stm32f103rbt_adc.bin \


# All Target
all: stm32f103rbt_adc.elf secondary-outputs

# Tool invocations
stm32f103rbt_adc.elf: $(OBJS) $(USER_OBJS) C:\Users\Moh\ Hisyam\ Faiz\STM32CubeIDE\workspace_1.3.0\stm32f103rbt_adc\STM32F103RBTX_FLASH.ld
	arm-none-eabi-gcc -o "stm32f103rbt_adc.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Moh Hisyam Faiz\STM32CubeIDE\workspace_1.3.0\stm32f103rbt_adc\STM32F103RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="stm32f103rbt_adc.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

stm32f103rbt_adc.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "stm32f103rbt_adc.list"
	@echo 'Finished building: $@'
	@echo ' '

stm32f103rbt_adc.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "stm32f103rbt_adc.hex"
	@echo 'Finished building: $@'
	@echo ' '

stm32f103rbt_adc.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "stm32f103rbt_adc.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
