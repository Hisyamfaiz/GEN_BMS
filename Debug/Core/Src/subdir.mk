################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Battery_Charge_Discharge.c \
../Core/Src/CANbus.c \
../Core/Src/adc.c \
../Core/Src/can.c \
../Core/Src/dma.c \
../Core/Src/eeprom.c \
../Core/Src/fonts.c \
../Core/Src/fram.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/iwdg.c \
../Core/Src/main.c \
../Core/Src/powermeter_ade7880.c \
../Core/Src/spi.c \
../Core/Src/ssd1306.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/sys.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/Battery_Charge_Discharge.o \
./Core/Src/CANbus.o \
./Core/Src/adc.o \
./Core/Src/can.o \
./Core/Src/dma.o \
./Core/Src/eeprom.o \
./Core/Src/fonts.o \
./Core/Src/fram.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/iwdg.o \
./Core/Src/main.o \
./Core/Src/powermeter_ade7880.o \
./Core/Src/spi.o \
./Core/Src/ssd1306.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/sys.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/Battery_Charge_Discharge.d \
./Core/Src/CANbus.d \
./Core/Src/adc.d \
./Core/Src/can.d \
./Core/Src/dma.d \
./Core/Src/eeprom.d \
./Core/Src/fonts.d \
./Core/Src/fram.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/iwdg.d \
./Core/Src/main.d \
./Core/Src/powermeter_ade7880.d \
./Core/Src/spi.d \
./Core/Src/ssd1306.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/sys.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Battery_Charge_Discharge.o: ../Core/Src/Battery_Charge_Discharge.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/Battery_Charge_Discharge.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/CANbus.o: ../Core/Src/CANbus.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/CANbus.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/adc.o: ../Core/Src/adc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/adc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/can.o: ../Core/Src/can.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/can.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/dma.o: ../Core/Src/dma.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/dma.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/eeprom.o: ../Core/Src/eeprom.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/eeprom.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/fonts.o: ../Core/Src/fonts.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/fonts.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/fram.o: ../Core/Src/fram.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/fram.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/gpio.o: ../Core/Src/gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/gpio.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/i2c.o: ../Core/Src/i2c.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/i2c.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/iwdg.o: ../Core/Src/iwdg.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/iwdg.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/main.o: ../Core/Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/powermeter_ade7880.o: ../Core/Src/powermeter_ade7880.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/powermeter_ade7880.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/spi.o: ../Core/Src/spi.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/spi.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/ssd1306.o: ../Core/Src/ssd1306.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/ssd1306.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/stm32f1xx_hal_msp.o: ../Core/Src/stm32f1xx_hal_msp.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f1xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/stm32f1xx_it.o: ../Core/Src/stm32f1xx_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f1xx_it.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/sys.o: ../Core/Src/sys.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sys.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/syscalls.o: ../Core/Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/sysmem.o: ../Core/Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/system_stm32f1xx.o: ../Core/Src/system_stm32f1xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/system_stm32f1xx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/tim.o: ../Core/Src/tim.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/tim.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/usart.o: ../Core/Src/usart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"C:/Users/Moh Hisyam Faiz/STM32CubeIDE/workspace_1.3.0/stm32f103rbt_adc/Core/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/DSP/Include" -I"C:/Users/Moh Hisyam Faiz/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/usart.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
