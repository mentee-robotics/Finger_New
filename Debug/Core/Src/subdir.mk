################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/calibration.c \
../Core/Src/control.c \
../Core/Src/dma.c \
../Core/Src/encoder.c \
../Core/Src/fdcan.c \
../Core/Src/fsm.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/max22200_driver.c \
../Core/Src/motor.c \
../Core/Src/spi.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/tim.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/calibration.o \
./Core/Src/control.o \
./Core/Src/dma.o \
./Core/Src/encoder.o \
./Core/Src/fdcan.o \
./Core/Src/fsm.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/max22200_driver.o \
./Core/Src/motor.o \
./Core/Src/spi.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/tim.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/calibration.d \
./Core/Src/control.d \
./Core/Src/dma.d \
./Core/Src/encoder.d \
./Core/Src/fdcan.d \
./Core/Src/fsm.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/max22200_driver.d \
./Core/Src/motor.d \
./Core/Src/spi.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/tim.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/calibration.cyclo ./Core/Src/calibration.d ./Core/Src/calibration.o ./Core/Src/calibration.su ./Core/Src/control.cyclo ./Core/Src/control.d ./Core/Src/control.o ./Core/Src/control.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/fdcan.cyclo ./Core/Src/fdcan.d ./Core/Src/fdcan.o ./Core/Src/fdcan.su ./Core/Src/fsm.cyclo ./Core/Src/fsm.d ./Core/Src/fsm.o ./Core/Src/fsm.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/max22200_driver.cyclo ./Core/Src/max22200_driver.d ./Core/Src/max22200_driver.o ./Core/Src/max22200_driver.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su

.PHONY: clean-Core-2f-Src

