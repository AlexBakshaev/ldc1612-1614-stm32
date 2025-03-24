################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LDC1612/Src/ldc1612_1614.c 

OBJS += \
./LDC1612/Src/ldc1612_1614.o 

C_DEPS += \
./LDC1612/Src/ldc1612_1614.d 


# Each subdirectory must supply rules for building sources it contributes
LDC1612/Src/%.o LDC1612/Src/%.su LDC1612/Src/%.cyclo: ../LDC1612/Src/%.c LDC1612/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L072xx -c -I../LDC1612/Inc -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-LDC1612-2f-Src

clean-LDC1612-2f-Src:
	-$(RM) ./LDC1612/Src/ldc1612_1614.cyclo ./LDC1612/Src/ldc1612_1614.d ./LDC1612/Src/ldc1612_1614.o ./LDC1612/Src/ldc1612_1614.su

.PHONY: clean-LDC1612-2f-Src

