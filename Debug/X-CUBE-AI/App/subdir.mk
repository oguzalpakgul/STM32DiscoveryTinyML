################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../X-CUBE-AI/App/gesture.c \
../X-CUBE-AI/App/gesture_data.c 

OBJS += \
./X-CUBE-AI/App/gesture.o \
./X-CUBE-AI/App/gesture_data.o 

C_DEPS += \
./X-CUBE-AI/App/gesture.d \
./X-CUBE-AI/App/gesture_data.d 


# Each subdirectory must supply rules for building sources it contributes
X-CUBE-AI/App/%.o: ../X-CUBE-AI/App/%.c X-CUBE-AI/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/AI/Inc -I../X-CUBE-AI/App -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-X-2d-CUBE-2d-AI-2f-App

clean-X-2d-CUBE-2d-AI-2f-App:
	-$(RM) ./X-CUBE-AI/App/gesture.d ./X-CUBE-AI/App/gesture.o ./X-CUBE-AI/App/gesture_data.d ./X-CUBE-AI/App/gesture_data.o

.PHONY: clean-X-2d-CUBE-2d-AI-2f-App

