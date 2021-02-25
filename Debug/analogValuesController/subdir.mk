################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../analogValuesController/analogValuesController.c 

OBJS += \
./analogValuesController/analogValuesController.o 

C_DEPS += \
./analogValuesController/analogValuesController.d 


# Each subdirectory must supply rules for building sources it contributes
analogValuesController/analogValuesController.o: ../analogValuesController/analogValuesController.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F746xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/home/neset/stmProjects/cleaningRobotf7/analogValuesController" -I"/home/neset/stmProjects/cleaningRobotf7/buttonController" -I"/home/neset/stmProjects/cleaningRobotf7/motorDriverInterface" -I"/home/neset/stmProjects/cleaningRobotf7/lcdDriver" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"analogValuesController/analogValuesController.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

