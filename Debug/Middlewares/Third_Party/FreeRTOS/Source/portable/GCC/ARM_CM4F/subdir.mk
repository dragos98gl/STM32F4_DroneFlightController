################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.d 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.o Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.su: ../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.c Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F405xx -DUSE_HAL_DRIVER -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/BMP390_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/Interfaces" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/FrSkyRX_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/HC05_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/ICM42688P_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/LIS3MDLTR_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/MB1043_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/PMW3901UY_lib" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/Utils" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/Buzzer_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/BatteryManagement_lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-portable-2f-GCC-2f-ARM_CM4F

clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-portable-2f-GCC-2f-ARM_CM4F:
	-$(RM) ./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.d ./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o ./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-portable-2f-GCC-2f-ARM_CM4F

