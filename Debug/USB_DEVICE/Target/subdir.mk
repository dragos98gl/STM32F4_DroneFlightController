################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_DEVICE/Target/usbd_conf.c 

C_DEPS += \
./USB_DEVICE/Target/usbd_conf.d 

OBJS += \
./USB_DEVICE/Target/usbd_conf.o 


# Each subdirectory must supply rules for building sources it contributes
USB_DEVICE/Target/%.o USB_DEVICE/Target/%.su: ../USB_DEVICE/Target/%.c USB_DEVICE/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F405xx -DUSE_HAL_DRIVER -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/BMP390_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Interfaces" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/FrSkyRX_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/HC05_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/ICM42688P_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/LIS3MDLTR_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/MB1043_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/PMW3901UY_lib" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Utils" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Buzzer_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/BatteryManagement_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation/tasks" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation/ISRs" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/nvm" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/VL53L0X_lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USB_DEVICE-2f-Target

clean-USB_DEVICE-2f-Target:
	-$(RM) ./USB_DEVICE/Target/usbd_conf.d ./USB_DEVICE/Target/usbd_conf.o ./USB_DEVICE/Target/usbd_conf.su

.PHONY: clean-USB_DEVICE-2f-Target

