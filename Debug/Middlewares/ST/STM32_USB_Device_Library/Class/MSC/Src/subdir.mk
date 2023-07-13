################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc.c \
../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_bot.c \
../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_data.c \
../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_scsi.c 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc.d \
./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_bot.d \
./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_data.d \
./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_scsi.d 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc.o \
./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_bot.o \
./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_data.o \
./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_scsi.o 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/%.o Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/%.su: ../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/%.c Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F405xx -DUSE_HAL_DRIVER -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/BMP390_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Interfaces" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/FrSkyRX_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/HC05_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/ICM42688P_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/LIS3MDLTR_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/MB1043_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/PMW3901UY_lib" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Utils" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Buzzer_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/BatteryManagement_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation/tasks" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation/ISRs" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/nvm" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/VL53L0X_lib" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-MSC-2f-Src

clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-MSC-2f-Src:
	-$(RM) ./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc.d ./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc.o ./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc.su ./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_bot.d ./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_bot.o ./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_bot.su ./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_data.d ./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_data.o ./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_data.su ./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_scsi.d ./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_scsi.o ./Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_scsi.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-MSC-2f-Src

