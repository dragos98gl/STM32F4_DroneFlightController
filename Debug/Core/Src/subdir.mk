################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/freertos.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

CPP_SRCS += \
../Core/Src/MadgwickAHRS.cpp \
../Core/Src/main.cpp 

C_DEPS += \
./Core/Src/freertos.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/MadgwickAHRS.o \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

CPP_DEPS += \
./Core/Src/MadgwickAHRS.d \
./Core/Src/main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DSTM32F405xx -DUSE_HAL_DRIVER -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/BMP390_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Interfaces" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/FrSkyRX_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/HC05_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/ICM42688P_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/LIS3MDLTR_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/MB1043_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/PMW3901UY_lib" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Utils" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Buzzer_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/BatteryManagement_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation/tasks" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation/ISRs" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/nvm" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/VL53L0X_lib" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F405xx -DUSE_HAL_DRIVER -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/BMP390_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Interfaces" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/FrSkyRX_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/HC05_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/ICM42688P_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/LIS3MDLTR_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/MB1043_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/PMW3901UY_lib" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Utils" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Buzzer_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/BatteryManagement_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation/tasks" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation/ISRs" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/nvm" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/VL53L0X_lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MadgwickAHRS.d ./Core/Src/MadgwickAHRS.o ./Core/Src/MadgwickAHRS.su ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

