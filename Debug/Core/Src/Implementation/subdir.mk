################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Implementation/FlightControllerImplementation.cpp \
../Core/Src/Implementation/FlightControllerInstance.cpp 

OBJS += \
./Core/Src/Implementation/FlightControllerImplementation.o \
./Core/Src/Implementation/FlightControllerInstance.o 

CPP_DEPS += \
./Core/Src/Implementation/FlightControllerImplementation.d \
./Core/Src/Implementation/FlightControllerInstance.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Implementation/%.o Core/Src/Implementation/%.su: ../Core/Src/Implementation/%.cpp Core/Src/Implementation/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DSTM32F405xx -DUSE_HAL_DRIVER -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/libs/BMP390_lib" -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/libs/Interfaces" -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/libs/FrSkyRX_lib" -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/libs/HC05_lib" -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/libs/ICM42688P_lib" -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/libs/LIS3MDLTR_lib" -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/libs/MB1043_lib" -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/libs/PMW3901UY_lib" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/libs/Utils" -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/libs/Buzzer_lib" -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/libs/BatteryManagement_lib" -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/Src/Implementation" -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/Src/Implementation/tasks" -I"C:/Users/DDarie/STM32F4_DroneFlightController/Core/Src/Implementation/ISRs" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Implementation

clean-Core-2f-Src-2f-Implementation:
	-$(RM) ./Core/Src/Implementation/FlightControllerImplementation.d ./Core/Src/Implementation/FlightControllerImplementation.o ./Core/Src/Implementation/FlightControllerImplementation.su ./Core/Src/Implementation/FlightControllerInstance.d ./Core/Src/Implementation/FlightControllerInstance.o ./Core/Src/Implementation/FlightControllerInstance.su

.PHONY: clean-Core-2f-Src-2f-Implementation

