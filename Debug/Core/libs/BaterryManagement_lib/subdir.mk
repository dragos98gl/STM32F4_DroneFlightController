################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/libs/BaterryManagement_lib/BaterryManagement.cpp 

OBJS += \
./Core/libs/BaterryManagement_lib/BaterryManagement.o 

CPP_DEPS += \
./Core/libs/BaterryManagement_lib/BaterryManagement.d 


# Each subdirectory must supply rules for building sources it contributes
Core/libs/BaterryManagement_lib/%.o Core/libs/BaterryManagement_lib/%.su: ../Core/libs/BaterryManagement_lib/%.cpp Core/libs/BaterryManagement_lib/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DSTM32F405xx -DUSE_HAL_DRIVER -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/BMP390_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/FrSkyRX_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/HC05_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/ICM42688P_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/Interfaces" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/LIS3MDLTR_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/MB1043_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/PMW3901UY_lib" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/Utils" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/Buzzer_lib" -I"C:/Users/DDarie/STM32CubeIDE/workspace_1.10.1/flightController_v3/Core/libs/BaterryManagement_lib" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-libs-2f-BaterryManagement_lib

clean-Core-2f-libs-2f-BaterryManagement_lib:
	-$(RM) ./Core/libs/BaterryManagement_lib/BaterryManagement.d ./Core/libs/BaterryManagement_lib/BaterryManagement.o ./Core/libs/BaterryManagement_lib/BaterryManagement.su

.PHONY: clean-Core-2f-libs-2f-BaterryManagement_lib

