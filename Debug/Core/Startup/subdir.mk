################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f405rgtx.s 

S_DEPS += \
./Core/Startup/startup_stm32f405rgtx.d 

OBJS += \
./Core/Startup/startup_stm32f405rgtx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/BMP390_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Interfaces" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/FrSkyRX_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/HC05_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/ICM42688P_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/LIS3MDLTR_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/MB1043_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/PMW3901UY_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Utils" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/Buzzer_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/BatteryManagement_lib" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation/tasks" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/Src/Implementation/ISRs" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/nvm" -I"C:/Users/Dragos/STM32F4_DroneFlightController/Core/libs/VL53L0X_lib" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f405rgtx.d ./Core/Startup/startup_stm32f405rgtx.o

.PHONY: clean-Core-2f-Startup

