/*
 * SensorsDataReadTask.hpp
 *
 *  Created on: Apr 27, 2023
 *      Author: DDarie
 */

#ifndef SRC_IMPLEMENTATION_TASKS_SENSORSDATAREADTASK_HPP_
#define SRC_IMPLEMENTATION_TASKS_SENSORSDATAREADTASK_HPP_

#include "FlightControllerImplementation.hpp"
#include "DynamicProcessTask.hpp"
#include "FaultsCheckTask.hpp"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

void sensorsDataReadTask(void *pvParameters)
{
	FlightControllorImplementation *flightControllerInstance = FlightControllorImplementation::getInstance();

	HAL_TIM_Base_Start_IT(&htim4);
	HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_SetPriorityGrouping(0);

	bool startup;
	if(!flightControllerInstance->getLIS3MDLTRinstance().defaultInit())
	  startup = false;
	flightControllerInstance->getLIS3MDLTRinstance().update();

	if (!flightControllerInstance->getICM42688Pinstance().defaultInit())
	  startup = false;
	flightControllerInstance->getICM42688Pinstance().update();

	if (!flightControllerInstance->getBMP390instance().defaultInit())
	  startup = false;
	flightControllerInstance->getBMP390instance().update();

	flightControllerInstance->getFrSkyRXinstance().begin();
	flightControllerInstance->getMB1043instance().begin();
	flightControllerInstance->getPMW3901UYinstance().begin();
	flightControllerInstance->getVL53L0Xinstance().begin();

	uint32_t currentSensor = 0;
	xTaskCreate(DynamicsProcessTask,"DynamicsProcessTask",2048,NULL,tskIDLE_PRIORITY+2, flightControllerInstance->getDynamicsProcessHandlerPtr());
	xTaskCreate(FaultsCheckTask,"FaultsCheckTask",1024,NULL,tskIDLE_PRIORITY+2, flightControllerInstance->getFaultsCheckHandlerPtr());

	while (1)
	{
		if (xTaskNotifyWait(0x00, 0xFFFFFFFFUL, &currentSensor, portMAX_DELAY) == pdTRUE)
		{
			if (currentSensor & EnumSensorsInterrupt::ICM42688P_t)
			{
				flightControllerInstance->getICM42688Pinstance().update();
				flightControllerInstance->getICM42688Pinstance().incrementTaskCounter();
				/*icmCouter2++;
				float testt = flightController->getICM42688Pinstance().getAccX();
				if (testt==duplicates)
					duplicatesCounter++;
				duplicates = testt;*/
			}

			if (currentSensor & EnumSensorsInterrupt::BMP390_t)
			{
				flightControllerInstance->getBMP390instance().update();
				flightControllerInstance->getBMP390instance().incrementTaskCounter();
			}

			if (currentSensor & EnumSensorsInterrupt::LIS3MDLTR_t)
			{
				flightControllerInstance->getLIS3MDLTRinstance().update();
				flightControllerInstance->getLIS3MDLTRinstance().incrementTaskCounter();
			}

			if (currentSensor & EnumSensorsInterrupt::PMW_t)
			{
				__HAL_UART_FLUSH_DRREGISTER(&huart2);
				flightControllerInstance->getPMW3901UYinstance().update();
				flightControllerInstance->getPMW3901UYinstance().incrementTaskCounter();
			}

			if (currentSensor & EnumSensorsInterrupt::REMOTERX_t)
			{
				FaultsStatus faultStatus = flightControllerInstance->getCurrentFaultsStatus();

				if (faultStatus != FaultsStatus::FAILURE && faultStatus != FaultsStatus::CRITICAL)
				{
					__HAL_UART_FLUSH_DRREGISTER(&huart3);
					flightControllerInstance->getFrSkyRXinstance().update();
					flightControllerInstance->getFrSkyRXinstance().incrementTaskCounter();
				}
			}

			if (currentSensor & EnumSensorsInterrupt::SONAR_t)
			{
			   __HAL_UART_FLUSH_DRREGISTER(&huart4);
			   flightControllerInstance->getMB1043instance().update();
			   flightControllerInstance->getMB1043instance().incrementTaskCounter();
			}

			if (currentSensor & EnumSensorsInterrupt::VL53L0X_t)
			{
			   __HAL_UART_FLUSH_DRREGISTER(&huart6);
			   flightControllerInstance->getVL53L0Xinstance().update();
			   flightControllerInstance->getVL53L0Xinstance().incrementTaskCounter();
			}

			//taskCounter++;
		}
	}
}

#endif /* SRC_IMPLEMENTATION_TASKS_SENSORSDATAREADTASK_HPP_ */
