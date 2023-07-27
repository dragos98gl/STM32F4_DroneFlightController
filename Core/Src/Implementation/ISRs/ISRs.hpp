/*
 * FlightControllerInterrupts.hpp
 *
 *  Created on: Apr 27, 2023
 *      Author: DDarie
 */

#ifndef SRC_IMPLEMENTATION_ISRS_ISRS_HPP_
#define SRC_IMPLEMENTATION_ISRS_ISRS_HPP_

#include "stm32f4xx_hal.h"
#include "FlightControllerImplementation.hpp"

int timCounter2 = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }

  if (htim->Instance == TIM4)
  {
	  //timCounter2++;
  }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
	FlightControllorImplementation *flightControllerInstance = FlightControllorImplementation::getInstance();
	TaskHandle_t* sensorsDataReadHandler = flightControllerInstance->getSensorsDataReadHandlerPtr();

	switch (GPIO_Pin)
	{
	case (GPIO_PIN_4):
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::ICM42688P_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		flightControllerInstance->getICM42688Pinstance().incrementInterruptCounter();
		break;

	case (GPIO_PIN_8):
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::BMP390_t, eSetBits, &pxHigherPriorityTaskWoken);
		if (pxHigherPriorityTaskWoken)
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		flightControllerInstance->getBMP390instance().incrementInterruptCounter();
		break;

	case (GPIO_PIN_2):
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::LIS3MDLTR_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		flightControllerInstance->getLIS3MDLTRinstance().incrementInterruptCounter();
		break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
	FlightControllorImplementation *flightControllerInstance = FlightControllorImplementation::getInstance();
	TaskHandle_t* sensorsDataReadHandler = flightControllerInstance->getSensorsDataReadHandlerPtr();

	if (huart->Instance == USART2)
	{
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::PMW_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		flightControllerInstance->getPMW3901UYinstance().incrementInterruptCounter();
	} else if (huart->Instance == USART3)
	{
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::REMOTERX_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		flightControllerInstance->getFrSkyRXinstance().incrementInterruptCounter();
	} else if (huart->Instance == UART4)
	{
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::SONAR_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		flightControllerInstance->getMB1043instance().incrementInterruptCounter();
	}else if (huart->Instance == USART6)
	{
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::VL53L0X_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		flightControllerInstance->getVL53L0Xinstance().incrementInterruptCounter();
	}
}

#endif /* SRC_IMPLEMENTATION_ISRS_ISRS_HPP_ */
