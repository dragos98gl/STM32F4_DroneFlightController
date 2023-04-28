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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
    	//tick1++;

    	//flightController->getHC05instance().printfSensorsValues();
    }
}

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
	TaskHandle_t* sensorsDataReadHandler = FlightControllorImplementation::getInstance()->getSensorsDataReadHandlerPtr();

	switch (GPIO_Pin)
	{
	case (GPIO_PIN_4):
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::ICM42688P_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		//icmCounter1++;
		break;

	case (GPIO_PIN_8):
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::BMP390_t, eSetBits, &pxHigherPriorityTaskWoken);
		if (pxHigherPriorityTaskWoken)
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		//bmpCounter1++;
		break;

	case (GPIO_PIN_2):
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::LIS3MDLTR_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		//lisCounter1++;
		break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
	TaskHandle_t* sensorsDataReadHandler = FlightControllorImplementation::getInstance()->getSensorsDataReadHandlerPtr();

	if (huart->Instance == USART2)
	{
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::PMW_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		//pmwCounter1++;
	} else if (huart->Instance == USART3)
	{
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::REMOTERX_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		//remoteCounter1++;
	} else if (huart->Instance == UART4)
	{
		xTaskNotifyFromISR(*sensorsDataReadHandler, EnumSensorsInterrupt::SONAR_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		//sonarCounter1++;
	}
}

#endif /* SRC_IMPLEMENTATION_ISRS_ISRS_HPP_ */
