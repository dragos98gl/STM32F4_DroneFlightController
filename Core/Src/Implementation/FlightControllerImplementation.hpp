/*
 * Implementation.hpp
 *
 *  Created on: Apr 18, 2023
 *      Author: DDarie
 */

#ifndef SRC_IMPLEMENTATION_FLIGHTCONTROLLERIMPLEMENTATION_HPP_
#define SRC_IMPLEMENTATION_FLIGHTCONTROLLERIMPLEMENTATION_HPP_

#include "stm32f4xx_hal.h"
#include "Enums.hpp"
#include "BMP390.hpp"
#include "ICM42688P.hpp"
#include "LIS3MDLTR.hpp"
#include "PMW3901UY.hpp"
#include "HC05.hpp"
#include "MB1043.hpp"
#include "FrSkyRX.hpp"
#include "PID_Control.hpp"
#include "Buzzer.hpp"
#include "BatteryManagement.hpp"
#include <string>
#include "math.h"

class FlightControllorImplementation
{
private:
	ADC_HandleTypeDef *hadc1;
	SPI_HandleTypeDef *hspi2;
	UART_HandleTypeDef *huart1;
	UART_HandleTypeDef *huart2;
	UART_HandleTypeDef *huart3;
	UART_HandleTypeDef *huart4;
	DMA_HandleTypeDef *hdma_usart2_rx;
	DMA_HandleTypeDef *hdma_usart3_rx;
	DMA_HandleTypeDef *hdma_uart4_rx;

	Buzzer buzz;
	LIS3MDLTR lis;
	BMP390 bmp;
	ICM42688P icm;
	HC05 bt;
	PMW3901UY pmw;
	FrSkyRX remote_rx;
	MB1043 sonar;
	BatteryManagement BattMgmt;

public:
	FlightControllorImplementation (
			ADC_HandleTypeDef *hadc1,
			SPI_HandleTypeDef *hspi2,
			UART_HandleTypeDef *huart1,
			UART_HandleTypeDef *huart2,
			UART_HandleTypeDef *huart3,
			UART_HandleTypeDef *huart4,
			DMA_HandleTypeDef *hdma_usart2_rx,
			DMA_HandleTypeDef *hdma_usart3_rx,
			DMA_HandleTypeDef *hdma_uart4_rx
			)
	: hadc1 {hadc1},
	  hspi2 {hspi2},
	  huart1 {huart1},
	  huart2 {huart2},
	  huart3 {huart3},
	  huart4 {huart4},
	  hdma_usart2_rx {hdma_usart2_rx},
	  hdma_usart3_rx {hdma_usart3_rx},
	  hdma_uart4_rx {hdma_uart4_rx},
	  lis (hspi2),
	  bmp (hspi2),
	  icm (hspi2),
	  bt (huart1),
	  pmw (huart2, hdma_usart2_rx, 255U, icm),
	  remote_rx (huart3, hdma_usart3_rx, &buzz, 1),
	  sonar (huart4,hdma_uart4_rx,255U),
	  BattMgmt (hadc1,&buzz,1000U)
	{

	}

	void initialization();
};

#endif /* SRC_IMPLEMENTATION_FLIGHTCONTROLLERIMPLEMENTATION_HPP_ */
