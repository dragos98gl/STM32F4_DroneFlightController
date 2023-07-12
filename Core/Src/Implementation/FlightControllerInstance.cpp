/*
 * FlightControllerInstance.cpp
 *
 *  Created on: Apr 24, 2023
 *      Author: DDarie
 */

#include "FlightControllerImplementation.hpp"

extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

static FlightControllorImplementation *globalflightController = new FlightControllorImplementation(
		&hadc1,
		&hspi2,
		&huart1,
		&huart2,
		&huart3,
		&huart4,
		&huart6,
		&hdma_usart2_rx,
		&hdma_usart3_rx,
		&hdma_uart4_rx,
		&hdma_usart6_rx);

FlightControllorImplementation *FlightControllorImplementation::getInstance()
{
	return globalflightController;
}
