/*
 * HC05.cpp
 *
 *  Created on: Sep 27, 2022
 *      Author: DragosDarie
 */

#include "HC05.hpp"

HC05::HC05(UART_HandleTypeDef *uart_port)
{
	HC05::uart_port = uart_port;
}

void HC05::send()
{
	HAL_UART_Transmit_DMA(uart_port, (uint8_t*)USART1_TxBuffer, strlen(USART1_TxBuffer));
}

void HC05::send(const char *data,uint8_t len)
{
	HAL_UART_Transmit_DMA(uart_port, (uint8_t*)data, len);
}

void HC05::addSensor(PrintableSensor* sen)
{
	senList.push_back(sen);
}

void HC05::printfSensorsValues()
{
	strcpy(USART1_TxBuffer,"");

	for (int i=0;i<(int)senList.size();i++)
	{
		strcat(USART1_TxBuffer,senList[i]->getSensorValues_str(senorsList));
	}

	strcat(USART1_TxBuffer,"\n\r");

	send();
}

void HC05::addSensorParameter(HC05::SENSOR_DATA_PARAMETER sen)
{
 	this->senorsList.insert(sen);
}
