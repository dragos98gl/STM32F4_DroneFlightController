/*
 * HC05.cpp
 *
 *  Created on: Sep 27, 2022
 *      Author: DragosDarie
 */

#include "HC05.hpp"

HC05::HC05(UART_HandleTypeDef *uartPort):
	_uartPort {uartPort}
	,_senorsList {}
	,_USART1TxBuffer {}
{
}

void HC05::send()
{
	HAL_UART_Transmit_DMA(_uartPort, (uint8_t*)_USART1TxBuffer, strlen(_USART1TxBuffer));
}

void HC05::send(const char *data,uint8_t len)
{
	HAL_UART_Transmit_DMA(_uartPort, (uint8_t*)data, len);
}

void HC05::addSensor(PrintableSensor* sen)
{
	_senList.push_back(sen);
}

void HC05::printfSensorsValues()
{
	strcpy(_USART1TxBuffer,"");

	for (int i=0;i<(int)_senList.size();i++)
	{
		strcat(_USART1TxBuffer,_senList[i]->getSensorValues_str(_senorsList));
	}

	strcat(_USART1TxBuffer,"\n\r");

	send();
}

void HC05::addSensorParameter(SENSOR_DATA_PARAMETER sen)
{
 	this->_senorsList.insert(sen);
}
