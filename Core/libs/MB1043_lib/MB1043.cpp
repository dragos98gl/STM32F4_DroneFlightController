/*
 * MB1043.cpp
 *
 *  Created on: Sep 29, 2022
 *      Author: DragosDarie
 */

#include "MB1043.h"
#include <stdlib.h>

MB1043::MB1043(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma,uint8_t timeout)
{
	MB1043::uart_port = uart_port;
	MB1043::uart_port_dma=uart_port_dma;

	setTimeoutValue(timeout);
}

void MB1043::begin()
{
	HAL_UARTEx_ReceiveToIdle_DMA(uart_port, rx_buff, buff_len);
}

void MB1043::update()
{
	if (rx_buff[0]==BEGIN_BIT && rx_buff[5]==END_BIT)
	{
		distance_str[0]=rx_buff[1];
		distance_str[1]=rx_buff[2],
		distance_str[2]=rx_buff[3],
		distance_str[3]=rx_buff[4];

		distance = atoi(distance_str);

		resetTimeoutCounter();

		HAL_UARTEx_ReceiveToIdle_DMA(uart_port, rx_buff, buff_len);
		__HAL_DMA_DISABLE_IT(uart_port_dma, DMA_IT_HT);
	}
}

const char* MB1043::getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::SONAR_DISTANCE)!=senorsList.end())
	{
		strcat(packet,toCharArray(distance));
		strcat(packet,",");
	}

	return packet;
}

char* MB1043::getDistance_str()
{
	return distance_str;
}

uint8_t MB1043::getDistance()
{
	return distance;
}

uint8_t* MB1043::getTimeoutCounter()
{

}
