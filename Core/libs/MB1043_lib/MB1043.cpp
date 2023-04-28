/*
 * MB1043.cpp
 *
 *  Created on: Sep 29, 2022
 *      Author: DragosDarie
 */

#include "MB1043.hpp"
#include <stdlib.h>

MB1043::MB1043(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma,uint8_t timeout)
{
	MB1043::uart_port = uart_port;
	MB1043::uart_port_dma=uart_port_dma;

	setTimeoutValue(timeout);
}

void MB1043::begin()
{
	HAL_UART_Receive_DMA(this->uart_port, this->rx_buff, this->packet_length);
}

void MB1043::update()
{
	const bool isPacketOk = (this->rx_buff[0] == this->BEGIN_BIT) && (this->rx_buff[5]==this->END_BIT);

	if (isPacketOk)
	{
		distance_str[0]=rx_buff[1];
		distance_str[1]=rx_buff[2],
		distance_str[2]=rx_buff[3],
		distance_str[3]=rx_buff[4];

		distance = atoi(distance_str);

		resetTimeoutCounter();
	}
	else if (this->wrongDataReceived==false)
	{
		for (uint8_t iter=0;iter<this->packet_length-1U;iter++)
		{
			if ((this->rx_buff[iter]==this->END_BIT) && (this->rx_buff[iter+1U]==this->BEGIN_BIT))
			{
				HAL_UART_Receive_DMA (this->uart_port, this->rx_buff, this->packet_length+iter+1);
				this->wrongDataReceived = true;
				return;
			}
		}
	}

	if (this->wrongDataReceived == true)
		this->wrongDataReceived = false;

	HAL_UART_Receive_DMA(this->uart_port, this->rx_buff, this->packet_length);
	__HAL_DMA_DISABLE_IT(this->uart_port_dma, DMA_IT_HT);
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
