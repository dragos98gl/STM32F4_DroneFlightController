/*
 * MB1043.cpp
 *
 *  Created on: Sep 29, 2022
 *      Author: DragosDarie
 */

#include "MB1043.hpp"
#include <stdlib.h>

MB1043::MB1043(
		UART_HandleTypeDef* uartPort,
		DMA_HandleTypeDef* uartPortDMA,
		uint8_t timeout):
	_uartPort {uartPort}
	,_uartPortDMA {uartPortDMA}
	,_wrongDataReceived {false}
	,_rxBuff {}
	,_distanceStr {}
	,_distance {}
{
	setTimeoutValue(timeout);
}

void MB1043::begin()
{
	HAL_UART_Receive_DMA(this->_uartPort, this->_rxBuff, this->packetLength);
}

void MB1043::update()
{
	const bool isPacketOk = (this->_rxBuff[0] == this->BEGIN_BIT) && (this->_rxBuff[5]==this->END_BIT);

	if (isPacketOk)
	{
		_distanceStr[0]=_rxBuff[1];
		_distanceStr[1]=_rxBuff[2],
		_distanceStr[2]=_rxBuff[3],
		_distanceStr[3]=_rxBuff[4];

		_distance = atoi(_distanceStr);

		resetTimeoutCounter();
	}
	else if (this->_wrongDataReceived==false)
	{
		for (uint8_t iter=0;iter<this->packetLength-1U;iter++)
		{
			if ((this->_rxBuff[iter]==this->END_BIT) && (this->_rxBuff[iter+1U]==this->BEGIN_BIT))
			{
				HAL_UART_Receive_DMA (this->_uartPort, this->_rxBuff, this->packetLength+iter+1);
				this->_wrongDataReceived = true;
				return;
			}
		}
	}

	if (this->_wrongDataReceived == true)
	{
		this->_wrongDataReceived = false;
	}

	HAL_UART_Receive_DMA(this->_uartPort, this->_rxBuff, this->packetLength);
	__HAL_DMA_DISABLE_IT(this->_uartPortDMA, DMA_IT_HT);
}

const char* MB1043::getSensorValues_str(std::set<SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(SENSOR_DATA_PARAMETER::SONAR_DISTANCE)!=senorsList.end())
	{
		strcat(packet,toCharArray(_distance));
		strcat(packet,",");
	}

	return packet;
}

char* MB1043::getDistance_str()
{
	return _distanceStr;
}

uint8_t MB1043::getDistance()
{
	return _distance;
}

uint8_t* MB1043::getTimeoutCounter()
{

}
