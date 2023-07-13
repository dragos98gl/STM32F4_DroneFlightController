/*
 * VL53L0X.cpp
 *
 *  Created on: Jun 20, 2023
 *      Author: Dragos
 */

#include "VL53L0X.hpp"
#include "FlightControllerImplementation.hpp"

VL53L0X::VL53L0X(
		UART_HandleTypeDef *uartPort,
		DMA_HandleTypeDef *uartPortDMA,
		uint8_t timeout):
	_uartPort {uartPort}
	,_uartPortDMA {uartPortDMA}
	,_mpc {}
	,_lpf (1,0.1)
	,_mpcOut {0}
	,_rxBuff {}
	,_wrongDataReceived {false}
	,_distance {0U}
{
	setTimeoutValue(timeout);
}

void VL53L0X::begin()
{
	HAL_UART_Receive_DMA (_uartPort, _rxBuff, packetLength);
}

void VL53L0X::update()
{
	const bool isPacketOk = (this->_rxBuff[0]==this->FIRST_BIT && this->_rxBuff[1]==this->SECOND_BIT);

	if(isPacketOk)
	{
	    this->_distance = _rxBuff[4] << 8 | _rxBuff[5];
	    this->computeMPC();

		this->resetTimeoutCounter();
	}
	else if (this->_wrongDataReceived==false)
	{
		for (uint8_t iter=0;iter<this->packetLength-1U;++iter)
		{
			if ((this->_rxBuff[iter]==this->FIRST_BIT) && (this->_rxBuff[iter+1U]==this->SECOND_BIT))
			{
				HAL_UART_Receive_DMA (this->_uartPort, this->_rxBuff, this->packetLength+iter);
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

void VL53L0X::computeMPC()
{
	FlightControllorImplementation *flightControllerInstance = FlightControllorImplementation::getInstance();

	if (flightControllerInstance->getCurrentFaultsStatus() == FaultsStatus::OKAY)
	{
		this->_filteredDistance = this->_lpf.lsim(this->getAltitudeM());

		if (flightControllerInstance->getFrSkyRXinstance().getRU()==0)
		{
			altRef = -1;
		} else
		{
			altRef = 0.15;
		}

		this->_mpcOut = this->_mpc.predict(_filteredDistance,altRef);

		this->_mpcOut = this->_mpcOut-(this->_filteredDistance - this->_prevFilteredDistance)*5000;
		this->_prevFilteredDistance = _filteredDistance;

		/*this->_mpcOut = this->_mpc.predict(this->_filteredDistance,this->altRef);

		if (this->_mpcOut<0)
		{
			this->_mpcOut = 0;
		}*/
	}
}

float VL53L0X::getAltitudeM(void)
{
	return static_cast<float>(this->_distance)/1000.0F;
}

float VL53L0X::getMPCout(void)
{
	return this->_mpcOut;
}

float VL53L0X::getAltitudeCM(void)
{
	return static_cast<float>(this->_distance)/10.0F;
}

uint32_t VL53L0X::getAltitudeMM(void)
{
	return this->_distance;
}

const char* VL53L0X::getSensorValues_str(std::set<SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(SENSOR_DATA_PARAMETER::VL53_DISTANCE)!=senorsList.end())
	{
		strcat(packet,toCharArray(_distanceM));
		strcat(packet,",");
		strcat(packet,toCharArray(_filteredDistance));
		strcat(packet,",");
	}
	return packet;
}
