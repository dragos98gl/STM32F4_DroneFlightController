/*
 * PMW3901UY.cpp
 *
 *  Created on: Sep 26, 2022
 *      Author: Asus
 */

#include "PMW3901UY.hpp"

PMW3901UY::PMW3901UY(
		UART_HandleTypeDef *uartPort,
		DMA_HandleTypeDef *uartPortDMA,
		uint8_t timeout,ICM42688P& icm,
		VL53L0X& vl53,
		PID_Control& pidX,
		PID_Control& pidY):
	_uartPort {uartPort}
	,_uartPortDMA {uartPortDMA}
	,_icm (icm)
	,_vl53 (vl53)
	,_pidX (pidX)
	,_pidY (pidY)
	,_rxBuff {}
	,_wrongDataReceived {false}
	,_flowX {0}
	,_flowY {0}
	,_quality {0}
	,_xPos {0}
	,_yPos {0}
	,_xCmPos {0}
	,_yCmPos {0}
	,_targetX {0}
	,_targetY {0}
	,_lastAngleX {0}
	,_lastAngleY {0}
{
	setTimeoutValue(timeout);
}

void PMW3901UY::begin()
{
	HAL_UART_Receive_DMA (_uartPort, _rxBuff, packetLength);
}

void PMW3901UY::update()
{
	const bool isPacketOk = (this->_rxBuff[0]==this->BEGIN_BIT && this->_rxBuff[1]==this->DATA_LEN_BIT && this->_rxBuff[8]==this->END_BIT);

	if(isPacketOk)
	{
		this->_flowX = (int16_t)(this->_rxBuff[3]<<8 | this->_rxBuff[2]);
		this->_flowY = (int16_t)(this->_rxBuff[5]<<8 | this->_rxBuff[4]);
		this->_quality = this->_rxBuff[7];

		this->_xPos += this->_flowX;
		this->_yPos += this->_flowY;

		this->process();

		this->_pidX.update();
		this->_pidY.update();

		this->resetTimeoutCounter();
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
		this->_wrongDataReceived = false;

	HAL_UART_Receive_DMA(this->_uartPort, this->_rxBuff, this->packetLength);
	__HAL_DMA_DISABLE_IT(this->_uartPortDMA, DMA_IT_HT);
}

void PMW3901UY::process()
{
	float cpi = (this->_vl53.getAltitudeM() / 11.914F) * 2.54F;
	this->_xCmPos = this->_xCmPos + static_cast<float>(this->_flowX)*cpi;// - this->_lastAngleY * cpi * 10.0F;
	this->_yCmPos = this->_yCmPos + static_cast<float>(this->_flowY)*cpi;// - this->_lastAngleX * cpi * 10.0F;

	_lastAngleX = this->_icm.getEulerX();
	_lastAngleY = this->_icm.getEulerY();
}

const char* PMW3901UY::getSensorValues_str(std::set<SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(SENSOR_DATA_PARAMETER::PMW_POS_X)!=senorsList.end())
	{
		strcat(packet,toCharArray(_xCmPos));
		strcat(packet,",");
	}

	if (senorsList.find(SENSOR_DATA_PARAMETER::PMW_POS_Y)!=senorsList.end())
	{
		strcat(packet,toCharArray(_yCmPos));
		strcat(packet,",");
	}

	if (senorsList.find(SENSOR_DATA_PARAMETER::PMW_FLOW_X)!=senorsList.end())
	{
		strcat(packet,toCharArray(_flowX));
		strcat(packet,",");
	}

	if (senorsList.find(SENSOR_DATA_PARAMETER::PMW_FLOW_Y)!=senorsList.end())
	{
		strcat(packet,toCharArray(_flowY));
		strcat(packet,",");
	}

	if (senorsList.find(SENSOR_DATA_PARAMETER::PMW_QUALITY)!=senorsList.end())
	{
		strcat(packet,toCharArray(_quality));
		strcat(packet,",");
	}

	return packet;
}

uint16_t PMW3901UY::getFlowX()
{
	return _flowX;
}

uint16_t PMW3901UY::getFlowY()
{
	return _flowY;
}

uint8_t PMW3901UY::getQuality()
{
	return _quality;
}

float& PMW3901UY::getXpos()
{
	return _xCmPos;
}

float& PMW3901UY::getYpos()
{
	return _yCmPos;
}

float& PMW3901UY::getTargetX()
{
	return _targetX;
}

float& PMW3901UY::getTargetY()
{
	return _targetY;
}
