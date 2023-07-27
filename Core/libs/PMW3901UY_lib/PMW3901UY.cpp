/*
 * PMW3901UY.cpp
 *
 *  Created on: Sep 26, 2022
 *      Author: Asus
 */

#include "PMW3901UY.hpp"

PMW3901UY::PMW3901UY(
		UART_HandleTypeDef *uart_port,
		DMA_HandleTypeDef *uart_port_dma,
		uint8_t timeout,ICM42688P& icm,
		VL53L0X& vl53,
		PID_Control& pidX,
		PID_Control& pidY):
	uart_port {uart_port}
	,uart_port_dma {uart_port_dma}
	,_icm (icm)
	,_vl53 (vl53)
	,_pidX (pidX)
	,_pidY (pidY)
	,rx_buff {}
	,wrongDataReceived {false}
	,flow_x {0}
	,flow_y {0}
	,quality {0}
	,x_pos {0}
	,y_pos {0}
	,x_cm_pos {0}
	,y_cm_pos {0}
	,target_x {0}
	,target_y {0}
	,lastAngleX {0}
	,lastAngleY {0}
{
	setTimeoutValue(timeout);
}

void PMW3901UY::begin()
{
	HAL_UART_Receive_DMA (uart_port, rx_buff, packet_length);
}

void PMW3901UY::update()
{
	const bool isPacketOk = (this->rx_buff[6]==this->END_BIT);

	if(isPacketOk)
	{
		this->flow_x = (int16_t)(this->rx_buff[3]<<8 | this->rx_buff[2]);
		this->flow_y = (int16_t)(this->rx_buff[5]<<8 | this->rx_buff[4]);
		this->quality = this->rx_buff[7];

		this->x_pos += this->flow_x;
		this->y_pos += this->flow_y;

		this->process();

		this->_pidX.update();
		this->_pidY.update();

		this->resetTimeoutCounter();
	}
	else if (this->wrongDataReceived==false)
	{
		for (uint8_t iter=0;iter<this->packet_length-1U;iter++)
		{
			if ((this->rx_buff[iter]==this->END_BIT))
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

void PMW3901UY::process()
{
	float cpi = (this->_vl53.getAltitudeM() / 11.914F) * 2.54F;
	this->x_cm_pos = this->x_cm_pos + static_cast<float>(this->flow_x)*cpi;// - this->lastAngleY * cpi * 10.0F;
	this->y_cm_pos = this->y_cm_pos + static_cast<float>(this->flow_y)*cpi;// - this->lastAngleX * cpi * 10.0F;

	lastAngleX = this->_icm.getEulerX();
	lastAngleY = this->_icm.getEulerY();
}

const char* PMW3901UY::getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::PMW_POS_X)!=senorsList.end())
	{
		strcat(packet,toCharArray(x_cm_pos));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::PMW_POS_Y)!=senorsList.end())
	{
		strcat(packet,toCharArray(y_cm_pos));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::PMW_FLOW_X)!=senorsList.end())
	{
		strcat(packet,toCharArray(flow_x));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::PMW_FLOW_Y)!=senorsList.end())
	{
		strcat(packet,toCharArray(flow_y));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::PMW_QUALITY)!=senorsList.end())
	{
		strcat(packet,toCharArray(quality));
		strcat(packet,",");
	}

	return packet;
}

uint16_t PMW3901UY::getFlowX()
{
	return flow_x;
}

uint16_t PMW3901UY::getFlowY()
{
	return flow_y;
}

uint8_t PMW3901UY::getQuality()
{
	return quality;
}

float& PMW3901UY::getXpos()
{
	return x_cm_pos;
}

float& PMW3901UY::getYpos()
{
	return y_cm_pos;
}

float& PMW3901UY::getTargetX()
{
	return target_x;
}

float& PMW3901UY::getTargetY()
{
	return target_y;
}
