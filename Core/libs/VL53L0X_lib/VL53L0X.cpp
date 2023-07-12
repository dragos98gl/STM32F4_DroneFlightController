/*
 * VL53L0X.cpp
 *
 *  Created on: Jun 20, 2023
 *      Author: Dragos
 */

#include "VL53L0X.hpp"

VL53L0X::VL53L0X(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma, uint8_t timeout):
	mpc {}
{
	VL53L0X::uart_port = uart_port;
	VL53L0X::uart_port_dma=uart_port_dma;

	setTimeoutValue(timeout);
}

void VL53L0X::begin()
{
	HAL_UART_Receive_DMA (uart_port, rx_buff, packet_length);
}

void VL53L0X::update()
{
	const bool isPacketOk = (this->rx_buff[0]==this->FIRST_BIT && this->rx_buff[1]==this->SECOND_BIT);

	if(isPacketOk)
	{
	    this->distance = rx_buff[4] << 8 | rx_buff[5];
		this->resetTimeoutCounter();

		this->mpc_out = this->mpc.predict(this->getAltitudeM(),0.15F);
	}
	else if (this->wrongDataReceived==false)
	{
		for (uint8_t iter=0;iter<this->packet_length-1U;iter++)
		{
			if ((this->rx_buff[iter]==this->FIRST_BIT) && (this->rx_buff[iter+1U]==this->SECOND_BIT))
			{
				HAL_UART_Receive_DMA (this->uart_port, this->rx_buff, this->packet_length+iter);
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

float VL53L0X::getAltitudeM(void)
{
	return static_cast<float>(this->distance)/1000.0F;
}

float VL53L0X::getMPCout(void)
{
	return this->mpc_out;
}

float VL53L0X::getAltitudeCM(void)
{
	return static_cast<float>(this->distance)/10.0F;
}

uint32_t VL53L0X::getAltitudeMM(void)
{
	return this->distance;
}

const char* VL53L0X::getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::VL53_DISTANCE)!=senorsList.end())
	{
		strcat(packet,toCharArray(distance));
		strcat(packet,",");
	}
	return packet;
}
