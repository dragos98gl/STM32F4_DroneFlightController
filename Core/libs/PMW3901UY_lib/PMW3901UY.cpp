/*
 * PMW3901UY.cpp
 *
 *  Created on: Sep 26, 2022
 *      Author: Asus
 */

#include "PMW3901UY.hpp"
//#include "utils_functions.hpp"

PMW3901UY::PMW3901UY(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma,uint8_t timeout,ICM42688P& icm):
	icm(icm)
{
	PMW3901UY::uart_port = uart_port;
	PMW3901UY::uart_port_dma=uart_port_dma;

	setTimeoutValue(timeout);
}

void PMW3901UY::begin()
{
	HAL_UART_Receive_DMA (uart_port, rx_buff, packet_length);
}

void PMW3901UY::update()
{
	const bool isPacketOk = (this->rx_buff[0]==this->BEGIN_BIT && this->rx_buff[1]==this->DATA_LEN_BIT && this->rx_buff[8]==this->END_BIT);

	if(isPacketOk)
	{
		this->flow_x = (int16_t)(this->rx_buff[3]<<8 | this->rx_buff[2]);
		this->flow_y = (int16_t)(this->rx_buff[5]<<8 | this->rx_buff[4]);
		this->quality = this->rx_buff[7];

		this->x_pos += this->flow_x;
		this->y_pos += this->flow_y;

		this->process();
		this->resetTimeoutCounter();
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

void PMW3901UY::process()
{
	_mini.flow_x = static_cast<float>(this->flow_x);
	_mini.flow_x = static_cast<float>(this->flow_y);

	_mini.flow_x_i += _mini.flow_x;
	_mini.flow_y_i += _mini.flow_y;

	_pixel_flow.fix_x_i += (_mini.flow_x_i - _pixel_flow.fix_x_i) * 0.2F;
	_pixel_flow.fix_y_i += (_mini.flow_y_i - _pixel_flow.fix_y_i) * 0.2F;

	_pixel_flow.ang_x += (600.0F * tan(icm.getEulerX()*0.0174F) - _pixel_flow.ang_x) * 0.2F;
	_pixel_flow.ang_y += (600.0F * tan(icm.getEulerY()*0.0174F) - _pixel_flow.ang_y) * 0.2F;

	_pixel_flow.out_x_i = _pixel_flow.fix_x_i - _pixel_flow.ang_x;
	_pixel_flow.out_x_i = _pixel_flow.fix_y_i - _pixel_flow.ang_y;

	_pixel_flow.x = (_pixel_flow.out_x_i - _pixel_flow.out_x_i_o);// / dT;
	_pixel_flow.out_x_i_o = _pixel_flow.out_x_i;
	_pixel_flow.y = (_pixel_flow.out_y_i - _pixel_flow.out_y_i_o);// / dT;
	_pixel_flow.out_y_i_o = _pixel_flow.out_y_i;

	_pixel_flow.fit_x += (_pixel_flow.x - _pixel_flow.fix_x) * 0.1;
	_pixel_flow.fit_y += (_pixel_flow.y - _pixel_flow.fix_y) * 0.1;
}

const char* PMW3901UY::getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::PMW_POS_X)!=senorsList.end())
	{
		strcat(packet,toCharArray(x_pos));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::PMW_POS_Y)!=senorsList.end())
	{
		strcat(packet,toCharArray(y_pos));
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

uint16_t PMW3901UY::getXpos()
{
	return x_pos;
}

uint16_t PMW3901UY::getYpos()
{
	return y_pos;
}
