/*
 * PMW3901UY.cpp
 *
 *  Created on: Sep 26, 2022
 *      Author: Asus
 */

#include "PMW3901UY.h"

PMW3901UY::PMW3901UY(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma,uint8_t timeout)
{
	PMW3901UY::uart_port = uart_port;
	PMW3901UY::uart_port_dma=uart_port_dma;

	setTimeoutValue(timeout);
}

void PMW3901UY::begin()
{
	HAL_UARTEx_ReceiveToIdle_DMA(uart_port, rx_buff, buff_len);
}

void PMW3901UY::update()
{
	if (rx_buff[0]==BEGIN_BIT && rx_buff[1]==DATA_LEN_BIT && rx_buff[8]==END_BIT)
	{
		flow_x = (int16_t)(rx_buff[3]<<8 | rx_buff[2]);
		flow_y = (int16_t)(rx_buff[5]<<8 | rx_buff[4]);
		quality = rx_buff[7];

		x_pos += flow_x;
		y_pos += flow_y;

		resetTimeoutCounter();

		HAL_UARTEx_ReceiveToIdle_DMA(uart_port, rx_buff, buff_len);
		__HAL_DMA_DISABLE_IT(uart_port_dma, DMA_IT_HT);

		/*char s1[20];
		char s2[20];
		sprintf(s1,"%ld", x_pos);
		sprintf(s2,"%ld", y_pos);
		strcat(s1,",");
		strcat(s1,s2);
		strcat(s1,"\n\r");

		*/
		//std::string s1 = std::to_string(flow_x);
		//std::string s2 = std::to_string(flow_y);
		//s1 = s1 + "," + s2 + "\n\r";
		//int len = s1.length();
	}
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
