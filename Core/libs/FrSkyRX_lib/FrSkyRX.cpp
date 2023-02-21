/*
 * FrSkyRX.cpp
 *
 *  Created on: Sep 29, 2022
 *      Author: DragosDarie
 */

#include "FrSkyRX.h"

FrSkyRX::FrSkyRX(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma,Buzzer *buzz,uint8_t timeout)
{
	FrSkyRX::uart_port = uart_port;
	FrSkyRX::uart_port_dma=uart_port_dma;
	FrSkyRX::buzz=buzz;

	setTimeoutValue(timeout);
}

void FrSkyRX::begin()
{
	HAL_UARTEx_ReceiveToIdle_DMA(uart_port, rx_frame, frame_length);
}

void FrSkyRX::update()
{
    channels[0]  = static_cast<int16_t>(rx_frame[1] | rx_frame[2] << 8 & 0x07FF);
    channels[1]  = static_cast<int16_t>(rx_frame[2] >> 3 | rx_frame[3] << 5 & 0x07FF);
    channels[2]  = static_cast<int16_t>(rx_frame[3] >> 6 | rx_frame[4] << 2 | rx_frame[5] << 10 & 0x07FF);
    channels[3]  = static_cast<int16_t>(rx_frame[5] >> 1 | rx_frame[6] << 7 & 0x07FF);
    channels[4]  = static_cast<int16_t>(rx_frame[6] >> 4 | rx_frame[7] << 4 & 0x07FF);
    channels[5]  = static_cast<int16_t>(rx_frame[7] >> 7 | rx_frame[8] << 1 | rx_frame[9] << 9 & 0x07FF);
    channels[6]  = static_cast<int16_t>(rx_frame[9] >> 2 | rx_frame[10] << 6 & 0x07FF);
    channels[7]  = static_cast<int16_t>(rx_frame[10] >> 5 | rx_frame[11] << 3 & 0x07FF);

    channels[8]  = static_cast<int16_t>(rx_frame[12] | rx_frame[13] << 8 & 0x07FF);
    channels[9]  = static_cast<int16_t>(rx_frame[13] >> 3 | rx_frame[14] << 5 & 0x07FF);
    channels[10] = static_cast<int16_t>(rx_frame[14] >> 6 | rx_frame[15] << 2 | rx_frame[16] << 10 & 0x07FF);
    channels[11] = static_cast<int16_t>(rx_frame[16] >> 1 | rx_frame[17] << 7 & 0x07FF);
    channels[12] = static_cast<int16_t>(rx_frame[17] >> 4 | rx_frame[18] << 4 & 0x07FF);
    channels[13] = static_cast<int16_t>(rx_frame[18] >> 7 | rx_frame[19] << 1 | rx_frame[20] << 9 & 0x07FF);
    channels[14] = static_cast<int16_t>(rx_frame[20] >> 2 | rx_frame[21] << 6 & 0x07FF);
    channels[15] = static_cast<int16_t>(rx_frame[21] >> 5 | rx_frame[22] << 3 & 0x07FF);

	throttle = channels[2];
	roll = channels[0];
	pitch = channels[1];
	yaw = channels[3];

	if (channels[4]<1000)
		lb=0;
	else
		lb=1;

	if (channels[7]<1000)
		rb=0;
	else
		rb=1;

	if (channels[5]<500)
		lu = 0;
	else if(channels[5]<1500)
		lu = 1;
	else
		lu = 2;

	if (channels[6]<500)
		ru = 0;
	else if(channels[6]<1500)
		ru = 1;
	else
		ru = 2;

	if (rx_ok == 0 && throttle > 1500)
	{
		rx_ok = 1;
		buzz->beep(100,1,1,100);
	}

	if (rx_ok == 1 && throttle < 200)
	{
		rx_ok = 2;
		buzz->beep(100,1,3,100);
	}
	resetTimeoutCounter();

	HAL_UARTEx_ReceiveToIdle_DMA(uart_port, rx_frame, frame_length);
	__HAL_DMA_DISABLE_IT(uart_port_dma, DMA_IT_HT);
}

bool FrSkyRX::isRxOk()
{
	if (rx_ok == 2)
		return true;

	return false;
}

uint8_t* FrSkyRX::getBuffer_ptr()
{
	return rx_frame;
}

uint8_t FrSkyRX::getFrameLength()
{
	return frame_length;
}
