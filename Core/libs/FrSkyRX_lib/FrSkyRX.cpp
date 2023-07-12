/*
 * FrSkyRX.cpp
 *
 *  Created on: Sep 29, 2022
 *      Author: DragosDarie
 */

#include "FrSkyRX.hpp"

FrSkyRX::FrSkyRX(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma,Buzzer *buzz,uint8_t timeout):
	currentState {FrSkyRXState::NOT_CONNECTED}
{
	FrSkyRX::uart_port = uart_port;
	FrSkyRX::uart_port_dma=uart_port_dma;
	FrSkyRX::buzz=buzz;

	setTimeoutValue(timeout);
}

void FrSkyRX::begin()
{
	HAL_UART_Receive_DMA(this->uart_port, this->rx_buff, this->packet_length);
}

void FrSkyRX::update()
{
	const bool isPacketOk = (this->rx_buff[0] == this->BEGIN_BIT) && (this->rx_buff[24]==this->END_BIT);

	if(isPacketOk)
	{
		this->updateValues();
		this->processStateMachine();

		if (this->isDisconnected() || this->rb == 0U)
			this->currentState = FrSkyRXState::TIMEOUT;
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

void FrSkyRX::processStateMachine()
{
	switch (this->currentState)
	{
	case FrSkyRXState::NOT_CONNECTED:
		if ((this->lu == 0U) && (this->throttle < 300U) && (this->rb == 1U))
		{
			this->currentState = FrSkyRXState::CONNECTED;
			buzz->stop();
			buzz->beep(600U,100U,3U);
		}
		else
		{
			buzz->beep(3000U,100U,2U);
		}
		break;
	case FrSkyRXState::CONNECTED:
		if (this->lu == 1U)
		{
			TIM3 -> CCR1 = 3000;
			TIM3 -> CCR2 = 3000;
			TIM3 -> CCR3 = 3000;
			TIM3 -> CCR4 = 3000;
			this->currentState = FrSkyRXState::ARMED;
		}
		break;
	case FrSkyRXState::ARMED:
		if (this->lu == 2U)
		{
			TIM3 -> CCR1 = 3300;
			TIM3 -> CCR2 = 3300;
			TIM3 -> CCR3 = 3300;
			TIM3 -> CCR4 = 3300;
			buzz->beep(200U,100U,1U);
			this->currentState = FrSkyRXState::READY;
		}
		break;
	case FrSkyRXState::READY:
		if (raw_roll > 1150 || raw_roll < 800 || raw_pitch > 1150 || raw_pitch < 800)
		{
			target_roll += -static_cast<float>(mid_position - raw_roll) * roll_scaleFactor;
			target_pitch += static_cast<float>(mid_position - raw_pitch) * pitch_scaleFactor;
			target_yaw = static_cast<float>(mid_position - raw_yaw) * yaw_scaleFactor;

			//target_roll = ((target_roll > 2.0F) || (target_roll < -2.0F)) ? target_roll : 0.0F;
			//target_pitch = ((target_pitch > 2.0F) || (target_pitch < -2.0F)) ? target_pitch : 0.0F;
		}
		break;
	case FrSkyRXState::TIMEOUT:

		break;
	}
}

FrSkyRXState FrSkyRX::getCurrentState() const
{
	return this->currentState;
}

void FrSkyRX::updateValues()
{
	this->channels[0]  = static_cast<int16_t>(rx_buff[1] | (rx_buff[2] << 8 & 0x07FF));
	this->channels[1]  = static_cast<int16_t>(rx_buff[2] >> 3 | (rx_buff[3] << 5 & 0x07FF));
	this->channels[2]  = static_cast<int16_t>(rx_buff[3] >> 6 | (rx_buff[4] << 2 | (rx_buff[5] << 10 & 0x07FF)));
	this->channels[3]  = static_cast<int16_t>(rx_buff[5] >> 1 | (rx_buff[6] << 7 & 0x07FF));
	this->channels[4]  = static_cast<int16_t>(rx_buff[6] >> 4 | (rx_buff[7] << 4 & 0x07FF));
	this->channels[5]  = static_cast<int16_t>(rx_buff[7] >> 7 | (rx_buff[8] << 1 | (rx_buff[9] << 9 & 0x07FF)));
	this->channels[6]  = static_cast<int16_t>(rx_buff[9] >> 2 | (rx_buff[10] << 6 & 0x07FF));
	this->channels[7]  = static_cast<int16_t>(rx_buff[10] >> 5 | (rx_buff[11] << 3 & 0x07FF));

	this->channels[8]  = static_cast<int16_t>(rx_buff[12] | (rx_buff[13] << 8 & 0x07FF));
	this->channels[9]  = static_cast<int16_t>(rx_buff[13] >> 3 | (rx_buff[14] << 5 & 0x07FF));
	this->channels[10] = static_cast<int16_t>(rx_buff[14] >> 6 | (rx_buff[15] << 2 | (rx_buff[16] << 10 & 0x07FF)));
	this->channels[11] = static_cast<int16_t>(rx_buff[16] >> 1 | (rx_buff[17] << 7 & 0x07FF));
	this->channels[12] = static_cast<int16_t>(rx_buff[17] >> 4 | (rx_buff[18] << 4 & 0x07FF));
	this->channels[13] = static_cast<int16_t>(rx_buff[18] >> 7 | (rx_buff[19] << 1 | (rx_buff[20] << 9 & 0x07FF)));
	this->channels[14] = static_cast<int16_t>(rx_buff[20] >> 2 | (rx_buff[21] << 6 & 0x07FF));
	this->channels[15] = static_cast<int16_t>(rx_buff[21] >> 5 | (rx_buff[22] << 3 & 0x07FF));

	this->throttle = static_cast<float>(channels[2]);
	this->raw_roll = this->channels[0];
	this->raw_pitch = this->channels[1];
	this->raw_yaw = this->channels[3];

	if (this->channels[4]<1000)
		this->lb=0;
	else
		this->lb=1;

	if (this->channels[7]<1000)
		this->rb=0;
	else
		this->rb=1;

	if (this->channels[5]<500)
		this->lu = 0;
	else if(this->channels[5]<1500)
		this->lu = 1;
	else
		this->lu = 2;

	if (this->channels[6]<500)
		this->ru = 0;
	else if(this->channels[6]<1500)
		this->ru = 1;
	else
		this->ru = 2;
}

bool FrSkyRX::isDisconnected() const
{
	const bool failsafe =
			(this->rb == 0) &&
			(this->ru == 0) &&
			(this->lb == 0) &&
			(this->lu == 0) &&
			(this->throttle > 1800) &&
			(this->raw_roll > 1800) &&
			(this->raw_pitch > 1800) &&
			(this->raw_yaw > 1800);

	return failsafe;
}

const char* FrSkyRX::getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::FRSKY_THROTTLE)!=senorsList.end())
	{
		strcat(packet,toCharArray(this->throttle));
		strcat(packet,",");
	}

	return packet;
}

float& FrSkyRX::getThrottle()
{
	return this->throttle;
}

float& FrSkyRX::getTargetRoll()
{
	return this->target_roll;
}

float& FrSkyRX::getTargetPitch()
{
	return this->target_pitch;
}

float& FrSkyRX::getTargetYaw()
{
	return this->target_yaw;
}

bool FrSkyRX::isRxOk()
{
	if (rx_ok == 2)
		return true;

	return false;
}

uint8_t* FrSkyRX::getBuffer_ptr()
{
	return this->rx_buff;
}

uint8_t FrSkyRX::getFrameLength()
{
	return this->packet_length;
}
