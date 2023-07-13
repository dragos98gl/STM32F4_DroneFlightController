/*
 * FrSkyRX.cpp
 *
 *  Created on: Sep 29, 2022
 *      Author: DragosDarie
 */

#include "FrSkyRX.hpp"

FrSkyRX::FrSkyRX(
		UART_HandleTypeDef *uartPort,
		DMA_HandleTypeDef *uartPortDMA,
		Buzzer *buzz,uint8_t timeout):
	_uartPort {uartPort}
	,_uartPortDMA {uartPortDMA}
	,_buzz {buzz}
	,_rxBuff {}
	,_wrongDataReceived {false}
	,_channels {}
	,_lb {0U}
	,_lu {0U}
	,_rb {0U}
	,_ru {0U}
	,_rawRoll {0U}
	,_rawPitch {0U}
	,_rawYaw {0U}
	,_rxOk {0U}
	,_currentState {FrSkyRXState::NOT_CONNECTED}
	,throttle {0.0F}
	,target_roll {0.0F}
	,target_pitch {0.0F}
	,target_yaw {0.0F}
{
	setTimeoutValue(timeout);
}

void FrSkyRX::begin()
{
	HAL_UART_Receive_DMA(this->_uartPort, this->_rxBuff, this->packetLength);
}

void FrSkyRX::update()
{
	const bool isPacketOk = (this->_rxBuff[0] == this->BEGIN_BIT) && (this->_rxBuff[24]==this->END_BIT);

	if(isPacketOk)
	{
		this->updateValues();
		this->processStateMachine();

		if (this->isDisconnected() || this->_rb == 0U)
			this->_currentState = FrSkyRXState::TIMEOUT;
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

void FrSkyRX::processStateMachine()
{
	switch (this->_currentState)
	{
	case FrSkyRXState::NOT_CONNECTED:
		if ((this->_lu == 0U) && (this->throttle < 300U) && (this->_rb == 1U))
		{
			this->_currentState = FrSkyRXState::CONNECTED;
			this->_buzz->stop();
			this->_buzz->beep(600U,100U,3U);
		}
		else
		{
			this->_buzz->beep(3000U,100U,2U);
		}
		break;
	case FrSkyRXState::CONNECTED:
		if (this->_lu == 1U)
		{
			TIM3 -> CCR1 = 3000;
			TIM3 -> CCR2 = 3000;
			TIM3 -> CCR3 = 3000;
			TIM3 -> CCR4 = 3000;
			this->_currentState = FrSkyRXState::ARMED;
		}
		break;
	case FrSkyRXState::ARMED:
		if (this->_lu == 2U)
		{
			TIM3 -> CCR1 = 3300;
			TIM3 -> CCR2 = 3300;
			TIM3 -> CCR3 = 3300;
			TIM3 -> CCR4 = 3300;
			this->_buzz->beep(200U,100U,1U);
			this->_currentState = FrSkyRXState::READY;
		}
		break;
	case FrSkyRXState::READY:
		if (_rawRoll > 1150 || _rawRoll < 800 || _rawPitch > 1150 || _rawPitch < 800)
		{
			target_roll += -static_cast<float>(mid_position - _rawRoll) * roll_scaleFactor;
			target_pitch += static_cast<float>(mid_position - _rawPitch) * pitch_scaleFactor;
			target_yaw = static_cast<float>(mid_position - _rawYaw) * yaw_scaleFactor;

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
	return this->_currentState;
}

void FrSkyRX::updateValues()
{
	this->_channels[0]  = static_cast<int16_t>(_rxBuff[1] | (_rxBuff[2] << 8 & 0x07FF));
	this->_channels[1]  = static_cast<int16_t>(_rxBuff[2] >> 3 | (_rxBuff[3] << 5 & 0x07FF));
	this->_channels[2]  = static_cast<int16_t>(_rxBuff[3] >> 6 | (_rxBuff[4] << 2 | (_rxBuff[5] << 10 & 0x07FF)));
	this->_channels[3]  = static_cast<int16_t>(_rxBuff[5] >> 1 | (_rxBuff[6] << 7 & 0x07FF));
	this->_channels[4]  = static_cast<int16_t>(_rxBuff[6] >> 4 | (_rxBuff[7] << 4 & 0x07FF));
	this->_channels[5]  = static_cast<int16_t>(_rxBuff[7] >> 7 | (_rxBuff[8] << 1 | (_rxBuff[9] << 9 & 0x07FF)));
	this->_channels[6]  = static_cast<int16_t>(_rxBuff[9] >> 2 | (_rxBuff[10] << 6 & 0x07FF));
	this->_channels[7]  = static_cast<int16_t>(_rxBuff[10] >> 5 | (_rxBuff[11] << 3 & 0x07FF));

	this->_channels[8]  = static_cast<int16_t>(_rxBuff[12] | (_rxBuff[13] << 8 & 0x07FF));
	this->_channels[9]  = static_cast<int16_t>(_rxBuff[13] >> 3 | (_rxBuff[14] << 5 & 0x07FF));
	this->_channels[10] = static_cast<int16_t>(_rxBuff[14] >> 6 | (_rxBuff[15] << 2 | (_rxBuff[16] << 10 & 0x07FF)));
	this->_channels[11] = static_cast<int16_t>(_rxBuff[16] >> 1 | (_rxBuff[17] << 7 & 0x07FF));
	this->_channels[12] = static_cast<int16_t>(_rxBuff[17] >> 4 | (_rxBuff[18] << 4 & 0x07FF));
	this->_channels[13] = static_cast<int16_t>(_rxBuff[18] >> 7 | (_rxBuff[19] << 1 | (_rxBuff[20] << 9 & 0x07FF)));
	this->_channels[14] = static_cast<int16_t>(_rxBuff[20] >> 2 | (_rxBuff[21] << 6 & 0x07FF));
	this->_channels[15] = static_cast<int16_t>(_rxBuff[21] >> 5 | (_rxBuff[22] << 3 & 0x07FF));

	this->throttle = static_cast<float>(_channels[2]);
	this->_rawRoll = this->_channels[0];
	this->_rawPitch = this->_channels[1];
	this->_rawYaw = this->_channels[3];

	if (this->_channels[4]<1000)
		this->_lb=0;
	else
		this->_lb=1;

	if (this->_channels[7]<1000)
		this->_rb=0;
	else
		this->_rb=1;

	if (this->_channels[5]<500)
		this->_lu = 0;
	else if(this->_channels[5]<1500)
		this->_lu = 1;
	else
		this->_lu = 2;

	if (this->_channels[6]<500)
		this->_ru = 0;
	else if(this->_channels[6]<1500)
		this->_ru = 1;
	else
		this->_ru = 2;
}

bool FrSkyRX::isDisconnected() const
{
	const bool failsafe =
			(this->_rb == 0) &&
			(this->_ru == 0) &&
			(this->_lb == 0) &&
			(this->_lu == 0) &&
			(this->throttle > 1800) &&
			(this->_rawRoll > 1800) &&
			(this->_rawPitch > 1800) &&
			(this->_rawYaw > 1800);

	return failsafe;
}

const char* FrSkyRX::getSensorValues_str(std::set<SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(SENSOR_DATA_PARAMETER::FRSKY_THROTTLE)!=senorsList.end())
	{
		strcat(packet,toCharArray(this->throttle));
		strcat(packet,",");
	}

	return packet;
}

uint8_t& FrSkyRX::getRU()
{
	return this->_ru;
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
	if (_rxOk == 2)
		return true;

	return false;
}

uint8_t* FrSkyRX::getBuffer_ptr()
{
	return this->_rxBuff;
}

uint8_t FrSkyRX::getFrameLength()
{
	return this->packetLength;
}
