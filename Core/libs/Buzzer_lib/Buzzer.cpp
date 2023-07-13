/*
 * Buzzer.cpp
 *
 *  Created on: Nov 8, 2022
 *      Author: DDarie
 */

#include "Buzzer.hpp"

bool Buzzer::beep(int duration_ms,int freq_ms,int repetition_count)
{
	if (!this->_busy)
	{
		this->_busy = true;

		this->_counterEnd = this->msToTick(duration_ms);
		this->_freqTick = this->msToTick(freq_ms);
		this->_repetitions = this->_freqTick * repetition_count * 2U - 1U;

		Buzz_on();

		return _busy;
	}

	return _busy;
}

void Buzzer::run()
{
	if (this->_busy)
	{
		this->_counterStart++;

		if ((this->_repetitions > 0U) && (this->_freqTick!=0U))
		{
			if (!((this->_counterStart/this->_freqTick) % 2U))
			{
				this->Buzz_on();
			}
			else
			{
				this->Buzz_off();
			}
			this->_repetitions--;
		} else
		{
			this->Buzz_off();
		}

		if (this->_counterStart >= this->_counterEnd)
		{
			this->stop();
		}
	}
}

void Buzzer::stop()
{
	this->_busy = false;
	this->_counterStart = 0U;
	this->Buzz_off();
}

bool Buzzer::isBusy()
{
	return _busy;
}

uint16_t Buzzer::msToTick(uint16_t ms)
{
	return ms/TIM_FREQ;
}

void Buzzer::Buzz_on()
{
//#if DISABLE_ALL_BEEPS == 5
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
//#endif
}

void Buzzer::Buzz_off()
{
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
}
