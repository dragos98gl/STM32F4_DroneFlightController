/*
 * Buzzer.cpp
 *
 *  Created on: Nov 8, 2022
 *      Author: DDarie
 */

#include "Buzzer.hpp"

bool Buzzer::beep(int duration_ms,int freq_ms,int repetition_count)
{
	if (!this->busy)
	{
		this->busy = true;

		this->counterEnd = this->msToTick(duration_ms);
		this->freqTick = this->msToTick(freq_ms);
		this->repetitions = this->freqTick * repetition_count * 2U - 1U;

		Buzz_on();

		return busy;
	}

	return busy;
}

void Buzzer::run()
{
	if (this->busy)
	{
		this->counterStart++;

		if ((this->repetitions > 0U) && (this->freqTick!=0U))
		{
			if (!((this->counterStart/this->freqTick) % 2U))
			{
				this->Buzz_on();
			}
			else
			{
				this->Buzz_off();
			}
			this->repetitions--;
		} else
		{
			this->Buzz_off();
		}

		if (this->counterStart >= this->counterEnd)
		{
			this->stop();
		}
	}
}

void Buzzer::stop()
{
	this->busy = false;
	this->counterStart = 0U;
	this->Buzz_off();
}

bool Buzzer::isBusy()
{
	return busy;
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
