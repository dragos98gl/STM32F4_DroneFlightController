/*
 * Buzzer.cpp
 *
 *  Created on: Nov 8, 2022
 *      Author: DDarie
 */

#include "Buzzer.hpp"

bool Buzzer::beep(int duration_ms,int freq_ms,int repetition_count,int pause_ms)
{
	if (!busy)
	{
		busy = true;

		pause_tick = msToTick(pause_ms);
		counter_end = msToTick((duration_ms+pause_ms)*(repetition_count));
		freq_tick = msToTick(freq_ms);

		Buzz_on();

		return busy;
	}

	return busy;
}

void Buzzer::run()
{
	if (busy)
	{
		counter_start++;

		if (!((counter_start/pause_tick)%2))
		{
			if (freq_tick!=0)
			{
				if ((counter_start/freq_tick)%2)
					Buzz_on();
				else
					Buzz_off();
			}
		} else
			Buzz_off();

		if (counter_start>=counter_end)
		{
			busy = false;
			counter_start=0;
			freq_tick = 0;
			Buzz_off();
		}
	}
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
#if DISABLE_ALL_BEEPS == 5
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
#endif
}

void Buzzer::Buzz_off()
{
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
}
