/*
 * BaterryManagement.cpp
 *
 *  Created on: Nov 9, 2022
 *      Author: DDarie
 */

//#include "../BatteryManagement_lib/BaterryManagement.hpp"

#include "BatteryManagement.hpp"

void BatteryManagement::run()
{
	_tickCounter++;
	toPercentage();

	if (_tickCounter>=_frequencyTick)
	{
		HAL_ADC_Start_DMA(_adcPort, &_batteryVal,1);

		if (_batteryVal<=BATERRY_MIN_3V2 && _batteryVal > BATTERY_CONNECTED_THRESHOLD)
		{
			_buzz->beep(2000U,200U,1U);
		}

		_tickCounter = 0;
	}
}

void BatteryManagement::toPercentage()
{
	if(_batteryVal>BATERRY_MAX_4V2)
	{
		_batteryPercentage = 100;
		_batteryVoltage = 4.2;
	}
	else if(_batteryVal<BATERRY_MIN_3V2)
	{
		_batteryPercentage = 0;
		_batteryVoltage = 3.2;
	}
	else
	{
		_batteryVoltage = 3.2+(450.0-(BATERRY_MAX_4V2-_batteryVoltage))/(BATERRY_MAX_4V2-BATERRY_MIN_3V2);
		_batteryPercentage = 123.0 - 123.0/pow((1.0 + pow(_batteryVoltage/3.7,80)),0.165);
	}
}

float BatteryManagement::getBatteryPercentage()
{
	return _batteryPercentage;
}

uint16_t BatteryManagement::msToTick(uint16_t ms)
{
	return ms/TIM_FREQ;
}
