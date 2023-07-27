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
	tick_counter++;
	toPercentage();

	if (tick_counter>=frequency_tick)
	{
		HAL_ADC_Start_DMA(adc_port, &batteryVal,1);

		if (batteryVal<=BATERRY_MIN_3V2 && batteryVal > BATTERY_CONNECTED_THRESHOLD)
		{
			buzz->beep(2000U,200U,1U);
		}

		tick_counter = 0;
	}
}

void BatteryManagement::toPercentage()
{
	//if(batteryVal>BATERRY_MAX_4V2)
	//{
		//batteryPercentage = 100;
		//batteryVoltage = 4.2;
	//}
	//else if(batteryVal<BATERRY_MIN_3V2)
	//{
		//batteryPercentage = 0;
		//batteryVoltage = 3.2;
	//}
	//else
	{
		batteryVoltage = 3.2+(450.0-(BATERRY_MAX_4V2-batteryVal))/(BATERRY_MAX_4V2-BATERRY_MIN_3V2);
		batteryPercentage = 123.0 - 123.0/pow((1.0 + pow(batteryVoltage/3.7,80)),0.165);
	}
}

const char* BatteryManagement::getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::SONAR_DISTANCE)!=senorsList.end())
	{
		strcat(packet,toCharArray(static_cast<int>(batteryVoltage*10000)));
		strcat(packet,",");
	}

	return packet;
}

float BatteryManagement::getBatteryPercentage()
{
	return batteryPercentage;
}

uint16_t BatteryManagement::msToTick(uint16_t ms)
{
	return ms/TIM_FREQ;
}
