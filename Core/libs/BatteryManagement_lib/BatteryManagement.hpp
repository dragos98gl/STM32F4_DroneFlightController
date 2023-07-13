/*
 * BaterryManagement.hpp
 *
 *  Created on: Nov 9, 2022
 *      Author: DDarie
 */

#ifndef LIBS_BATTERYMANAGEMENT_LIB_BATTERYMANAGEMENT_HPP_
#define LIBS_BATTERYMANAGEMENT_LIB_BATTERYMANAGEMENT_HPP_

#include "Buzzer.hpp"
#include "stm32f4xx_hal.h"
#include "math.h"

/*
	2700........4.2
	2250........3.2

	0...........0V
	450.........1V
*/
class BatteryManagement
{
private:
	const uint16_t BATERRY_MAX_4V2 = 2850;
	const uint16_t BATERRY_MIN_3V2 = 2400;
	const uint16_t BATTERY_CONNECTED_THRESHOLD = 1500;

	const float TIM_FREQ = (1.0/2000.0)*1000.0;

	ADC_HandleTypeDef* _adcPort;
	Buzzer* _buzz;
	uint16_t _frequencyTick;
	uint32_t _tickCounter;
	uint32_t _batteryVal;
	float _batteryPercentage;
	float _batteryVoltage;

	uint16_t msToTick(uint16_t ms);
	void toPercentage();
public:
	BatteryManagement(ADC_HandleTypeDef *adc_port,Buzzer *buzz,uint16_t frequency_ms):
		_adcPort(adc_port)
		,_buzz(buzz)
		,_frequencyTick{0}
		,_tickCounter{0}
		,_batteryVal{0}
		,_batteryPercentage{0}
		,_batteryVoltage{0}
	{
		_frequencyTick = msToTick(frequency_ms);
	}

	float getBatteryPercentage();
	void run();
};

#endif /* LIBS_BATTERYMANAGEMENT_LIB_BATTERYMANAGEMENT_HPP_ */
