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

	const float TIM_FREQ = (1.0/2000.0)*1000.0;

	uint16_t frequency_tick = 0;
	uint32_t tick_counter = 0;

	uint32_t batteryVal = 0;
	float batteryPercentage = 0;
	float batteryVoltage = 0;
	Buzzer *buzz;
	ADC_HandleTypeDef *adc_port;

	uint16_t msToTick(uint16_t ms);
	void toPercentage();
public:
	BatteryManagement(ADC_HandleTypeDef *adc_port,Buzzer *buzz,uint16_t frequency_ms):
		adc_port(adc_port),buzz(buzz)
	{
		frequency_tick = msToTick(frequency_ms);
	}

	float getBatteryPercentage();
	void run();
};

#endif /* LIBS_BATTERYMANAGEMENT_LIB_BATTERYMANAGEMENT_HPP_ */
