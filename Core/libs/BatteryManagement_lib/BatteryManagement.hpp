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
#include "HC05.hpp"

/*
	2700........4.2
	2250........3.2

	0...........0V
	450.........1V
*/
class BatteryManagement: public PrintableSensor
{
private:
	const uint16_t BATERRY_MAX_4V2 = 2850;
	const uint16_t BATERRY_MIN_3V2 = 2400;
	const uint16_t BATTERY_CONNECTED_THRESHOLD = 1500;

	const float TIM_FREQ = (1.0/2000.0)*1000.0;

	ADC_HandleTypeDef *adc_port;
	Buzzer *buzz;
	uint16_t frequency_tick;
	uint32_t tick_counter;
	uint32_t batteryVal;
	float batteryPercentage;
	float batteryVoltage;

	uint16_t msToTick(uint16_t ms);
	void toPercentage();
public:
	BatteryManagement(ADC_HandleTypeDef *adc_port,Buzzer *buzz,uint16_t frequency_ms):
		adc_port(adc_port)
		,buzz(buzz)
		,frequency_tick{0}
		,tick_counter{0}
		,batteryVal{0}
		,batteryPercentage{0}
		,batteryVoltage{0}
	{
		frequency_tick = msToTick(frequency_ms);
	}

	float getBatteryPercentage();
	void run();
	const char* getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList);
};

#endif /* LIBS_BATTERYMANAGEMENT_LIB_BATTERYMANAGEMENT_HPP_ */
