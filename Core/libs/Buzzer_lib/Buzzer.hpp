/*
 * Buzzer.hpp
 *
 *  Created on: Nov 8, 2022
 *      Author: DDarie
 */

#ifndef LIBS_BUZZER_LIB_BUZZER_HPP_
#define LIBS_BUZZER_LIB_BUZZER_HPP_

#include "stm32f4xx_hal.h"

class Buzzer
{
private:
	const int BUZZ_PIN = GPIO_PIN_4;
	GPIO_TypeDef* BUZZ_PORT = GPIOA;
	const float TIM_FREQ = (1.0/2000.0)*1000.0;


	uint16_t freq_tick=0;
	uint16_t pause_tick=0;
	uint16_t counter_start=0;
	uint16_t counter_end=0;
	uint16_t repetitions=0;
	bool busy=false;

	void Buzz_on();
	void Buzz_off();
	uint16_t msToTick(uint16_t ms);
public:
	bool beep(int duration_ms,int freq_hz=0,int repetition_count=1,int pause_ms=0);
	bool isBusy();
	void run();
};

#endif /* LIBS_BUZZER_LIB_BUZZER_HPP_ */
