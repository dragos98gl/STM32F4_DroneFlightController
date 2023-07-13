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
	static constexpr float TIM_FREQ = (1.0/1000.0)*1000.0;
	static constexpr int BUZZ_PIN = GPIO_PIN_4;
	GPIO_TypeDef* BUZZ_PORT = GPIOA;

	void Buzz_on();
	void Buzz_off();
	uint16_t msToTick(uint16_t ms);
	uint16_t _freqTick;
	uint16_t _pauseTick;
	uint16_t _counterStart;
	uint16_t _counterEnd;
	uint16_t _repetitions;
	bool _busy;

public:
	Buzzer():
		_freqTick {0U}
		,_pauseTick {0U}
		,_counterStart {0U}
		,_counterEnd {0U}
		,_repetitions {0U}
		,_busy {false}
	{
	}

	bool beep(int duration_ms,int freq_hz=0,int repetition_count=1);
	bool isBusy();
	void run();
	void stop();
};

#endif /* LIBS_BUZZER_LIB_BUZZER_HPP_ */
