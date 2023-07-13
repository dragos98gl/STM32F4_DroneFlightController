/*
 * PID_Control.h
 *
 *  Created on: Nov 6, 2022
 *      Author: DDarie
 */

#ifndef LIBS_UTILS_PID_CONTROL_HPP_
#define LIBS_UTILS_PID_CONTROL_HPP_

#include "LowPassFilter.hpp"

class PID_Control
{
private:
	float& _signal;
	float _lastSignal = 0;
	float& _reference;
	float _Kp = 0;
	float _Ki = 0;
	float _Kd = 0;
	float _error = 0;
	float _pidP = 0;
	float _pidI = 0;
	float _pidD = 0;
	float _pid = 0;
	LowPassFilter _lowPassFilter;
public:
	PID_Control(float &signal, float &reference,float Kp,float Ki,float Kd):
		_signal (signal)
		,_lastSignal (signal)
		,_reference (reference)
		,_Kp (Kp)
		,_Ki (Ki)
		,_Kd (Kd)
		,_error {0.0F}
		,_pidP {0.0F}
		,_pidI {0.0F}
		,_pidD {0.0F}
		,_pid {0.0F}
		,_lowPassFilter (80,0.001)
	{
	};

	void update();
	float getOut();
};

#endif /* LIBS_UTILS_PID_CONTROL_HPP_ */
