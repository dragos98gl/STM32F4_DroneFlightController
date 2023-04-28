/*
 * PID_Control.cpp
 *
 *  Created on: Nov 6, 2022
 *      Author: DDarie
 */

#include "PID_Control.hpp"

float PID_Control::out()
{
	error = reference - signal;

	pid_p = error;
	pid_i = pid_i + error;
	pid_d = -(signal-last_signal);

	last_signal = signal;

	return Kp*pid_p + Ki*pid_i + Kd*pid_d;
}
