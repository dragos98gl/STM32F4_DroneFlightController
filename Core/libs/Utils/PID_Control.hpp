/*
 * PID_Control.h
 *
 *  Created on: Nov 6, 2022
 *      Author: DDarie
 */

#ifndef LIBS_UTILS_PID_CONTROL_HPP_
#define LIBS_UTILS_PID_CONTROL_HPP_

class PID_Control
{
private:
	float Kp = 0;
	float Ki = 0;
	float Kd = 0;
	float pid_p = 0;
	float pid_i = 0;
	float pid_d = 0;

	float reference = 0;
	float error = 0;
	float &signal;
	float last_signal = 0;
public:
	PID_Control(float &signal,float Kp,float Ki,float Kd):
		signal(signal),Kp(Kp),Ki(Ki),Kd(Kd)
	{
		last_signal = signal;
	};

	float out();
};

#endif /* LIBS_UTILS_PID_CONTROL_HPP_ */
