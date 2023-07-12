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
	float &signal;
	float &reference;
	float Kp = 0;
	float Ki = 0;
	float Kd = 0;

	float error = 0;
	float last_signal = 0;
	float pid_p = 0;
	float pid_i = 0;
	float pid_d = 0;
	float pid = 0;
public:
	PID_Control(float &signal, float &reference,float Kp,float Ki,float Kd):
		signal (signal)
		,reference (reference)
		,Kp (Kp)
		,Ki (Ki)
		,Kd (Kd)
	,error {0.0F}
	,last_signal {0.0F}
	,pid_p {0.0F}
	,pid_i {0.0F}
	,pid_d {0.0F}
	,pid {0.0F}
	{
		last_signal = signal;
	};

	void update();
	float getOut();
};

#endif /* LIBS_UTILS_PID_CONTROL_HPP_ */
