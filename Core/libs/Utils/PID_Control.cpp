/*
 * PID_Control.cpp
 *
 *  Created on: Nov 6, 2022
 *      Author: DDarie
 */

#include "PID_Control.hpp"

void PID_Control::update()
{
	this->error = this->reference - this->signal;

	this->pid_p = this->error;
	this->pid_i = this->pid_i + this->error;
	this->pid_d = -(this->signal-this->last_signal);

	this->last_signal = this->signal;

	this->pid =  this->Kp * this->pid_p + this->Ki * this->pid_i + this->Kd * this->pid_d;
}

float PID_Control::getOut()
{
	return this->pid;
}
