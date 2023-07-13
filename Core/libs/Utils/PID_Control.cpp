/*
 * PID_Control.cpp
 *
 *  Created on: Nov 6, 2022
 *      Author: DDarie
 */

#include "PID_Control.hpp"

void PID_Control::update()
{
	this->_error = this->_reference - this->_signal;

	this->_pidP = this->_error;
	this->_pidI = this->_pidI + this->_error;
	this->_pidD = -(this->_signal-this->_lastSignal);

	this->_pidD = this->_lowPassFilter.lsim(this->_pidD);

	this->_lastSignal = this->_signal;

	this->_pid =  this->_Kp * this->_pidP + this->_Ki * this->_pidI + this->_Kd * this->_pidD;
}

float PID_Control::getOut()
{
	return this->_pid;
}
