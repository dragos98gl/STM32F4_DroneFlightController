/*
 * FlightControllerImplementation.cpp
 *
 *  Created on: Apr 18, 2023
 *      Author: DDarie
 */

#include "FlightControllerImplementation.hpp"

nvm& FlightControllorImplementation::getNvmInstance()
{
	return this->nvmInstance;
}

PID_Control& FlightControllorImplementation::getRollPidInstance()
{
	return this->roll_pid;
}

PID_Control& FlightControllorImplementation::getPitchPidInstance()
{
	return this->pitch_pid;
}

PID_Control& FlightControllorImplementation::getYawPidInstance()
{
	return this->yaw_pid;
}

LIS3MDLTR& FlightControllorImplementation::getLIS3MDLTRinstance()
{
	return this->lis;
}

Buzzer& FlightControllorImplementation::getBuzzerinstance()
{
	return this->buzz;
}

BMP390& FlightControllorImplementation::getBMP390instance()
{
	return this->bmp;
}

ICM42688P& FlightControllorImplementation::getICM42688Pinstance()
{
	return this->icm;
}

HC05& FlightControllorImplementation::getHC05instance()
{
	return this->bt;
}

PMW3901UY& FlightControllorImplementation::getPMW3901UYinstance()
{
	return this->pmw;
}

FrSkyRX& FlightControllorImplementation::getFrSkyRXinstance()
{
	return this->remote_rx;
}

MB1043& FlightControllorImplementation::getMB1043instance()
{
	return this->sonar;
}

BatteryManagement& FlightControllorImplementation::getBatteryManagementinstance()
{
	return this->battMgmt;
}

TaskHandle_t* FlightControllorImplementation::getFaultsCheckHandlerPtr()
{
	return &this->_faultsCheckHandler;
}

TaskHandle_t* FlightControllorImplementation::getSensorsDataReadHandlerPtr()
{
	return &this->_sensorsDataReadHandler;
}

TaskHandle_t* FlightControllorImplementation::getDynamicsProcessHandlerPtr()
{
	return &this->_dynamicsProcessHandler;
}

FaultsStatus FlightControllorImplementation::getCurrentFaultsStatus() const
{
	return this->_currentFaultsStatus;
}

void FlightControllorImplementation::setCurrentFaultsStatus(FaultsStatus faultsStatus)
{
	this->_currentFaultsStatus = faultsStatus;
}
