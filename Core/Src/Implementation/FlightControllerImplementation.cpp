/*
 * FlightControllerImplementation.cpp
 *
 *  Created on: Apr 18, 2023
 *      Author: DDarie
 */

#include "FlightControllerImplementation.hpp"

MemoryManagement& FlightControllorImplementation::getNvmInstance()
{
	return this->_nvmInstance;
}

PID_Control& FlightControllorImplementation::getRollPidInstance()
{
	return this->_rollPID;
}

PID_Control& FlightControllorImplementation::getPitchPidInstance()
{
	return this->_pitchPID;
}

PID_Control& FlightControllorImplementation::getYawPidInstance()
{
	return this->_yawPID;
}

PID_Control& FlightControllorImplementation::getXPositionPidInstance()
{
	return this->_xPositionPID;
}

PID_Control& FlightControllorImplementation::getYPositionPidInstance()
{
	return this->_yPositionPID;
}

LIS3MDLTR& FlightControllorImplementation::getLIS3MDLTRinstance()
{
	return this->_lis;
}

Buzzer& FlightControllorImplementation::getBuzzerinstance()
{
	return this->_buzz;
}

BMP390& FlightControllorImplementation::getBMP390instance()
{
	return this->_bmp;
}

ICM42688P& FlightControllorImplementation::getICM42688Pinstance()
{
	return this->_icm;
}

HC05& FlightControllorImplementation::getHC05instance()
{
	return this->_bt;
}

PMW3901UY& FlightControllorImplementation::getPMW3901UYinstance()
{
	return this->_pmw;
}

FrSkyRX& FlightControllorImplementation::getFrSkyRXinstance()
{
	return this->_remote_rx;
}

MB1043& FlightControllorImplementation::getMB1043instance()
{
	return this->_sonar;
}

VL53L0X& FlightControllorImplementation::getVL53L0Xinstance()
{
	return this->_vl53;
}

BatteryManagement& FlightControllorImplementation::getBatteryManagementinstance()
{
	return this->_battMgmt;
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
