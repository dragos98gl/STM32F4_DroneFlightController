/*
 * FaultsCheckTask.hpp
 *
 *  Created on: Apr 27, 2023
 *      Author: DDarie
 */

#ifndef SRC_IMPLEMENTATION_TASKS_FAULTSCHECKTASK_HPP_
#define SRC_IMPLEMENTATION_TASKS_FAULTSCHECKTASK_HPP_

#include "FlightControllerImplementation.hpp"
long int ttt = 0;
long int ttt2 = 0;
long int ttt3 = 0;
bool isFailureFaultDetected(FlightControllorImplementation& flightControllerInstance)
{
	const bool isRxDisconnected = flightControllerInstance.getFrSkyRXinstance().getCurrentState()==FrSkyRXState::TIMEOUT;
	if (isRxDisconnected)
		return true;

	return false;
}

bool isCriticalFaultDetected(FlightControllorImplementation& flightControllerInstance)
{

	const bool isCrashDetected = flightControllerInstance.getICM42688Pinstance().isCriticalStateDetected();
	if (isCrashDetected)
		return false;

	return false;
}

void FaultsCheckTask(void *pvParameters)
{
	FlightControllorImplementation *flightControllerInstance = FlightControllorImplementation::getInstance();

	const TickType_t xFrequency = 1;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		ttt++;
		ttt2++;
		ttt3++;

		if (ttt3>=200)
		{
	    	flightControllerInstance->getHC05instance().printfSensorsValues();
	    	ttt3 = 0;
		}

		FaultsStatus currentFaultsStatus = flightControllerInstance->getCurrentFaultsStatus();

		flightControllerInstance->getPMW3901UYinstance().incrementTimeoutCounter();
		flightControllerInstance->getFrSkyRXinstance().incrementTimeoutCounter();
		flightControllerInstance->getMB1043instance().incrementTimeoutCounter();

		flightControllerInstance->getBuzzerinstance().run();
		flightControllerInstance->getBatteryManagementinstance().run();

		if (isCriticalFaultDetected(*flightControllerInstance) || currentFaultsStatus==FaultsStatus::CRITICAL)
		{
			flightControllerInstance->setCurrentFaultsStatus(FaultsStatus::CRITICAL);
		}
		else if (isFailureFaultDetected(*flightControllerInstance) || currentFaultsStatus==FaultsStatus::FAILURE)
		{
			flightControllerInstance->setCurrentFaultsStatus(FaultsStatus::CRITICAL);
		}
		else if (flightControllerInstance->getFrSkyRXinstance().getCurrentState() == FrSkyRXState::READY)
		{
			flightControllerInstance->setCurrentFaultsStatus(FaultsStatus::OKAY);
		}

		vTaskDelayUntil( &xLastWakeTime, xFrequency);
	}
}


#endif /* SRC_IMPLEMENTATION_TASKS_FAULTSCHECKTASK_HPP_ */
