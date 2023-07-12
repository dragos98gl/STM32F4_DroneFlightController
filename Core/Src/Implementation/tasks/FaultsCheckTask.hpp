/*
 * FaultsCheckTask.hpp
 *
 *  Created on: Apr 27, 2023
 *      Author: DDarie
 */

#ifndef SRC_IMPLEMENTATION_TASKS_FAULTSCHECKTASK_HPP_
#define SRC_IMPLEMENTATION_TASKS_FAULTSCHECKTASK_HPP_

#include "FlightControllerImplementation.hpp"

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
		return true;

	return false;
}

void FaultsCheckTask(void *pvParameters)
{
	FlightControllorImplementation *flightControllerInstance = FlightControllorImplementation::getInstance();

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1;

	xLastWakeTime = xTaskGetTickCount();

	for( ;; )
	{
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
