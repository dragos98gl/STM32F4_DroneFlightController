/*
 * FaultsCheckTask.hpp
 *
 *  Created on: Apr 27, 2023
 *      Author: DDarie
 */

#ifndef SRC_IMPLEMENTATION_TASKS_FAULTSCHECKTASK_HPP_
#define SRC_IMPLEMENTATION_TASKS_FAULTSCHECKTASK_HPP_

#include "FlightControllerImplementation.hpp"

void FaultsCheckTask(void *pvParameters)
{
	FlightControllorImplementation *flightControllerInstance = FlightControllorImplementation::getInstance();

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1;

	xLastWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		flightControllerInstance->getPMW3901UYinstance().incrementTimeoutCounter();
		flightControllerInstance->getFrSkyRXinstance().incrementTimeoutCounter();
		flightControllerInstance->getMB1043instance().incrementTimeoutCounter();

		flightControllerInstance->getBuzzerinstance().run();
		flightControllerInstance->getBatteryManagementinstance().run();

		if (flightControllerInstance->getFrSkyRXinstance().getCurrentState() == FrSkyRXState::READY)
		{
			flightControllerInstance->setCurrentFaultsStatus(FaultsStatus::OKAY);
		}

		vTaskDelayUntil( &xLastWakeTime, xFrequency);
	}
}


#endif /* SRC_IMPLEMENTATION_TASKS_FAULTSCHECKTASK_HPP_ */
