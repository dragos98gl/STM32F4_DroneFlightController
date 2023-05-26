/*
 * DynamicProcessTask.hpp
 *
 *  Created on: Apr 27, 2023
 *      Author: DDarie
 */
/*
	CR1    CR3
	   \  /
		\/
		/\
	   /  \
	CR2    CR4
 */

#ifndef SRC_IMPLEMENTATION_TASKS_DYNAMICPROCESSTASK_HPP_
#define SRC_IMPLEMENTATION_TASKS_DYNAMICPROCESSTASK_HPP_

#include "FlightControllerImplementation.hpp"

void DynamicsProcessTask(void *pvParameters)
{
	FlightControllorImplementation *flightControllerInstance = FlightControllorImplementation::getInstance();

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1;

	xLastWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		FaultsStatus currentFaultsStatus = flightControllerInstance->getCurrentFaultsStatus();
		float rollPidOutput = flightControllerInstance->getRollPidInstance().out();
		float pitchPidOutput = flightControllerInstance->getPitchPidInstance().out();
		float yawPidOutput = flightControllerInstance->getYawPidInstance().out();

		//timCounter++;

		if (currentFaultsStatus == FaultsStatus::OKAY)
		{

			float CCR1_value = 3000 + flightControllerInstance->getFrSkyRXinstance().throttle + rollPidOutput + pitchPidOutput - yawPidOutput;
			float CCR2_value = 3000 + flightControllerInstance->getFrSkyRXinstance().throttle + rollPidOutput - pitchPidOutput + yawPidOutput;
			float CCR3_value = 3000 + flightControllerInstance->getFrSkyRXinstance().throttle - rollPidOutput + pitchPidOutput + yawPidOutput;
			float CCR4_value = 3000 + flightControllerInstance->getFrSkyRXinstance().throttle - rollPidOutput - pitchPidOutput - yawPidOutput;

			if (CCR1_value<3300)
				TIM3 -> CCR1 = 3300;
			else
				TIM3 -> CCR1 = CCR1_value;

			if (CCR2_value<3300)
				TIM3 -> CCR2 = 3300;
			else
				TIM3 -> CCR2 = CCR2_value;

			if (CCR3_value<3300)
				TIM3 -> CCR3 = 3300;
			else
				TIM3 -> CCR3 = CCR3_value;

			if (CCR4_value<3300)
				TIM3 -> CCR4 = 3300;
			else
				TIM3 -> CCR4 = CCR4_value;
		}

		if (currentFaultsStatus == FaultsStatus::FAILURE)
		{

		}

		if (currentFaultsStatus == FaultsStatus::CRITICAL)
		{

		}


		vTaskDelayUntil( &xLastWakeTime, xFrequency);
	}
}

#endif /* SRC_IMPLEMENTATION_TASKS_DYNAMICPROCESSTASK_HPP_ */
