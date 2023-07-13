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
#include "failsafe_functions.hpp"

float testttt = 0;

void DynamicsProcessTask(void *pvParameters)
{
	FlightControllorImplementation *flightControllerInstance = FlightControllorImplementation::getInstance();

	const TickType_t xFrequency = 1;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		FaultsStatus currentFaultsStatus = flightControllerInstance->getCurrentFaultsStatus();

		if (currentFaultsStatus != FaultsStatus::NOT_READY)
		{
			float rollPidOutput = flightControllerInstance->getRollPidInstance().getOut();
			float pitchPidOutput = flightControllerInstance->getPitchPidInstance().getOut();
			float yawPidOutput = flightControllerInstance->getYawPidInstance().getOut();
			float xPositionPidOutput = flightControllerInstance->getXPositionPidInstance().getOut();
			float yPositionPidOutput = flightControllerInstance->getYPositionPidInstance().getOut();
			float altitudeMpcOutput = flightControllerInstance->getVL53L0Xinstance().getMPCout();
			float throttle = flightControllerInstance->getFrSkyRXinstance().throttle;

			testttt = yPositionPidOutput;

			if (currentFaultsStatus == FaultsStatus::FAILURE)
			{
				drone::failsafe::slowlyLanding(*flightControllerInstance);
			}

			if (currentFaultsStatus == FaultsStatus::CRITICAL)
			{
				drone::failsafe::quickLanding(*flightControllerInstance);
			}

			float CCR1_value = 3000.0F + throttle + altitudeMpcOutput + rollPidOutput + pitchPidOutput + yawPidOutput - xPositionPidOutput - yPositionPidOutput;
			float CCR2_value = 3000.0F + throttle + altitudeMpcOutput + rollPidOutput - pitchPidOutput - yawPidOutput - xPositionPidOutput + yPositionPidOutput;
			float CCR3_value = 3000.0F + throttle + altitudeMpcOutput - rollPidOutput + pitchPidOutput - yawPidOutput + xPositionPidOutput - yPositionPidOutput;
			float CCR4_value = 3000.0F + throttle + altitudeMpcOutput - rollPidOutput - pitchPidOutput + yawPidOutput + xPositionPidOutput + yPositionPidOutput;

			if (currentFaultsStatus == FaultsStatus::OKAY)
			{
				if (CCR1_value<3300.0F)
					CCR1_value = 3300.0F;
				if (CCR1_value>6000.0F)
					CCR1_value = 6000.0F;

				if (CCR2_value<3300.0F)
					CCR2_value = 3300.0F;
				if (CCR2_value>6000.0F)
					CCR2_value = 6000.0F;

				if (CCR3_value<3300.0F)
					CCR3_value = 3300.0F;
				if (CCR3_value>6000.0F)
					CCR3_value = 6000.0F;

				if (CCR4_value<3300.0F)
					CCR4_value = 3300.0F;
				if (CCR4_value>6000.0F)
					CCR4_value = 6000.0F;
			}

			if (currentFaultsStatus == FaultsStatus::CRITICAL)
			{
				CCR1_value = 3000.0F;
				CCR2_value = 3000.0F;
				CCR3_value = 3000.0F;
				CCR4_value = 3000.0F;
			}

			TIM3 -> CCR1 = static_cast<uint32_t>(CCR1_value);
			TIM3 -> CCR2 = static_cast<uint32_t>(CCR2_value);
			TIM3 -> CCR3 = static_cast<uint32_t>(CCR3_value);
			TIM3 -> CCR4 = static_cast<uint32_t>(CCR4_value);
		}

		vTaskDelayUntil( &xLastWakeTime, xFrequency);
	}
}

#endif /* SRC_IMPLEMENTATION_TASKS_DYNAMICPROCESSTASK_HPP_ */
