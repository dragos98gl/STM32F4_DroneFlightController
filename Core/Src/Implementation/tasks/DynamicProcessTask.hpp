/*
 * DynamicProcessTask.hpp
 *
 *  Created on: Apr 27, 2023
 *      Author: DDarie
 */

#ifndef SRC_IMPLEMENTATION_TASKS_DYNAMICPROCESSTASK_HPP_
#define SRC_IMPLEMENTATION_TASKS_DYNAMICPROCESSTASK_HPP_

#include "FlightControllerImplementation.hpp"

void DynamicsProcessTask(void *pvParameters)
{
	FlightControllorImplementation *flightControllerInstance = FlightControllorImplementation::getInstance();

	float euler_x;
	float euler_y;
	float euler_z;


	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1;

	xLastWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		FaultsStatus currentFaultsStatus = flightControllerInstance->getCurrentFaultsStatus();

		euler_x = flightControllerInstance->getICM42688Pinstance().getEulerX();
		euler_y = flightControllerInstance->getICM42688Pinstance().getEulerY();
		euler_z = flightControllerInstance->getICM42688Pinstance().getEulerZ();

		//timCounter++;

		if (currentFaultsStatus == FaultsStatus::OKAY)
		{
			/*
			CR1    CR3
			   \  /
				\/
				/\
			   /  \
			CR2    CR4
			 */
			//float CCR1_value = 3000 + flightController->getFrSkyRXinstance().throttle + roll_pid.out() + pitch_pid.out() - yaw_pid.out();
			//float CCR2_value = 3000 + flightController->getFrSkyRXinstance().throttle + roll_pid.out() - pitch_pid.out() + yaw_pid.out();
			//float CCR3_value = 3000 + flightController->getFrSkyRXinstance().throttle - roll_pid.out() + pitch_pid.out() + yaw_pid.out();
			//float CCR4_value = 3000 + flightController->getFrSkyRXinstance().throttle - roll_pid.out() - pitch_pid.out() - yaw_pid.out();

			/*if (CCR1_value<3300)
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
				TIM3 -> CCR4 = CCR4_value;*/

		// ... = base_throttle + alt_compensation + roll/pitch/yaw_pid;
		}

		if (currentFaultsStatus == FaultsStatus::FAILURE)
		{

		}

		if (currentFaultsStatus == FaultsStatus::CRITICAL)


		vTaskDelayUntil( &xLastWakeTime, xFrequency);
	}
}

#endif /* SRC_IMPLEMENTATION_TASKS_DYNAMICPROCESSTASK_HPP_ */
