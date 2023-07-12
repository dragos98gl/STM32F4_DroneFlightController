/*
 * failsafe_functions.hpp
 *
 *  Created on: Apr 16, 2023
 *      Author: DDarie
 */

#ifndef LIBS_UTILS_FAILSAFE_FUNCTIONS_HPP_
#define LIBS_UTILS_FAILSAFE_FUNCTIONS_HPP_

#include "stm32f4xx_hal.h"
#include "Constants.hpp"

namespace drone
{
	namespace failsafe
	{
		inline void slowlyLanding(FlightControllorImplementation& flightControllerInstance)
		{
			static float failSafeDownStartingThrottleValue = constFailSafeDownStartingThrottleValue;

			flightControllerInstance.getFrSkyRXinstance().getThrottle() = failSafeDownStartingThrottleValue;
			flightControllerInstance.getFrSkyRXinstance().getTargetRoll() = 0.0F;
			flightControllerInstance.getFrSkyRXinstance().getTargetPitch() = 0.0F;

			failSafeDownStartingThrottleValue -= 0.1F;

			if (failSafeDownStartingThrottleValue <= 0.0F)
			{
				flightControllerInstance.setCurrentFaultsStatus(FaultsStatus::CRITICAL);
			}
		}

		inline void quickLanding(FlightControllorImplementation& flightControllerInstance)
		{
			flightControllerInstance.getFrSkyRXinstance().getThrottle() = 0.0F;
			flightControllerInstance.getFrSkyRXinstance().getTargetRoll() = 0.0F;
			flightControllerInstance.getFrSkyRXinstance().getTargetPitch() = 0.0F;
		}
	}
}

#endif /* LIBS_UTILS_FAILSAFE_FUNCTIONS_HPP_ */
