/*
 * utils_functions.hpp
 *
 *  Created on: Apr 11, 2023
 *      Author: DDarie
 */

#ifndef LIBS_UTILS_UTILS_FUNCTIONS_HPP_
#define LIBS_UTILS_UTILS_FUNCTIONS_HPP_

#include <cstdint>

namespace drone
{
namespace utils
{
	/*namespace constants
	{
		static constexpr float angle_to_rad = 0.0174F;
	}*/

	namespace functions
	{
		inline int16_t concatBytes(uint8_t msb, uint8_t lsb)
		{
			return (((int16_t)msb << 8) | (int16_t)lsb);
		}

		inline float HzToRads(float val)
		{
		  return val * 2 * M_PI;
		}

		inline float RadsToHz(float val)
		{
		  return val / 2 / M_PI;
		}
	}
}
}

#endif /* LIBS_UTILS_UTILS_FUNCTIONS_HPP_ */
