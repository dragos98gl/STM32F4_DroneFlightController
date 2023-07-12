/*
 * nvmTypes.hpp
 *
 *  Created on: Jun 6, 2023
 *      Author: Dragos
 */

#ifndef LIBS_NVM_NVMTYPES_HPP_
#define LIBS_NVM_NVMTYPES_HPP_

static constexpr uint8_t dataSize = 4U;

union intObj {
	using Type = int;

	int value;
	uint32_t bytes;
};

union floatObj{
	using Type = float;

	float value;
	uint32_t bytes;
};

enum class MemoryDescriptor: uint8_t
{
	firstBoot,
	flashReadWriteError,
	PID_ROLL_P,
	PID_ROLL_I,
	PID_ROLL_D,
	PID_PITCH_P,
	PID_PITCH_I,
	PID_PITCH_D,
	PID_YAW_P,
	PID_YAW_I,
	PID_YAW_D
};

#endif /* LIBS_NVM_NVMTYPES_HPP_ */
