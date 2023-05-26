/*
 * flashMemoryBlock.hpp
 *
 *  Created on: May 17, 2023
 *      Author: DDarie
 */

#ifndef LIBS_NVM_FLASHMEMORYBLOCK_HPP_
#define LIBS_NVM_FLASHMEMORYBLOCK_HPP_

static constexpr uint8_t intSize = 4U;
static constexpr uint8_t floatSize = 4U;

union intObj {
	int value;
	uint8_t bytes[intSize];
};
union floatObj{
	int value;
	uint8_t bytes[floatSize];
};

template <typename unionType>
struct memoryData
{
	unionType data;
	uint32_t adress;
};

class flashMemoryBlock
{
	intObj firstBoot;
	intObj flashReadWriteError;
	floatObj PID_ROLL_P;
	floatObj PID_ROLL_P;
	floatObj PID_ROLL_P;
	floatObj PID_PITCH_P;
	floatObj PID_PITCH_P;
	floatObj PID_PITCH_P;
	floatObj PID_YAW_P;
	floatObj PID_YAW_P;
	floatObj PID_YAW_P;
};

#endif /* LIBS_NVM_FLASHMEMORYBLOCK_HPP_ */
