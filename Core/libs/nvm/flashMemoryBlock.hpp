/*
 * flashMemoryBlock.hpp
 *
 *  Created on: May 17, 2023
 *      Author: DDarie
 */

#ifndef LIBS_NVM_FLASHMEMORYBLOCK_HPP_
#define LIBS_NVM_FLASHMEMORYBLOCK_HPP_

#include "nvmUtils.hpp"
#include "nvmTypes.hpp"

template <typename unionType>
class memoryData
{
	using Type = typename unionType::Type;
public:
	memoryData(uint32_t& addr)
	{
		this->address = addr;
		addr+=dataSize;
	}

	void updateFromMemory()
	{
		this->data.bytes = readMemoryAddress<Type>(this->address);
	}

	void updateToMemory()
	{
		writeMemoryAddress<unionType>(this->address,this->data);
	}

	Type getValue()
	{
		return this->data.value;
	}

	void setValue(Type value)
	{
		this->data.value = value;
	}

private:
	unionType data;
	uint32_t address;
};

class FlashMemoryBlock
{
public:
	FlashMemoryBlock(uint32_t address):
		firstBoot{address},
		flashReadWriteError{address},
		PID_ROLL_P{address},
		PID_ROLL_I{address},
		PID_ROLL_D{address},
		PID_PITCH_P{address},
		PID_PITCH_I{address},
		PID_PITCH_D{address},
		PID_YAW_P{address},
		PID_YAW_I{address},
		PID_YAW_D{address},
		address{address}
	{

	}

	memoryData<intObj> firstBoot;
	memoryData<intObj> flashReadWriteError;
	memoryData<floatObj> PID_ROLL_P;
	memoryData<floatObj> PID_ROLL_I;
	memoryData<floatObj> PID_ROLL_D;
	memoryData<floatObj> PID_PITCH_P;
	memoryData<floatObj> PID_PITCH_I;
	memoryData<floatObj> PID_PITCH_D;
	memoryData<floatObj> PID_YAW_P;
	memoryData<floatObj> PID_YAW_I;
	memoryData<floatObj> PID_YAW_D;
	uint32_t address;
};

class IOManagement
{
public:
	void writeMemoryBlock(FlashMemoryBlock memBlock)
	{
		//massEraseSector(memBlock.address);

		memBlock.firstBoot.updateToMemory();
		memBlock.flashReadWriteError.updateToMemory();
		memBlock.PID_ROLL_P.updateToMemory();
		memBlock.PID_ROLL_I.updateToMemory();
		memBlock.PID_ROLL_D.updateToMemory();
		memBlock.PID_PITCH_P.updateToMemory();
		memBlock.PID_PITCH_I.updateToMemory();
		memBlock.PID_PITCH_D.updateToMemory();
		memBlock.PID_YAW_P.updateToMemory();
		memBlock.PID_YAW_I.updateToMemory();
		memBlock.PID_YAW_D.updateToMemory();
	}

protected:
	void copyMemoryBlockData(FlashMemoryBlock& to,FlashMemoryBlock from)
	{
		to.firstBoot.setValue(from.firstBoot.getValue());
		to.flashReadWriteError.setValue(from.flashReadWriteError.getValue());
		to.PID_ROLL_P.setValue(from.PID_ROLL_P.getValue());
		to.PID_ROLL_I.setValue(from.PID_ROLL_I.getValue());
		to.PID_ROLL_D.setValue(from.PID_ROLL_D.getValue());
		to.PID_PITCH_P.setValue(from.PID_PITCH_P.getValue());
		to.PID_PITCH_I.setValue(from.PID_PITCH_I.getValue());
		to.PID_PITCH_D.setValue(from.PID_PITCH_D.getValue());
		to.PID_YAW_P.setValue(from.PID_YAW_P.getValue());
		to.PID_YAW_I.setValue(from.PID_YAW_I.getValue());
		to.PID_YAW_D.setValue(from.PID_YAW_D.getValue());
	}

	void resetMemoryBlock(FlashMemoryBlock& memBlock)
	{
		memBlock.firstBoot.setValue(1U);
		memBlock.flashReadWriteError.setValue(0U);
		memBlock.PID_ROLL_P.setValue(0U);
		memBlock.PID_ROLL_I.setValue(0U);
		memBlock.PID_ROLL_D.setValue(0U);
		memBlock.PID_PITCH_P.setValue(0U);
		memBlock.PID_PITCH_I.setValue(0U);
		memBlock.PID_PITCH_D.setValue(0U);
		memBlock.PID_YAW_P.setValue(0U);
		memBlock.PID_YAW_I.setValue(0U);
		memBlock.PID_YAW_D.setValue(0U);
	}

	void updateMemoryBlock(FlashMemoryBlock& memBlock)
	{
		memBlock.firstBoot.updateFromMemory();
		memBlock.flashReadWriteError.updateFromMemory();
		memBlock.PID_ROLL_P.updateFromMemory();
		memBlock.PID_ROLL_I.updateFromMemory();
		memBlock.PID_ROLL_D.updateFromMemory();
		memBlock.PID_PITCH_P.updateFromMemory();
		memBlock.PID_PITCH_I.updateFromMemory();
		memBlock.PID_PITCH_D.updateFromMemory();
		memBlock.PID_YAW_P.updateFromMemory();
		memBlock.PID_YAW_I.updateFromMemory();
		memBlock.PID_YAW_D.updateFromMemory();
	}
};

class MemoryManagement: public IOManagement
{
public:
	MemoryManagement(uint32_t mainMemoryAddress):
		mainMemoryBlock{mainMemoryAddress}
	{
		this->updateMemoryBlock(this->mainMemoryBlock);

		if (this->mainMemoryBlock.firstBoot.getValue()!=1U)
		{
			resetMemoryBlock(this->mainMemoryBlock);
			writeMemoryBlock(this->mainMemoryBlock);
		}
	}

	FlashMemoryBlock& getMainMemoryBlock()
	{
		return this->mainMemoryBlock;
	}

private:

	FlashMemoryBlock mainMemoryBlock;
};

#endif /* LIBS_NVM_FLASHMEMORYBLOCK_HPP_ */
