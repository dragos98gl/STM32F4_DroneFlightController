/*
 * FlashReadWrite.h
 *
 *  Created on: Feb 5, 2023
 *      Author: DDarie
 */

#ifndef LIBS_UTILS_FLASHREADWRITE_HPP_
#define LIBS_UTILS_FLASHREADWRITE_HPP_

#include "stdint.h"
#include <map>
#include <utility>

static constexpr uint32_t SECTOR0_BEGIN = 0x08000000;
static constexpr uint32_t SECTOR0_END = 0x08003FFF;
static constexpr uint32_t SECTOR1_BEGIN = 0x08004000;
static constexpr uint32_t SECTOR1_END = 0x08007FFF;
static constexpr uint32_t SECTOR2_BEGIN = 0x08008000;
static constexpr uint32_t SECTOR2_END = 0x0800BFFF;
static constexpr uint32_t SECTOR3_BEGIN = 0x0800C000;
static constexpr uint32_t SECTOR3_END = 0x0800FFFF;
static constexpr uint32_t SECTOR4_BEGIN = 0x08010000;
static constexpr uint32_t SECTOR4_END = 0x0801FFFF;
static constexpr uint32_t SECTOR5_BEGIN = 0x08020000;
static constexpr uint32_t SECTOR5_END = 0x0803FFFF;
static constexpr uint32_t SECTOR6_BEGIN = 0x08040000;
static constexpr uint32_t SECTOR6_END = 0x0805FFFF;
static constexpr uint32_t SECTOR7_BEGIN = 0x08060000;
static constexpr uint32_t SECTOR7_END = 0x0807FFFF;
static constexpr uint32_t SECTOR8_BEGIN = 0x08080000;
static constexpr uint32_t SECTOR8_END = 0x0809FFFF;
static constexpr uint32_t SECTOR9_BEGIN = 0x080A0000;
static constexpr uint32_t SECTOR9_END = 0x080BFFFF;
static constexpr uint32_t SECTOR10_BEGIN = 0x080C0000;
static constexpr uint32_t SECTOR10_END = 0x080DFFFF;
static constexpr uint32_t SECTOR11_BEGIN = 0x080E0000;
static constexpr uint32_t SECTOR11_END = 0x080FFFFF;


enum class MemoryDataType: uint8_t
{
	intType,
	floatType
};

enum class MemoryDescriptor: uint8_t
{
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

static std::map<MemoryDescriptor, std::pair<uint32_t,MemoryDataType>> memoryMapp =
{
		{MemoryDescriptor::PID_ROLL_P,{0x080E0000,MemoryDataType::intType}},
		{MemoryDescriptor::PID_ROLL_I,{0x080E0004,MemoryDataType::floatType}},
		{MemoryDescriptor::PID_ROLL_D,{0x080E0008,MemoryDataType::floatType}}
};

template <typename dataType>
static void dataType2Bytes(uint8_t* buffer, dataType data)
{
	const uint8_t dataSize = sizeof(dataType);

    union {
    	dataType obj;
    	uint8_t bytes[dataSize];
    } dataObj;

    dataObj.obj = data;

    for (uint8_t i = 0; i < dataSize; i++) {
      buffer[i] = dataObj.bytes[i];
    }
}


template <typename dataType>
static dataType bytes2DataType(const uint8_t bytes[])
{
	const uint8_t dataSize = sizeof(dataType);

    union {
      dataType obj;
      uint8_t bytes[dataSize];
    } dataObj;

    for (uint8_t i = 0; i < dataSize; i++) {
    	dataObj.bytes[i] = bytes[i];
    }

   return dataObj.obj;
}

class nvm
{
public:
	uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint32_t *Data, uint16_t numberofwords);
	void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords);
	float Flash_Read_NUM (uint32_t StartSectorAddress);

	void erase(uint32_t adress);

	uint32_t massEraseSector(uint32_t sectorAdress);

	uint32_t write(MemoryDescriptor memDes,float value);
	uint32_t write(MemoryDescriptor memDes,int value);
private:
	void eraseFlashSector(uint32_t sector);
};

#endif /* LIBS_UTILS_FLASHREADWRITE_HPP_ */
