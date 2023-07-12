/*
 * nvmUtils.hpp
 *
 *  Created on: Jun 6, 2023
 *      Author: Dragos
 */

#include "nvmTypes.hpp"

#ifndef LIBS_NVM_NVMUTILS_HPP_
#define LIBS_NVM_NVMUTILS_HPP_

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

static constexpr uint32_t MAINMEMORY_ADDRESS = SECTOR11_BEGIN;

inline uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address >= SECTOR0_BEGIN) && (Address < SECTOR0_END))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address >= SECTOR1_BEGIN) && (Address < SECTOR1_END))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address >= SECTOR2_BEGIN) && (Address < SECTOR2_END))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address >= SECTOR3_BEGIN) && (Address < SECTOR3_END))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address >= SECTOR4_BEGIN) && (Address < SECTOR4_END))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address >= SECTOR5_BEGIN) && (Address < SECTOR5_END))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address >= SECTOR6_BEGIN) && (Address < SECTOR6_END))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address >= SECTOR7_BEGIN) && (Address < SECTOR7_END))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address >= SECTOR8_BEGIN) && (Address < SECTOR8_END))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address >= SECTOR9_BEGIN) && (Address < SECTOR9_END))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address >= SECTOR10_BEGIN) && (Address < SECTOR10_END))
  {
    sector = FLASH_SECTOR_10;
  }
  else if((Address >= SECTOR11_BEGIN) && (Address < SECTOR11_END))
  {
    sector = FLASH_SECTOR_11;
  }

  return sector;
}

inline uint32_t massEraseSector(uint32_t sectorAddress)
{
	static FLASH_EraseInitTypeDef eraseInitStruct;
	uint32_t sectorError;

	HAL_FLASH_Unlock();

	uint32_t sector = GetSector(sectorAddress);

	eraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	eraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	eraseInitStruct.Sector        = sector;
	eraseInitStruct.NbSectors     = 1U;

	if (HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError) != HAL_OK)
	{
		return HAL_FLASH_GetError ();
	}

	HAL_FLASH_Lock();

	return 0;
}

template <typename Type>
inline Type readMemoryAddress(const uint32_t addr)
{
	return static_cast<Type>(*(__IO uint32_t *)addr);
}

template <typename Type>
inline uint32_t writeMemoryAddress(uint32_t address,Type memoryData)
{
	HAL_FLASH_Unlock();

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, memoryData.bytes) != HAL_OK)
    {
    	return HAL_FLASH_GetError ();
    }

	HAL_FLASH_Lock();

	return 0;
}

#endif /* LIBS_NVM_NVMUTILS_HPP_ */
