/*
 * FlashReadWrite.cpp
 *
 *  Created on: Feb 5, 2023
 *      Author: DDarie
 */

#include "FlashReadWrite.hpp"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"

static uint32_t GetSector(uint32_t Address)
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

void nvm::Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{

		*RxBuf = *(__IO uint32_t *)StartSectorAddress;
		StartSectorAddress += 4;
		RxBuf++;
		if (!(numberofwords--)) break;
	}
}

float nvm::Flash_Read_NUM (uint32_t StartSectorAddress)
{
	uint8_t buffer[4];
	int value;

	Flash_Read_Data(StartSectorAddress, (uint32_t *)buffer, 1);
	value = bytes2DataType<int>(buffer);
	return value;
}

uint32_t nvm::write(MemoryDescriptor memDes,int value)
{
	uint32_t adress=memoryMapp[MemoryDescriptor::PID_ROLL_P].first;

	uint8_t dataSize = sizeof(value);
	uint8_t buffer[dataSize];
	dataType2Bytes(buffer, value);
	HAL_FLASH_Unlock();

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, adress, buffer[0]) != HAL_OK)
    {
    	return HAL_FLASH_GetError ();
    }

	HAL_FLASH_Lock();

	return 0;
}

uint32_t nvm::massEraseSector(uint32_t sectorAdress)
{
	static FLASH_EraseInitTypeDef eraseInitStruct;
	uint32_t sectorError;

	HAL_FLASH_Unlock();

	uint32_t sector = GetSector(sectorAdress);

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
