/*
 * FlashReadWrite.h
 *
 *  Created on: Feb 5, 2023
 *      Author: DDarie
 */

#ifndef LIBS_UTILS_FLASHREADWRITE_HPP_
#define LIBS_UTILS_FLASHREADWRITE_HPP_

#include "stdint.h"

uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint32_t *Data, uint16_t numberofwords);
void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords);
void Convert_To_Str (uint32_t *Data, char *Buf);
void Flash_Write_NUM (uint32_t StartSectorAddress, float Num);
float Flash_Read_NUM (uint32_t StartSectorAddress);

#endif /* LIBS_UTILS_FLASHREADWRITE_HPP_ */
