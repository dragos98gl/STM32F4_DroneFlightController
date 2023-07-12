/*
 * LIS3MDLTR.cpp
 *
 *  Created on: Sep 21, 2022
 *      Author: Asus
 */

#include "LIS3MDLTR.hpp"

LIS3MDLTR::LIS3MDLTR(SPI_HandleTypeDef *spi_port):
	spi_port {spi_port}
	,spiTxBuff {0,0}
	,spiRxBuff {0,0}
	,x_mag {0}
	,y_mag {0}
	,z_mag {0}
	,z_angle {0}
	,x_raw {0}
	,y_raw {0}
	,z_raw {0}
	,TEMP_val {0}
{
}

uint8_t LIS3MDLTR::whoAmI()
{
	return SPI_read(LIS_WHO_AM_I);
}

void LIS3MDLTR::update()
{
	uint8_t x_high=SPI_read(OUT_X_H);
	uint8_t x_low=SPI_read(OUT_X_L);
	uint8_t y_high=SPI_read(OUT_Y_H);
	uint8_t y_low=SPI_read(OUT_Y_L);
	uint8_t z_high=SPI_read(OUT_Z_H);
	uint8_t z_low=SPI_read(OUT_Z_L);
	uint8_t temp_high=SPI_read(TEMP_OUT_H);
	uint8_t temp_low=SPI_read(TEMP_OUT_L);

	x_raw = ((int16_t)x_high)<<8 | x_low;
	y_raw = ((int16_t)y_high)<<8 | y_low;
	z_raw = ((int16_t)z_high)<<8 | z_low;
	TEMP_val = ((int16_t)temp_high)<<8 | temp_low;
}

const char* LIS3MDLTR::getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::LIS_RAW_MAG_X)!=senorsList.end())
	{
		strcat(packet,toCharArray(x_raw));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::LIS_RAW_MAG_Y)!=senorsList.end())
	{
		strcat(packet,toCharArray(y_raw));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::LIS_RAW_MAG_Z)!=senorsList.end())
	{
		strcat(packet,toCharArray(z_raw));
		strcat(packet,",");
	}


	return packet;
}

bool LIS3MDLTR::defaultInit()
{
	  this->SPI_write(CTRL_REG1,0b11111100);
	  uint8_t ctrl1 = this->SPI_read(CTRL_REG1);

	  SPI_write(CTRL_REG2,0b01100000);
	  uint8_t ctrl2 = SPI_read(CTRL_REG2);

	  SPI_write(CTRL_REG3,0b00000000);
	  uint8_t ctrl3 = SPI_read(CTRL_REG3);

	  SPI_write(CTRL_REG4,0b00001100);
	  uint8_t ctrl4 = SPI_read(CTRL_REG4);

	  SPI_write(CTRL_REG5,0b00000000);
	  uint8_t ctrl5 = SPI_read(CTRL_REG5);
	/*
	 * ??????????????????????????????????????
	 * if (!initAndCheck(LIS_WHO_AM_I,LIS_WHO_AM_I_VALUE,10,true))
		return false;

	if (!initAndCheck(CTRL_REG2,0b00000100,10))
		return false;

	HAL_Delay(50);

	if (!initAndCheck(CTRL_REG1,0b11111100,10))
		return false;

	if (!initAndCheck(CTRL_REG2,0b01100000,10))
		return false;

	if (!initAndCheck(CTRL_REG3,0b00000000,10))
		return false;

	if (!initAndCheck(CTRL_REG4,0b00001100,10))
		return false;

	if (!initAndCheck(CTRL_REG5,0b00000000,10))
		return false;*/

	return true;
}

bool LIS3MDLTR::initAndCheck(uint8_t addr,uint8_t val,uint8_t numberOfTries,bool read_only)
{
	for (int i=0;i<numberOfTries;i++)
	{
		if (read_only==false)
			SPI_write(addr,val);
		uint8_t debug = SPI_read(addr);
		if (SPI_read(addr)==val)
			return true;
	}
	return false;
}

void LIS3MDLTR::SPI_write(uint8_t reg,uint8_t data)
{
	HAL_GPIO_WritePin(LIS_CS_PORT,LIS_CS_PIN,GPIO_PIN_RESET);
	spiTxBuff[0] = reg & 0x7f;
	spiTxBuff[1] = data;
	HAL_SPI_Transmit_DMA(spi_port, (uint8_t*)spiTxBuff,2);
	HAL_GPIO_WritePin(LIS_CS_PORT,LIS_CS_PIN,GPIO_PIN_SET);
}

uint8_t LIS3MDLTR::SPI_read(uint8_t reg)
{
	HAL_GPIO_WritePin(LIS_CS_PORT, LIS_CS_PIN, GPIO_PIN_RESET);
	spiTxBuff[0]=(reg & 0x3f)|0x80|0x40;
	HAL_SPI_Transmit_DMA(spi_port, (uint8_t*)spiTxBuff, 1);
	HAL_SPI_Receive_DMA(spi_port, (uint8_t*)spiRxBuff, 1);
	HAL_GPIO_WritePin(LIS_CS_PORT, LIS_CS_PIN, GPIO_PIN_SET);

	return spiRxBuff[0];
}

int16_t LIS3MDLTR::getX()
{
	return x_raw;
}

int16_t LIS3MDLTR::getY()
{
	return y_raw;
}

int16_t LIS3MDLTR::getZ()
{
	return z_raw;
}

int16_t LIS3MDLTR::getTEMP()
{
	return TEMP_val;
}
