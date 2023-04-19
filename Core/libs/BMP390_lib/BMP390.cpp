/*
 * BMP390.cpp
 *
 *  Created on: Sep 21, 2022
 *      Author: Asus
 */

#include "BMP390.hpp"

BMP390::BMP390(SPI_HandleTypeDef *spi_port)
{
	BMP390::spi_port = spi_port;
}

bool BMP390::defaultInit()
{
	SPI_write(CMD,CMD_SOFTRESET);
	HAL_Delay(20);

	int debug = SPI_read(STATUS);
	while ((debug= SPI_read(STATUS))==0)
		HAL_Delay(50);

	if (!initAndCheck(OSR,OSR_OSR_P_X16|OSR_OSR_T_X2,10))
		return false;

	if (!initAndCheck(CONFIG,CONFIG_COEF_3,10))
		return false;

	if (!initAndCheck(ODR,ODR_ODR_25,10))
		return false;

	if (!initAndCheck(INT_CTRL,INT_CTRL_DRDY_EN|INT_CTRL_INT_LEVEL,10))
		return false;

	if (!initAndCheck(PWR_CTRL,PWR_CTRL_PRESS_EN|PWR_CTRL_TEMP_EN|PWR_CTRL_MODE_NORMAL,10))
		return false;

	return true;
}

bool BMP390::initAndCheck(uint8_t addr,uint8_t val,uint8_t numberOfTries,bool read_only)
{
	for (int i=0;i<numberOfTries;i++)
	{
		if (read_only==false)
			SPI_write(addr,val);

		if (SPI_read(addr)==val)
			return true;
	}
	return false;
}

void BMP390::update()
{
	uint8_t DATA0=SPI_read(DATA_0);
	uint8_t DATA1=SPI_read(DATA_1);
	uint8_t DATA2=SPI_read(DATA_2);

	uint8_t TEMP0=SPI_read(DATA_3);
	uint8_t TEMP1=SPI_read(DATA_4);
	uint8_t TEMP2=SPI_read(DATA_5);

	pressure = ((int32_t)DATA2<<16)|((int16_t)DATA1<<8)|DATA0;
	temp = ((int32_t)TEMP2<<16)|((int16_t)TEMP1<<8)|TEMP0;
}

const char* BMP390::getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::BMP_RAW_PRESS)!=senorsList.end())
	{
		strcat(packet,toCharArray(pressure));
		strcat(packet,",");
	}

	return packet;
}

int32_t BMP390::getPressure()
{
	return pressure;
}

int32_t BMP390::getTemp()
{
	return temp;
}

uint8_t BMP390::getChipID()
{
	return SPI_read(CHIP_ID);
}

void BMP390::SPI_write(uint8_t reg,uint8_t data)
{
	HAL_GPIO_WritePin(BMP_CS_PORT,BMP_CS_PIN,GPIO_PIN_RESET);
	spiTxBuff[0] = reg;
	spiTxBuff[1] = data;
	HAL_SPI_Transmit_DMA(spi_port, (uint8_t*)spiTxBuff,2);
	HAL_GPIO_WritePin(BMP_CS_PORT,BMP_CS_PIN,GPIO_PIN_SET);
}

uint8_t BMP390::SPI_read(uint8_t reg)
{
	HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_RESET);
	spiTxBuff[0]=reg|0x80;
	spiTxBuff[1]=0x00;
	HAL_SPI_Transmit_DMA(spi_port, (uint8_t*)spiTxBuff, 2);
	HAL_SPI_Receive_DMA(spi_port, (uint8_t*)spiRxBuff, 1);
	HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_SET);

	return spiRxBuff[0];
}
