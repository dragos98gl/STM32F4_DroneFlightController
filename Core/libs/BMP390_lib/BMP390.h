/*
 * BMP390.h
 *
 *  Created on: Sep 21, 2022
 *      Author: Asus
 */

#ifndef BMP390_LIB_BMP390_H_
#define BMP390_LIB_BMP390_H_

#include "Interfaces.hpp"
#include "BMP390_reg.h"
#include "stm32f4xx_hal.h"
#include "HC05.h"

class BMP390: SPI_Conn,public PrintableSensor
{
private:
	SPI_HandleTypeDef *spi_port;

	uint8_t spiTxBuff[2]={0,0};
	uint8_t spiRxBuff[2]={0,0};

	void SPI_write(uint8_t reg,uint8_t data);
	uint8_t SPI_read(uint8_t reg);
	bool initAndCheck(uint8_t addr,uint8_t val,uint8_t numberOfTries,bool read_only = false);

	int32_t pressure;
	int32_t temp;
public:
	BMP390(SPI_HandleTypeDef *spi_port);
	bool defaultInit();
	void update();
	int32_t getPressure();
	int32_t getTemp();
	uint8_t getChipID();
	const char* getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList);
};



#endif /* BMP390_LIB_BMP390_H_ */
