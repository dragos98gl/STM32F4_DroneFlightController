/*
 * LIS3MDLTR.h
 *
 *  Created on: Sep 21, 2022
 *      Author: Asus
 */

#ifndef LIS3MDLTR_LIB_LIS3MDLTR_H_
#define LIS3MDLTR_LIB_LIS3MDLTR_H_

#include "HC05.hpp"
#include "Enums.hpp"
#include "Interfaces.hpp"
#include "LIS3MDLTR_reg.h"
#include "stm32f4xx_hal.h"
#include <math.h>

class LIS3MDLTR : SPI_Conn,public PrintableSensor, public CallsCounter
{
private:
	bool initAndCheck(
		uint8_t addr,
		uint8_t val,
		uint8_t numberOfTries,
		bool read_only = false);
	SPI_HandleTypeDef* _spiPort;
	uint8_t _spiTxBuff[2];
	uint8_t _spiRxBuff[2];
	float _xMag;
	float _yMag;
	float _zMag;
	float _zAngle;
	int16_t _xRaw;
	int16_t _yRaw;
	int16_t _zRaw;
	int16_t _temp;
public:
	LIS3MDLTR(SPI_HandleTypeDef* spiPort);
	void SPI_write(uint8_t reg,uint8_t data);
	uint8_t SPI_read(uint8_t reg);
	uint8_t	whoAmI();
	bool defaultInit();
	void update();
	int16_t getX();
	int16_t getY();
	int16_t getZ();
	int16_t getTemp();
	const char* getSensorValues_str(std::set<SENSOR_DATA_PARAMETER> &senorsList);
};


#endif /* LIS3MDLTR_LIB_LIS3MDLTR_H_ */
