/*
 * LIS3MDLTR.h
 *
 *  Created on: Sep 21, 2022
 *      Author: Asus
 */

#ifndef LIS3MDLTR_LIB_LIS3MDLTR_H_
#define LIS3MDLTR_LIB_LIS3MDLTR_H_

#include "Interfaces.hpp"
#include "LIS3MDLTR_reg.h"
#include "stm32f4xx_hal.h"
#include "HC05.h"

class LIS3MDLTR : SPI_Conn,public PrintableSensor
{
private:
	SPI_HandleTypeDef *spi_port;

	uint8_t spiTxBuff[2]={0,0};
	uint8_t spiRxBuff[2]={0,0};

	bool initAndCheck(uint8_t addr,uint8_t val,uint8_t numberOfTries,bool read_only = false);

	int16_t X_val;
	int16_t Y_val;
	int16_t Z_val;
	int16_t TEMP_val;
public:
	void SPI_write(uint8_t reg,uint8_t data);
	uint8_t SPI_read(uint8_t reg);

	LIS3MDLTR(SPI_HandleTypeDef *spi_port);
	uint8_t	whoAmI();
	bool defaultInit();
	void update();
	int16_t getX();
	int16_t getY();
	int16_t getZ();
	int16_t getTEMP();
	const char* getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList);
};


#endif /* LIS3MDLTR_LIB_LIS3MDLTR_H_ */
