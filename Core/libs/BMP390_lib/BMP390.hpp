/*
 * BMP390.h
 *
 *  Created on: Sep 21, 2022
 *      Author: Asus
 */

#ifndef BMP390_LIB_BMP390_H_
#define BMP390_LIB_BMP390_H_

#include "HC05.hpp"
#include "Interfaces.hpp"
#include "BMP390_reg.hpp"
#include "stm32f4xx_hal.h"
#include "math.h"

class BMP390: SPI_Conn,public PrintableSensor
{
private:
	SPI_HandleTypeDef *spi_port;

	uint8_t spiTxBuff[2]={0,0};
	uint8_t spiRxBuff[2]={0,0};

	struct calib_coef
	{
		uint16_t par_t1 = 0;
		uint16_t par_t2 = 0;
		int8_t par_t3 = 0;
		uint16_t par_p1 = 0;
		uint16_t par_p2 = 0;
		int8_t par_p3 = 0;
		int8_t par_p4 = 0;
		uint16_t par_p5 = 0;
		uint16_t par_p6 = 0;
		int8_t par_p7 = 0;
		int8_t par_p8 = 0;
		uint16_t par_p9 = 0;
		int8_t par_p10 = 0;
		int8_t par_p11 = 0;
	} calibCoef;

	struct quantized_calib_data
	{
	    double par_t1;
	    double par_t2;
	    double par_t3;
	    double par_p1;
	    double par_p2;
	    double par_p3;
	    double par_p4;
	    double par_p5;
	    double par_p6;
	    double par_p7;
	    double par_p8;
	    double par_p9;
	    double par_p10;
	    double par_p11;
	    double t_lin;
	} quantizedCalibCoef;

	void SPI_write(uint8_t reg,uint8_t data);
	uint8_t SPI_read(uint8_t reg);
	bool initAndCheck(uint8_t addr,uint8_t val,uint8_t numberOfTries,bool read_only = false);

	void read_calib_data();
	void compensate_data();
	double compensate_temperature();
	double compensate_pressure();

	double pressure;
	double temp;
	int32_t raw_pressure;
	int32_t raw_temp;
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
