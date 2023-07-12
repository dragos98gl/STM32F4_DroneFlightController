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
#include "utils_functions.hpp"

class BMP390: SPI_Conn,public PrintableSensor, public CallsCounter
{
private:
	struct calib_coef
	{
		uint16_t par_t1 {0};
		uint16_t par_t2 {0};
		int8_t par_t3 {0};
		int16_t par_p1 {0};
		int16_t par_p2 {0};
		int8_t par_p3 {0};
		int8_t par_p4 {0};
		uint16_t par_p5 {0};
		uint16_t par_p6 {0};
		int8_t par_p7 {0};
		int8_t par_p8 {0};
		int16_t par_p9 {0};
		int8_t par_p10 {0};
		int8_t par_p11 {0};
	} calibCoef;

	struct quantized_calib_data
	{
	    double par_t1 {0};
	    double par_t2 {0};
	    double par_t3 {0};
	    double par_p1 {0};
	    double par_p2 {0};
	    double par_p3 {0};
	    double par_p4 {0};
	    double par_p5 {0};
	    double par_p6 {0};
	    double par_p7 {0};
	    double par_p8 {0};
	    double par_p9 {0};
	    double par_p10 {0};
	    double par_p11 {0};
	    double t_lin {0};
	} quantizedCalibCoef;

	SPI_HandleTypeDef *spi_port;

	uint8_t spiTxBuff[2];
	uint8_t spiRxBuff[2];
	double pressure;
	double temp;
	uint32_t raw_pressure;
	uint32_t raw_temp;

	void SPI_write(uint8_t reg,uint8_t data);
	uint8_t SPI_read(uint8_t reg);
	bool initAndCheck(uint8_t addr,uint8_t val,uint8_t numberOfTries,bool read_only = false);
	void read_calib_data();
	void compensate_data();
	void compensate_temperature();
	void compensate_pressure();
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
