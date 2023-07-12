/*
 * HC05.h
 *
 *  Created on: Sep 27, 2022
 *      Author: DragosDarie
 */

#ifndef LIBS_HC05_LIB_HC05_HPP_
#define LIBS_HC05_LIB_HC05_HPP_

#include "stm32f4xx_hal.h"
#include "Interfaces.hpp"
#include <stdint.h>
#include <vector>
#include <set>
#include <string>

class PrintableSensor;

class HC05//: UART_Conn
{
public:
	enum class SENSOR_DATA_PARAMETER
	{
		ICM_RAW_GX,
		ICM_RAW_GY,
		ICM_RAW_GZ,
		ICM_GX,
		ICM_GY,
		ICM_GZ,
		ICM_RAW_AX,
		ICM_RAW_AY,
		ICM_RAW_AZ,
		ICM_AX,
		ICM_AY,
		ICM_AZ,
		ICM_EULER_X,
		ICM_EULER_Y,
		ICM_EULER_Z,
		BMP_RAW_PRESS,
		LIS_RAW_MAG_X,
		LIS_RAW_MAG_Y,
		LIS_RAW_MAG_Z,
		SONAR_DISTANCE,
		VL53_DISTANCE,
		PMW_POS_X,
		PMW_POS_Y,
		PMW_FLOW_X,
		PMW_FLOW_Y,
		PMW_QUALITY,
		FRSKY_THROTTLE
	};

private:
	std::set<SENSOR_DATA_PARAMETER> senorsList;

	UART_HandleTypeDef *uart_port;
	std::vector<PrintableSensor*> senList;
	char USART1_TxBuffer[100]={};

public:
	HC05(UART_HandleTypeDef *uart_port);
	void send();
	void send(const char *data,uint8_t len);
	void addSensor(PrintableSensor* sen);
	void printfSensorsValues();
	void addSensorParameter(SENSOR_DATA_PARAMETER sen);
};

class PrintableSensor
{
protected:
	char packet[50]={};

public:
	virtual const char* getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList) = 0;

	const char* toCharArray(int value)
	{
		return std::to_string(value).c_str();
	}
};

#endif /* LIBS_HC05_LIB_HC05_HPP_ */
