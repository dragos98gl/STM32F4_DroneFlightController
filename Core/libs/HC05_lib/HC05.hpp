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
#include "Enums.hpp"
#include <stdint.h>
#include <vector>

class HC05//: UART_Conn
{
private:
	UART_HandleTypeDef* _uartPort;
	std::vector<PrintableSensor*> _senList;
	std::set<SENSOR_DATA_PARAMETER> _senorsList;
	char _USART1TxBuffer[100];
public:
	HC05(UART_HandleTypeDef* uartPort);
	void send();
	void send(const char* data,uint8_t len);
	void addSensor(PrintableSensor* sen);
	void printfSensorsValues();
	void addSensorParameter(SENSOR_DATA_PARAMETER sen);
};

#endif /* LIBS_HC05_LIB_HC05_HPP_ */
