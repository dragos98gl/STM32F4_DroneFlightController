/*
 * MB1043.h
 *
 *  Created on: Sep 29, 2022
 *      Author: DragosDarie
 */

#ifndef LIBS_MB1043_LIB_MB1043_H_
#define LIBS_MB1043_LIB_MB1043_H_

#include "HC05.hpp"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "Timeout.hpp"

class MB1043:public Timeout ,public PrintableSensor, public CallsCounter //: UART_Conn
{
private:
	static constexpr const int packet_length = 6U;
	uint8_t BEGIN_BIT = 'R';
	uint8_t END_BIT = '\r';

	UART_HandleTypeDef *uart_port;
	DMA_HandleTypeDef *uart_port_dma;

	bool wrongDataReceived = false;
	uint8_t rx_buff[2U * packet_length];
	char distance_str[4];
	uint16_t distance;
public:
	MB1043(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma,uint8_t timeout);
	void begin();
	void update();
	char* getDistance_str();
	uint8_t getDistance();
	uint8_t* getTimeoutCounter();
	const char* getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList);
};

#endif /* LIBS_MB1043_LIB_MB1043_H_ */
