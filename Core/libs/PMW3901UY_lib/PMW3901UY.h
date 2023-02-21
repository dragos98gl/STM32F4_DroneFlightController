/*
 * PMW3901UY.h
 *
 *  Created on: Sep 26, 2022
 *      Author: Asus
 */

#ifndef PMW3901UY_LIB_PMW3901UY_H_
#define PMW3901UY_LIB_PMW3901UY_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "Timeout.h"
#include "HC05.h"

/*
 * TODO:Add timeout
 */
class PMW3901UY:public Timeout ,public PrintableSensor//: UART_Conn
{
private:
	uint8_t BEGIN_BIT = 0xFE;
	uint8_t DATA_LEN_BIT = 0x04;
	uint8_t END_BIT = 0xAA;

	UART_HandleTypeDef *uart_port;
	DMA_HandleTypeDef *uart_port_dma;

	uint8_t buff_len = 9;
	uint8_t rx_buff[9];
	int16_t flow_x;
	int16_t flow_y;
	uint8_t quality;
	int16_t x_pos=0;
	int16_t y_pos=0;
public:
	PMW3901UY(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma,uint8_t timeout);
	void begin(void);
	void update(void);
	uint16_t getFlowX();
	uint16_t getFlowY();
	uint8_t getQuality();
	uint16_t getXpos();
	uint16_t getYpos();
	const char* getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList);
};

#endif /* PMW3901UY_LIB_PMW3901UY_H_ */
