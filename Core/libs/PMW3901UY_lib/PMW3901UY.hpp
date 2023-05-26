/*
 * PMW3901UY.h
 *
 *  Created on: Sep 26, 2022
 *      Author: Asus
 */

#ifndef PMW3901UY_LIB_PMW3901UY_H_
#define PMW3901UY_LIB_PMW3901UY_H_

#include <HC05.hpp>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include "Timeout.hpp"
#include <stdlib.h>
#include "ICM42688P.hpp"
///ICM42688P
/*
 * TODO:Add timeout
 */
class PMW3901UY final:public Timeout ,public PrintableSensor, public CallsCounter //: UART_Conn f
{
private:
	static constexpr const int packet_length = 9U;
	const uint8_t BEGIN_BIT = 0xFE;
	const uint8_t DATA_LEN_BIT = 0x04;
	const uint8_t END_BIT = 0xAA;

	UART_HandleTypeDef *uart_port;
	DMA_HandleTypeDef *uart_port_dma;
	ICM42688P &icm;

	uint8_t rx_buff[2U * packet_length];
	bool wrongDataReceived = false;
	int16_t flow_x;
	int16_t flow_y;
	uint8_t quality;
	int16_t x_pos=0;
	int16_t y_pos=0;

	struct mini
	{
		float flow_x;
		float flow_y;
		float flow_x_i;
		float flow_y_i;
	} _mini;

	struct pixel_flow
	{
		float flow_x;
		float flow_y;
		float fix_x_i;
		float fix_y_i;
		float ang_x;
		float ang_y;
		float out_x_i;
		float out_y_i;
		float x;
		float y;
		float fit_x;
		float fit_y;
		float out_x_i_o;
		float out_y_i_o;
		float fix_x;
		float fix_y;
	} _pixel_flow;
	void process();
public:
	PMW3901UY(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma, uint8_t timeout ,ICM42688P& icm);
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
