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
#include "PID_Control.hpp"
#include "VL53L0X.hpp"

class PMW3901UY final:public Timeout ,public PrintableSensor, public CallsCounter//: UART_Conn f
{
private:
	static constexpr const int packet_length = 9U;
	const uint8_t BEGIN_BIT = 0xFE;
	const uint8_t DATA_LEN_BIT = 0x04;
	const uint8_t END_BIT = 0xAA;

	UART_HandleTypeDef *uart_port;
	DMA_HandleTypeDef *uart_port_dma;
	ICM42688P& _icm;
	VL53L0X& _vl53;
	PID_Control& _pidX;
	PID_Control& _pidY;

	uint8_t rx_buff[2U * packet_length];
	bool wrongDataReceived = false;
	int16_t flow_x;
	int16_t flow_y;
	uint8_t quality;
	float x_pos;
	float y_pos;
	float x_cm_pos;
	float y_cm_pos;
	float target_x;
	float target_y;
	float lastAngleX;
	float lastAngleY;

	void process();
public:
	PMW3901UY(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma, uint8_t timeout ,ICM42688P& icm,VL53L0X& vl53, PID_Control& pidX,PID_Control& pidY);
	void begin(void);
	void update(void);
	uint16_t getFlowX();
	uint16_t getFlowY();
	uint8_t getQuality();
	float& getXpos();
	float& getYpos();
	float& getTargetX();
	float& getTargetY();
	const char* getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList);
};

#endif /* PMW3901UY_LIB_PMW3901UY_H_ */
