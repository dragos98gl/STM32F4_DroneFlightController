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
#include "Enums.hpp"
#include "VL53L0X.hpp"

class PMW3901UY final:public Timeout ,public PrintableSensor, public CallsCounter//: UART_Conn f
{
private:
	static constexpr int packetLength = 9U;
	static constexpr uint8_t BEGIN_BIT = 0xFE;
	static constexpr uint8_t DATA_LEN_BIT = 0x04;
	static constexpr uint8_t END_BIT = 0xAA;

	void process();
	UART_HandleTypeDef* _uartPort;
	DMA_HandleTypeDef* _uartPortDMA;
	ICM42688P& _icm;
	VL53L0X& _vl53;
	PID_Control& _pidX;
	PID_Control& _pidY;
	uint8_t _rxBuff[2U * packetLength];
	bool _wrongDataReceived;
	int16_t _flowX;
	int16_t _flowY;
	uint8_t _quality;
	float _xPos;
	float _yPos;
	float _xCmPos;
	float _yCmPos;
	float _targetX;
	float _targetY;
	float _lastAngleX;
	float _lastAngleY;
public:
	PMW3901UY(
		UART_HandleTypeDef* uartPort,
		DMA_HandleTypeDef* uartPortDMA,
		uint8_t timeout,
		ICM42688P& icm,
		VL53L0X& vl53,
		PID_Control& pidX,
		PID_Control& pidY);
	void begin(void);
	void update(void);
	uint16_t getFlowX();
	uint16_t getFlowY();
	uint8_t getQuality();
	float& getXpos();
	float& getYpos();
	float& getTargetX();
	float& getTargetY();
	const char* getSensorValues_str(std::set<SENSOR_DATA_PARAMETER> &senorsList);
};

#endif /* PMW3901UY_LIB_PMW3901UY_H_ */
