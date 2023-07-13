/*
 * VL53L0X.hpp
 *
 *  Created on: Jun 20, 2023
 *      Author: Dragos
 */

#ifndef LIBS_VL53L0X_LIB_VL53L0X_HPP_
#define LIBS_VL53L0X_LIB_VL53L0X_HPP_

#include <HC05.hpp>
#include "stm32f4xx_hal.h"
#include "Timeout.hpp"
#include "Enums.hpp"
#include "MPC_Controller.hpp"
#include "LowPassFilter.hpp"

class VL53L0X final:public Timeout ,public PrintableSensor, public CallsCounter//: UART_Conn f
{
private:
	static constexpr uint8_t packetLength = 8U;
	static constexpr uint8_t FIRST_BIT = 0x5A;
	static constexpr uint8_t SECOND_BIT = 0x5A;

	void computeMPC();
	UART_HandleTypeDef* _uartPort;
	DMA_HandleTypeDef* _uartPortDMA;
	MPC_Controller _mpc;
	LowPassFilter _lpf;
	float32_t _mpcOut;
	uint8_t _rxBuff[2U * packetLength];
	bool _wrongDataReceived;
	uint32_t _distance;
	float _filteredDistance;
	float _prevFilteredDistance;
	float _distanceM;
	float altRef;
public:
	VL53L0X(
		UART_HandleTypeDef *uartPort,
		DMA_HandleTypeDef *uartPortDMA,
		uint8_t timeout);
	void begin(void);
	void update(void);
	float getMPCout(void);
	float getAltitudeM(void);
	float getAltitudeCM(void);
	uint32_t getAltitudeMM(void);

	const char* getSensorValues_str(std::set<SENSOR_DATA_PARAMETER> &senorsList);
};

#endif /* LIBS_VL53L0X_LIB_VL53L0X_HPP_ */
