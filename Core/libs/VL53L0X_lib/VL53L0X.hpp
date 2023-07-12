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
#include "MPC_Controller.hpp"

class VL53L0X final:public Timeout ,public PrintableSensor, public CallsCounter//: UART_Conn f
{
private:
	static constexpr const int packet_length = 8U;
	const uint8_t FIRST_BIT = 0x5A;
	const uint8_t SECOND_BIT = 0x5A;
	UART_HandleTypeDef *uart_port;
	DMA_HandleTypeDef *uart_port_dma;
	MPC_Controller mpc;
	float32_t mpc_out;
	uint8_t rx_buff[2U * packet_length];
	bool wrongDataReceived = false;
	uint32_t distance;
public:
	VL53L0X(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma, uint8_t timeout);
	void begin(void);
	void update(void);
	float getMPCout(void);
	float getAltitudeM(void);
	float getAltitudeCM(void);
	uint32_t getAltitudeMM(void);

	const char* getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList);
};

#endif /* LIBS_VL53L0X_LIB_VL53L0X_HPP_ */
