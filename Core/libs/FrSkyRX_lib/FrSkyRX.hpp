/*
 * FrSkyRX.h
 *
 *  Created on: Sep 29, 2022
 *      Author: DragosDarie
 */

#ifndef LIBS_FRSKYRX_LIB_FRSKYRX_H_
#define LIBS_FRSKYRX_LIB_FRSKYRX_H_

/*
 * UART:INVERTED
 * BAUD:100000
 * PARITY:EVEN
 * STOP_BITS:2
 *
 * UART:DMA
 * DMA_DIRECTION:PERIPHERAL TO MEMORY
 * DMA_MODE:CIRCULAR
 */

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Timeout.hpp"
#include "Buzzer.hpp"
#include <stdint.h>

enum class FrSkyRXState: uint8_t
{
	NOT_CONNECTED = 0U,
	CONNECTED = 1U,
	ARMED = 2U,
	READY = 3U,
	TIMEOUT = 4U
};

class FrSkyRX:public Timeout//: UART_Conn
{
private:
	static constexpr const uint16_t mid_position = 992U;
	static constexpr const float roll_scaleFactor = 0.01F;
	static constexpr const float pitch_scaleFactor = 0.01F;
	static constexpr const float yaw_scaleFactor = 0.001F;
	static constexpr const uint8_t packet_length = 25U;
	const uint8_t BEGIN_BIT = '\017';
	const uint8_t END_BIT = '\0';

	UART_HandleTypeDef *uart_port;
	DMA_HandleTypeDef *uart_port_dma;
	Buzzer *buzz;

	uint8_t rx_buff[2U * packet_length];
	bool wrongDataReceived = false;

	uint16_t channels[16];
	uint8_t rx_ok = 0;

	FrSkyRXState currentState;
	bool isDisconnected() const;
	void updateValues();
	void processStateMachine();
public:
	float throttle;
	float target_roll;
	float target_pitch;
	float target_yaw;

	uint16_t raw_roll;
	uint16_t raw_pitch;
	uint16_t raw_yaw;
	uint8_t lb;
	uint8_t lu;
	uint8_t rb;
	uint8_t ru;

	FrSkyRX(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma,Buzzer *buzz,uint8_t timeout);
	float getThrottle();
	float getTargetRoll();
	float getTargetPitch();
	float getTargetYaw();
	FrSkyRXState getCurrentState() const;

	void begin();
	void update();
	bool isRxOk();
	uint8_t* getBuffer_ptr();
	uint8_t getFrameLength();
};

#endif /* LIBS_FRSKYRX_LIB_FRSKYRX_H_ */
