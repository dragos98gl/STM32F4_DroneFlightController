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
#include "Interfaces.hpp"
#include "Timeout.hpp"
#include "Buzzer.hpp"
#include "Enums.hpp"
#include "HC05.hpp"
#include <stdint.h>

enum class FrSkyRXState: uint8_t
{
	NOT_CONNECTED = 0U,
	CONNECTED = 1U,
	ARMED = 2U,
	READY = 3U,
	TIMEOUT = 4U
};

class FrSkyRX:public Timeout, public CallsCounter, public PrintableSensor//: UART_Conn
{
private:
	static constexpr uint16_t mid_position = 992U;
	static constexpr float roll_scaleFactor = 0.00005F;
	static constexpr float pitch_scaleFactor = 0.00005F;
	static constexpr float yaw_scaleFactor = 0.001F;
	static constexpr uint8_t packetLength = 25U;
	static constexpr uint8_t BEGIN_BIT = '\017';
	static constexpr uint8_t END_BIT = '\0';

	bool isDisconnected() const;
	void updateValues();
	void processStateMachine();
	UART_HandleTypeDef* _uartPort;
	DMA_HandleTypeDef* _uartPortDMA;
	Buzzer* _buzz;
	uint8_t _rxBuff[2U * packetLength];
	bool _wrongDataReceived;
	uint16_t _channels[16];
	uint8_t _lb;
	uint8_t _lu;
	uint8_t _rb;
	uint8_t _ru;
	uint16_t _rawRoll;
	uint16_t _rawPitch;
	uint16_t _rawYaw;
	uint8_t _rxOk;
	FrSkyRXState _currentState;
public:
	FrSkyRX(
		UART_HandleTypeDef* uartPort,
		DMA_HandleTypeDef* uartPortDMA,
		Buzzer* buzz,
		uint8_t timeout);
	float& getThrottle();
	uint8_t& getRU();
	float& getTargetRoll();
	float& getTargetPitch();
	float& getTargetYaw();
	FrSkyRXState getCurrentState() const;
	void begin();
	void update();
	bool isRxOk();
	uint8_t* getBuffer_ptr();
	uint8_t getFrameLength();
	float throttle;
	float target_roll;
	float target_pitch;
	float target_yaw;

	const char* getSensorValues_str(std::set<SENSOR_DATA_PARAMETER> &senorsList);
};

#endif /* LIBS_FRSKYRX_LIB_FRSKYRX_H_ */
