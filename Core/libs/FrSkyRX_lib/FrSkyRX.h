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
#include "Timeout.h"
#include "Buzzer.hpp"
#include <stdint.h>

class FrSkyRX:public Timeout//: UART_Conn
{
private:
	UART_HandleTypeDef *uart_port;
	DMA_HandleTypeDef *uart_port_dma;
	Buzzer *buzz;

	uint8_t frame_length = 25;
	uint8_t rx_frame[25];
	uint16_t channels[16];
	uint8_t rx_ok = 0;

public:
	uint16_t throttle;
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	uint8_t lb;
	uint8_t lu;
	uint8_t rb;
	uint8_t ru;

	FrSkyRX(UART_HandleTypeDef *uart_port,DMA_HandleTypeDef *uart_port_dma,Buzzer *buzz,uint8_t timeout);
	void begin();
	void update();
	bool isRxOk();
	uint8_t* getBuffer_ptr();
	uint8_t getFrameLength();
};

#endif /* LIBS_FRSKYRX_LIB_FRSKYRX_H_ */
