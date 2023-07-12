/*
 * Interfaces.h
 *
 *  Created on: Sep 17, 2022
 *      Author: Asus
 */

#ifndef INTERFACES_INTERFACES_H_
#define INTERFACES_INTERFACES_H_

#include <stdint.h>
#include <string.h>

class SPI_Conn
{
private:
	virtual void SPI_write(uint8_t reg,uint8_t data)=0;
	virtual uint8_t SPI_read(uint8_t reg)=0;
};

class UART_Conn
{
private:
	virtual void UART_read()=0;
	virtual void UART_write()=0;
};

class CallsCounter
{
public:
	void incrementInterruptCounter()
	{
		this->interruptCounter++;
	}

	void incrementTaskCounter()
	{
		this->taskCounter++;
	}

	uint64_t getInterruptCounterValue()
	{
		return this->interruptCounter;
	}

	uint64_t getTaskCounterValue()
	{
		return this->interruptCounter;
	}
protected:
	CallsCounter() = default;

private:
	uint64_t interruptCounter{0};
	uint64_t taskCounter{0};
};

#endif /* INTERFACES_INTERFACES_H_ */
