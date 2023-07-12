/*
 * Timeout.h
 *
 *  Created on: Sep 30, 2022
 *      Author: Asus
 */

#ifndef LIBS_INTERFACES_TIMEOUT_H_
#define LIBS_INTERFACES_TIMEOUT_H_

enum Status
{
	Timeout,
	OK
};

class Timeout
{
private:
	Status status {Status::OK};
	uint8_t timeout_counter {0U};
	uint8_t timeout_value {0U};
public:
	void resetTimeoutCounter()
	{
		timeout_counter=0;
	}

	void incrementTimeoutCounter()
	{
		timeout_counter++;

		if (timeout_counter>timeout_value)
			status = Status::Timeout;
	}

	uint8_t* getTimeoutCounter()
	{
		return &timeout_counter;
	}

	void setTimeoutValue(uint8_t value)
	{
		timeout_value = value;
	}

	Status getStatus()
	{
		return status;
	}
};

#endif /* LIBS_INTERFACES_TIMEOUT_H_ */
