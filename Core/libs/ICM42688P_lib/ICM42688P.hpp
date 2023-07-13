    /*
 * ICM42688P.h
 *
 *  Created on: Jul 22, 2022
 *      Author: DragosDarie
 */

#ifndef INC_ICM42688P_H_
#define INC_ICM42688P_H_

#include "HC05.hpp"
#include "Enums.hpp"
#include "ICM42688P_reg.hpp"
#include "Interfaces.hpp"
#include "Buzzer.hpp"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "string.h"
#include <set>
#include "PID_Control.hpp"

class ICM42688P: SPI_Conn,public PrintableSensor, public CallsCounter
{
private:
	static constexpr float RADIANS_TO_DEGREES = 180.0F / M_PI;
	static constexpr float GYRO_FULLSCALE = 32768.0F / 2000.0F;
	static constexpr float DT = 1.0F / 1000.0F;
	static constexpr uint16_t criticalStateAngleThreshold = 20.0F;//10

	uint8_t SPI_read(uint8_t reg);
	bool initAndCheck(
		uint8_t addr,
		uint8_t val,
		uint8_t numberOfTries,
		bool read_only = false);
	void checkCriticalState();
	void checkCrashState();
	SPI_HandleTypeDef* _spiPort;
	Buzzer* _buzz;
	PID_Control& _rollPID;
	PID_Control& _pitchPID;
	PID_Control& _yawPID;
	uint8_t _spiTxBuff[2];
	uint8_t _spiRxBuff[2];
	float _gx;
	float _gy;
	float _gz;
	float _ax;
	float _ay;
	float _az;
	float _rawGx;
	float _rawGy;
	float _rawGz;
	float _rawAx;
	float _rawAy;
	float _rawAz;
	float _temp;
	float _eulerX;
	float _eulerY;
	float _eulerZ;
	float _gxDrift;
	float _gyDrift;
	float _gzDrift;
	float _axOffset;
	float _ayOffset;
	float _azOffset;
	float _axScale;
	float _ayScale;
	float _azScale;
	float _prevRawAx;
	float _prevRawAy;
	float _prevRawAz;
	float _maxAxDt;
	float _maxAyDt;
	float _maxAzDt;
	bool _crashState;
	bool _criticalState;
public:
	ICM42688P(
		SPI_HandleTypeDef* spiPort,
		Buzzer *buzz,
		PID_Control& rollPID,
		PID_Control& pitchPID,
		PID_Control& yawPID);
	void SPI_write(uint8_t reg,uint8_t data);
	bool defaultInit();
	void update();
	uint8_t WhoAmI();
	int16_t getGyroX();
	int16_t getGyroY();
	int16_t getGyroZ();
	int16_t getAccX();
	int16_t getAccY();
	int16_t getAccZ();
	int16_t getTempX();
	uint8_t getIntStatus();
	void toEuler();
	float& getEulerXref();
	float& getEulerYref();
	float& getEulerZref();
	float getEulerX();
	float getEulerY();
	float getEulerZ();
	void computeGyroDrift(uint32_t count);
	void computeAccOffset(uint32_t count);
	void calibrate(uint32_t count);
	bool isCriticalStateDetected();
	bool isCrashDetected();
	const char* getSensorValues_str(std::set<SENSOR_DATA_PARAMETER> &senorsList);
};

#endif /* INC_ICM42688P_H_ */
