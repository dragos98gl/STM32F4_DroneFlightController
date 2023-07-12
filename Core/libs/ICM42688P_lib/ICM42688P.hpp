    /*
 * ICM42688P.h
 *
 *  Created on: Jul 22, 2022
 *      Author: DragosDarie
 */

#ifndef INC_ICM42688P_H_
#define INC_ICM42688P_H_

#include "HC05.hpp"
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
	static constexpr uint16_t criticalStateAngleThreshold = 20.0F;//10

	SPI_HandleTypeDef *spi_port;
	Buzzer *buzz;

	uint8_t spiTxBuff[2]={0U,0U};
	uint8_t spiRxBuff[2]={0U,0U};

	uint8_t SPI_read(uint8_t reg);

	bool initAndCheck(uint8_t addr,uint8_t val,uint8_t numberOfTries,bool read_only = false);
	void checkCriticalState();
	void checkCrashState();

	const float RADIANS_TO_DEGREES = 180.0F / M_PI;
	const float GYRO_FULLSCALE = 32768.0F / 2000.0F;
	const float DT = 1.0F / 1000.0F;

	float gx = 0.0F;
	float gy = 0.0F;
	float gz = 0.0F;
	float ax = 0.0F;
	float ay = 0.0F;
	float az = 0.0F;

	float raw_gx=0.0F;
	float raw_gy=0.0F;
	float raw_gz=0.0F;
	float raw_ax=0.0F;
	float raw_ay=0.0F;
	float raw_az=0.0F;
	float temp=0.0F;

	float euler_x = 0.0F;
	float euler_y = 0.0F;
	float euler_z = 0.0F;

	float gxDrift = -12.0F;
	float gyDrift = -13.0F;
	float gzDrift = 7.00F;
	float axOffset = 2135.0F / 2.0F;//2125.0F;
	float ayOffset = -850.0F / 2.0F;//-775.0F;
	float azOffset = 2570.0F / 2.0F;//2550.0F;
	float axScale = 2.0F;
	float ayScale = 2.0F;
	float azScale = 2.0F;

	float prev_raw_ax=0.0F;
	float prev_raw_ay=0.0F;
	float prev_raw_az=0.0F;

	float max_ax_dt = 0.0F;
	float max_ay_dt = 0.0F;
	float max_az_dt = 0.0F;
	bool crashState = false;
	bool criticalState = false;

	PID_Control& _rollPID;
	PID_Control& _pitchPID;
	PID_Control& _yawPID;
public:

	void SPI_write(uint8_t reg,uint8_t data);
	const char* getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList);
	ICM42688P(SPI_HandleTypeDef *spi_port,Buzzer *buzz, PID_Control& rollPID,PID_Control& pitchPID, PID_Control& yawPID);
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
};

#endif /* INC_ICM42688P_H_ */
