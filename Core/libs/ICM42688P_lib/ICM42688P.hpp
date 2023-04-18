    /*
 * ICM42688P.h
 *
 *  Created on: Jul 22, 2022
 *      Author: DragosDarie
 */

#ifndef INC_ICM42688P_H_
#define INC_ICM42688P_H_

#include <HC05.hpp>
#include <Interfaces.hpp>
#include "ICM42688P_reg.h"
#include "FlashReadWrite.hpp"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "string.h"
#include <set>

class ICM42688P: SPI_Conn,public PrintableSensor
{
private:
	SPI_HandleTypeDef *spi_port;

	uint8_t spiTxBuff[2]={0,0};
	uint8_t spiRxBuff[2]={0,0};

	uint8_t SPI_read(uint8_t reg);

	bool initAndCheck(uint8_t addr,uint8_t val,uint8_t numberOfTries,bool read_only = false);

	const float RADIANS_TO_DEGREES = 180.0 / M_PI;
	const float GYRO_FULLSCALE = 32768.0/2000.0;
	const float DT = 1.0/1000.0;

	uint8_t axH;
	uint8_t axL;
	uint8_t ayH;
	uint8_t ayL;
	uint8_t azH;
	uint8_t azL;
	uint8_t gxH;
	uint8_t gxL;
	uint8_t gyH;
	uint8_t gyL;
	uint8_t gzH;
	uint8_t gzL;
	uint8_t tempH;
	uint8_t tempL;

	float gx = 0;
	float gy = 0;
	float gz = 0;
	float ax = 0;
	float ay = 0;
	float az = 0;

	float raw_gx=0;
	float raw_gy=0;
	float raw_gz=0;
	float raw_ax=0;
	float raw_ay=0;
	float raw_az=0;
	float temp=0;

	float euler_x = 0;
	float euler_y = 0;
	float euler_z = 0;

	float gxDrift = -9.3F;
	float gyDrift = -11.3F;
	float gzDrift = 7.68F;
	float axOffset = 2135.0F;//2125.0F;
	float ayOffset = -850.0F;//-775.0F;
	float azOffset = 2570.0F;//2550.0F;
	float axScale = 2.0F;
	float ayScale = 2.0F;
	float azScale = 2.0F;
public:

	void SPI_write(uint8_t reg,uint8_t data);
	const char* getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList);
	ICM42688P(SPI_HandleTypeDef *spi_port);
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
	float getEulerX();
	float getEulerY();
	float getEulerZ();
	bool selfTest();
	void computeGyroDrift(uint32_t count);
	void computeAccOffset(uint32_t count);
	void calibrate(uint32_t count);
};

#endif /* INC_ICM42688P_H_ */
