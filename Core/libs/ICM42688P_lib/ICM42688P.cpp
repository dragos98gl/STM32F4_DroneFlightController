/*
 * ICM42688P.cpp
 *
 *  Created on: Sep 17, 2022
 *      Author: Asus
 */

#include "ICM42688P.hpp"
#include "Constants.hpp"

ICM42688P::ICM42688P(
		SPI_HandleTypeDef* spiPort,
		Buzzer* buzz,
		PID_Control& rollPID,
		PID_Control& pitchPID,
		PID_Control& yawPID):
	_spiPort {spiPort}
	,_buzz {buzz}
	,_rollPID (rollPID)
	,_pitchPID (pitchPID)
	,_yawPID (yawPID)
	,_spiTxBuff {0U,0U}
	,_spiRxBuff {0U,0U}
	,_gx {0.0F}
	,_gy {0.0F}
	,_gz {0.0F}
	,_ax {0.0F}
	,_ay {0.0F}
	,_az {0.0F}
	,_rawGx {0.0F}
	,_rawGy {0.0F}
	,_rawGz {0.0F}
	,_rawAx {0.0F}
	,_rawAy {0.0F}
	,_rawAz {0.0F}
	,_temp {0.0F}
	,_eulerX {0.0F}
	,_eulerY {0.0F}
	,_eulerZ {0.0F}
	,_gxDrift {-12.0F}
	,_gyDrift {-13.0F}
	,_gzDrift {7.00F}
	,_axOffset {1030.0F}
	,_ayOffset {-974.0F / 2.0F}
	,_azOffset {2570.0F / 2.0F}
	,_axScale {2.0F}
	,_ayScale {2.0F}
	,_azScale {2.0F}
	,_prevRawAx {0.0F}
	,_prevRawAy {0.0F}
	,_prevRawAz {0.0F}
	,_maxAxDt {0.0F}
	,_maxAyDt {0.0F}
	,_maxAzDt {0.0F}
	,_crashState {false}
	,_criticalState {false}
{
}

bool ICM42688P::defaultInit()
{
	if (!initAndCheck(INTF_CONFIG1,0x00,10))
		return false;

	SPI_write(DEVICE_CONFIG,DEVICE_CONFIG_SOFT_RESET_CONFIG);
	HAL_Delay(20);

	if (!initAndCheck(INTF_CONFIG1,0x00,10))
		return false;

	if (!initAndCheck(INT_CONFIG0,INT_CONFIG0_UI_DRDY_INT_CLEAR_ONSENSORREGREAD,10))
		return false;

	if (!initAndCheck(INT_CONFIG,INT_CONFIG_INT1_POLARITY_ACTIVE_HIGH|INT_CONFIG_INT1_DRIVE_CIRCUIT_PUSH_PULL,10))
		return false;

	if (!initAndCheck(INT_SOURCE0,INT_SOURCE0_UI_DRDY_INT1_EN,10))
		return false;

	if (!initAndCheck(GYRO_CONFIG_STATIC2,0b11,10))
		return false;

	if (!initAndCheck(GYRO_CONFIG0,GYRO_CONFIG0_GYRO_ODR_1KHZ|GYRO_CONFIG0_GYRO_FS_SEL_2000DPS,10))
		return false;

	if (!initAndCheck(ACCEL_CONFIG0,ACCEL_CONFIG0_ACCEL_ODR_1KHZ|ACCEL_CONFIG0_ACCEL_FS_SEL_4G,10))
		return false;

	if (!initAndCheck(PWR_MGMT0,0x0F,10))
		return false;

	HAL_Delay(50);

	this->update();
	this->_eulerX = this->_ay;
	this->_eulerY = this->_ax;
	this->_eulerZ = this->_az;

	return true;
}

bool ICM42688P::initAndCheck(uint8_t addr,uint8_t val,uint8_t numberOfTries,bool read_only)
{
	for (int i=0;i<numberOfTries;i++)
	{
		if (read_only==false)
			SPI_write(addr,val);
		if (SPI_read(addr)==val)
			return true;
	}
	return false;
}

const char* ICM42688P::getSensorValues_str(std::set<SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(SENSOR_DATA_PARAMETER::ICM_RAW_GX)!=senorsList.end())
	{
		strcat(packet,toCharArray(_eulerX));
		strcat(packet,",");
	}

	if (senorsList.find(SENSOR_DATA_PARAMETER::ICM_RAW_GY)!=senorsList.end())
	{
		strcat(packet,toCharArray(_eulerY));
		strcat(packet,",");
	}

	if (senorsList.find(SENSOR_DATA_PARAMETER::ICM_RAW_GZ)!=senorsList.end())
	{
		strcat(packet,toCharArray(_eulerZ));
		strcat(packet,",");
	}

	if (senorsList.find(SENSOR_DATA_PARAMETER::ICM_RAW_AX)!=senorsList.end())
	{
		strcat(packet,toCharArray(-_ax));
		strcat(packet,",");
	}

	if (senorsList.find(SENSOR_DATA_PARAMETER::ICM_RAW_AY)!=senorsList.end())
	{
		strcat(packet,toCharArray(_ay));
		strcat(packet,",");
	}

	if (senorsList.find(SENSOR_DATA_PARAMETER::ICM_RAW_AZ)!=senorsList.end())
	{
		strcat(packet,toCharArray(_az));
		strcat(packet,",");
	}

	if (senorsList.find(SENSOR_DATA_PARAMETER::ICM_AX)!=senorsList.end())
	{
		strcat(packet,toCharArray(_maxAxDt));
		strcat(packet,",");
	}

	if (senorsList.find(SENSOR_DATA_PARAMETER::ICM_AY)!=senorsList.end())
	{
		strcat(packet,toCharArray(_maxAyDt));
		strcat(packet,",");
	}

	if (senorsList.find(SENSOR_DATA_PARAMETER::ICM_AZ)!=senorsList.end())
	{
		strcat(packet,toCharArray(_maxAzDt));
		strcat(packet,",");
	}

	return packet;
}

void ICM42688P::update()
{
	uint8_t axL = SPI_read(ACCEL_DATA_X0);
	uint8_t axH = SPI_read(ACCEL_DATA_X1);
	uint8_t ayL = SPI_read(ACCEL_DATA_Y0);
	uint8_t ayH = SPI_read(ACCEL_DATA_Y1);
	uint8_t azL = SPI_read(ACCEL_DATA_Z0);
	uint8_t azH = SPI_read(ACCEL_DATA_Z1);

	uint8_t gxL = SPI_read(GYRO_DATA_X0);
	uint8_t gxH = SPI_read(GYRO_DATA_X1);
	uint8_t gyL = SPI_read(GYRO_DATA_Y0);
	uint8_t gyH = SPI_read(GYRO_DATA_Y1);
	uint8_t gzL = SPI_read(GYRO_DATA_Z0);
	uint8_t gzH = SPI_read(GYRO_DATA_Z1);

	uint8_t tempL = SPI_read(TEMP_DATA0);
	uint8_t tempH = SPI_read(TEMP_DATA1);

	this->_rawAx = (static_cast<float>(((int16_t)(((int16_t)axH<<8) | axL))) - this->_axOffset);// * this->axScale;
	this->_rawAy = (static_cast<float>(((int16_t)(((int16_t)ayH<<8) | ayL))) - this->_ayOffset);// * this->ayScale;
	this->_rawAz = (static_cast<float>(((int16_t)(((int16_t)azH<<8) | azL))) - this->_azOffset);// * this->azScale;
	this->_rawGx = static_cast<float>((int16_t)(((int16_t)gxH<<8) | gxL)) - this->_gxDrift;
	this->_rawGy = static_cast<float>((int16_t)(((int16_t)gyH<<8) | gyL)) - this->_gyDrift;
	this->_rawGz = static_cast<float>((int16_t)(((int16_t)gzH<<8) | gzL)) - this->_gzDrift;
	this->_temp = static_cast<float>((int16_t)(((int16_t)tempH<<8) | tempL)) / 132.48F + 25.0F;

	this->toEuler();

	this->checkCrashState();
	this->checkCriticalState();

	this->_rollPID.update();
	this->_pitchPID.update();
	this->_yawPID.update();
}

void ICM42688P::checkCriticalState()
{
	if (abs(this->_eulerX) >=criticalStateAngleThreshold || abs(this->_eulerY) >=criticalStateAngleThreshold)
	{
		this->_criticalState = true;
	}
}

void ICM42688P::checkCrashState()
{
	if (this->_prevRawAx == 0.0F)
		this->_prevRawAx = this->_rawAx;

	if (this->_prevRawAy == 0.0F)
		this->_prevRawAy = this->_rawAy;

	if (this->_prevRawAz == 0.0F)
		this->_prevRawAz = this->_rawAz;

	float ax_dt = fabs(this->_rawAx-this->_prevRawAx);
	float ay_dt = fabs(this->_rawAy-this->_prevRawAy);
	float az_dt = fabs(this->_rawAz-this->_prevRawAz);

	if (ax_dt > constCrashAccDtThreshold ||
			ay_dt > constCrashAccDtThreshold ||
			az_dt > constCrashAccDtThreshold)
	{
		this->_crashState = true;
	}

	if (ax_dt > _maxAxDt)
		_maxAxDt = ax_dt;

	if (ay_dt > _maxAyDt)
		_maxAyDt = ay_dt;

	if (az_dt > _maxAzDt)
		_maxAzDt = az_dt;

	this->_prevRawAx=this->_rawAx;
	this->_prevRawAy=this->_rawAy;
	this->_prevRawAz=this->_rawAz;
}

bool ICM42688P::isCriticalStateDetected()
{
	return this->_criticalState;
}

bool ICM42688P::isCrashDetected()
{
	return this->_crashState;
}


void ICM42688P::toEuler()
{
	this->_gx = this->_gx + this->_rawGx*(DT/GYRO_FULLSCALE);
	this->_gy = this->_gy + this->_rawGy*(DT/GYRO_FULLSCALE);
	this->_gz = this->_gz + this->_rawGz*(DT/GYRO_FULLSCALE);

	this->_eulerX = this->_eulerX + this->_rawGx*(DT/GYRO_FULLSCALE);
	this->_eulerY = this->_eulerY + this->_rawGy*(DT/GYRO_FULLSCALE);
	this->_eulerZ = this->_eulerZ + this->_rawGz*(DT/GYRO_FULLSCALE);

	this->_ax = atan2(this->_rawAx,sqrt(this->_rawAy*this->_rawAy + this->_rawAz*this->_rawAz))*RADIANS_TO_DEGREES;
	this->_ay = atan2(this->_rawAy,sqrt(this->_rawAx*this->_rawAx + this->_rawAz*this->_rawAz))*RADIANS_TO_DEGREES;
	this->_az = atan2(this->_rawAz,sqrt(this->_rawAx*this->_rawAx + this->_rawAy*this->_rawAy))*RADIANS_TO_DEGREES -90.0F;

	this->_eulerX = this->_eulerX*0.9999+this->_ay*0.0001;
	this->_eulerY = this->_eulerY*0.9999-this->_ax*0.0001;
}

void ICM42688P::computeGyroDrift(uint32_t count)
{
	this->_gxDrift = 0.0F;
	this->_gyDrift = 0.0F;
	this->_gzDrift = 0.0F;

	for (uint32_t i=0;i < count;i++)
	{
		uint8_t gxL = SPI_read(GYRO_DATA_X0);
		uint8_t gxH = SPI_read(GYRO_DATA_X1);
		uint8_t gyL = SPI_read(GYRO_DATA_Y0);
		uint8_t gyH = SPI_read(GYRO_DATA_Y1);
		uint8_t gzL = SPI_read(GYRO_DATA_Z0);
		uint8_t gzH = SPI_read(GYRO_DATA_Z1);

		this->_rawGx = static_cast<float>((int16_t)(((int16_t)gxH<<8) | gxL));
		this->_rawGy = static_cast<float>((int16_t)(((int16_t)gyH<<8) | gyL));
		this->_rawGz = static_cast<float>((int16_t)(((int16_t)gzH<<8) | gzL));

		this->_gxDrift += this->_rawGx;
		this->_gyDrift += this->_rawGy;
		this->_gzDrift += this->_rawGz;
	}

	this->_gxDrift /= static_cast<float>(count);
	this->_gyDrift /= static_cast<float>(count);
	this->_gzDrift /= static_cast<float>(count);
}

void ICM42688P::computeAccOffset(uint32_t count)
{
	this->_axOffset = 0.0F;
	this->_ayOffset = 0.0F;
	this->_azOffset = 0.0F;

	for (uint32_t i=0;i < count;i++)
	{
		uint8_t axL = SPI_read(ACCEL_DATA_X0);
		uint8_t axH = SPI_read(ACCEL_DATA_X1);
		uint8_t ayL = SPI_read(ACCEL_DATA_Y0);
		uint8_t ayH = SPI_read(ACCEL_DATA_Y1);
		uint8_t azL = SPI_read(ACCEL_DATA_Z0);
		uint8_t azH = SPI_read(ACCEL_DATA_Z1);

		this->_rawAx = static_cast<float>(((int16_t)(((int16_t)axH<<8) | axL)));
		this->_rawAy = static_cast<float>(((int16_t)(((int16_t)ayH<<8) | ayL)));
		this->_rawAz = static_cast<float>(((int16_t)(((int16_t)azH<<8) | azL)));

		this->_axOffset += this->_rawAx;
		this->_ayOffset += this->_rawAy;
		this->_azOffset += this->_rawAz;
	}

	this->_axOffset /= static_cast<float>(count);
	this->_ayOffset /= static_cast<float>(count);
	this->_azOffset = this->_azOffset / static_cast<float>(count) - 8192.0F;
}

void ICM42688P::calibrate(uint32_t count)
{
	this->_buzz->beep(3000U,100U,2U);
	this->computeGyroDrift(count);
	this->computeAccOffset(count);
	this->_buzz->beep(200U,100U,1U);
}

float ICM42688P::getEulerX()
{
	return this->_eulerX;
}

float& ICM42688P::getEulerXref()
{
	return this->_eulerX;
}

float ICM42688P::getEulerY()
{
	return this->_eulerY;
}

float& ICM42688P::getEulerYref()
{
	return this->_eulerY;
}

float ICM42688P::getEulerZ()
{
	return this->_eulerZ;
}

float& ICM42688P::getEulerZref()
{
	return this->_eulerZ;
}

uint8_t ICM42688P::WhoAmI()
{
	return this->SPI_read(WHO_AM_I);
}

int16_t ICM42688P::getGyroX()
{
	return this->_gx;
}

int16_t ICM42688P::getGyroY()
{
	return this->_gy;
}

int16_t ICM42688P::getGyroZ()
{
	return this->_gz;
}

int16_t ICM42688P::getAccX()
{
	return this->_rawAx;
}

int16_t ICM42688P::getAccY()
{
	return this->_ay;
}

int16_t ICM42688P::getAccZ()
{
	return this->_az;
}

int16_t ICM42688P::getTempX()
{
	return this->_temp;
}

uint8_t ICM42688P::getIntStatus()
{
	return this->SPI_read(INT_STATUS);
}

void ICM42688P::SPI_write(uint8_t reg,uint8_t data)
{
	HAL_GPIO_WritePin(ICM_CS_PORT,ICM_CS_PIN,GPIO_PIN_RESET);
	this->_spiTxBuff[0] = reg;
	this->_spiTxBuff[1] = data;
	HAL_SPI_Transmit_DMA(_spiPort, (uint8_t*)_spiTxBuff,2);
	HAL_GPIO_WritePin(ICM_CS_PORT,ICM_CS_PIN,GPIO_PIN_SET);
}

uint8_t ICM42688P::SPI_read(uint8_t reg)
{
	HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
	this->_spiTxBuff[0]=reg|0x80;

	HAL_SPI_Transmit_DMA(this->_spiPort, (uint8_t*)_spiTxBuff, 1);
	HAL_SPI_Receive_DMA(this->_spiPort, (uint8_t*)_spiRxBuff, 1);
	HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);

	return this->_spiRxBuff[0];
}
