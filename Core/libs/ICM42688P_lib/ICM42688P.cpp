/*
 * ICM42688P.cpp
 *
 *  Created on: Sep 17, 2022
 *      Author: Asus
 */

#include "ICM42688P.hpp"
#include "Constants.hpp"

ICM42688P::ICM42688P(SPI_HandleTypeDef *spi_port, Buzzer *buzz, PID_Control& rollPID,PID_Control& pitchPID, PID_Control& yawPID):
	_rollPID(rollPID),
	_pitchPID(pitchPID),
	_yawPID(yawPID)
{
	ICM42688P::spi_port = spi_port;
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
	this->euler_x = this->ay;
	this->euler_y = this->ax;
	this->euler_z = this->az;

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

const char* ICM42688P::getSensorValues_str(std::set<HC05::SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_RAW_GX)!=senorsList.end())
	{
		strcat(packet,toCharArray(euler_x));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_RAW_GY)!=senorsList.end())
	{
		strcat(packet,toCharArray(euler_y));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_RAW_GZ)!=senorsList.end())
	{
		strcat(packet,toCharArray(euler_z));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_RAW_AX)!=senorsList.end())
	{
		strcat(packet,toCharArray(-ax));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_RAW_AY)!=senorsList.end())
	{
		strcat(packet,toCharArray(ay));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_RAW_AZ)!=senorsList.end())
	{
		strcat(packet,toCharArray(az));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_AX)!=senorsList.end())
	{
		strcat(packet,toCharArray(max_ax_dt));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_AY)!=senorsList.end())
	{
		strcat(packet,toCharArray(max_ay_dt));
		strcat(packet,",");
	}

	if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_AZ)!=senorsList.end())
	{
		strcat(packet,toCharArray(max_az_dt));
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

	this->raw_ax = (static_cast<float>(((int16_t)(((int16_t)axH<<8) | axL))) - this->axOffset);// * this->axScale;
	this->raw_ay = (static_cast<float>(((int16_t)(((int16_t)ayH<<8) | ayL))) - this->ayOffset);// * this->ayScale;
	this->raw_az = (static_cast<float>(((int16_t)(((int16_t)azH<<8) | azL))) - this->azOffset);// * this->azScale;
	this->raw_gx = static_cast<float>((int16_t)(((int16_t)gxH<<8) | gxL)) - this->gxDrift;
	this->raw_gy = static_cast<float>((int16_t)(((int16_t)gyH<<8) | gyL)) - this->gyDrift;
	this->raw_gz = static_cast<float>((int16_t)(((int16_t)gzH<<8) | gzL)) - this->gzDrift;
	this->temp = static_cast<float>((int16_t)(((int16_t)tempH<<8) | tempL)) / 132.48F + 25.0F;

	this->toEuler();

	this->checkCrashState();
	this->checkCriticalState();

	this->_rollPID.update();
	this->_pitchPID.update();
	this->_yawPID.update();
}

void ICM42688P::checkCriticalState()
{
	if (abs(this->euler_x) >=criticalStateAngleThreshold || abs(this->euler_y) >=criticalStateAngleThreshold)
	{
		this->criticalState = true;
	}
}

void ICM42688P::checkCrashState()
{
	if (this->prev_raw_ax == 0.0F)
		this->prev_raw_ax = this->raw_ax;

	if (this->prev_raw_ay == 0.0F)
		this->prev_raw_ay = this->raw_ay;

	if (this->prev_raw_az == 0.0F)
		this->prev_raw_az = this->raw_az;

	float ax_dt = fabs(this->raw_ax-this->prev_raw_ax);
	float ay_dt = fabs(this->raw_ay-this->prev_raw_ay);
	float az_dt = fabs(this->raw_az-this->prev_raw_az);

	if (ax_dt > constCrashAccDtThreshold ||
			ay_dt > constCrashAccDtThreshold ||
			az_dt > constCrashAccDtThreshold)
	{
		this->crashState = true;
	}

	if (ax_dt > max_ax_dt)
		max_ax_dt = ax_dt;

	if (ay_dt > max_ay_dt)
		max_ay_dt = ay_dt;

	if (az_dt > max_az_dt)
		max_az_dt = az_dt;

	this->prev_raw_ax=this->raw_ax;
	this->prev_raw_ay=this->raw_ay;
	this->prev_raw_az=this->raw_az;
}

bool ICM42688P::isCriticalStateDetected()
{
	return this->criticalState;
}

bool ICM42688P::isCrashDetected()
{
	return this->crashState;
}


void ICM42688P::toEuler()
{
	this->gx = this->gx + this->raw_gx*(DT/GYRO_FULLSCALE);
	this->gy = this->gy + this->raw_gy*(DT/GYRO_FULLSCALE);
	this->gz = this->gz + this->raw_gz*(DT/GYRO_FULLSCALE);

	this->euler_x = this->euler_x + this->raw_gx*(DT/GYRO_FULLSCALE);
	this->euler_y = this->euler_y + this->raw_gy*(DT/GYRO_FULLSCALE);
	this->euler_z = this->euler_z + this->raw_gz*(DT/GYRO_FULLSCALE);

	this->ax = atan2(this->raw_ax,sqrt(this->raw_ay*this->raw_ay + this->raw_az*this->raw_az))*RADIANS_TO_DEGREES;
	this->ay = atan2(this->raw_ay,sqrt(this->raw_ax*this->raw_ax + this->raw_az*this->raw_az))*RADIANS_TO_DEGREES;
	this->az = atan2(this->raw_az,sqrt(this->raw_ax*this->raw_ax + this->raw_ay*this->raw_ay))*RADIANS_TO_DEGREES -90.0F;

	this->euler_x = this->euler_x*0.9999+this->ay*0.0001;
	this->euler_y = this->euler_y*0.9999-this->ax*0.0001;
}

void ICM42688P::computeGyroDrift(uint32_t count)
{
	this->gxDrift = 0.0F;
	this->gyDrift = 0.0F;
	this->gzDrift = 0.0F;

	for (uint32_t i=0;i < count;i++)
	{
		uint8_t gxL = SPI_read(GYRO_DATA_X0);
		uint8_t gxH = SPI_read(GYRO_DATA_X1);
		uint8_t gyL = SPI_read(GYRO_DATA_Y0);
		uint8_t gyH = SPI_read(GYRO_DATA_Y1);
		uint8_t gzL = SPI_read(GYRO_DATA_Z0);
		uint8_t gzH = SPI_read(GYRO_DATA_Z1);

		this->raw_gx = static_cast<float>((int16_t)(((int16_t)gxH<<8) | gxL));
		this->raw_gy = static_cast<float>((int16_t)(((int16_t)gyH<<8) | gyL));
		this->raw_gz = static_cast<float>((int16_t)(((int16_t)gzH<<8) | gzL));

		this->gxDrift += this->raw_gx;
		this->gyDrift += this->raw_gy;
		this->gzDrift += this->raw_gz;
	}

	this->gxDrift /= static_cast<float>(count);
	this->gyDrift /= static_cast<float>(count);
	this->gzDrift /= static_cast<float>(count);
}

void ICM42688P::computeAccOffset(uint32_t count)
{
	this->axOffset = 0.0F;
	this->ayOffset = 0.0F;
	this->azOffset = 0.0F;

	for (uint32_t i=0;i < count;i++)
	{
		uint8_t axL = SPI_read(ACCEL_DATA_X0);
		uint8_t axH = SPI_read(ACCEL_DATA_X1);
		uint8_t ayL = SPI_read(ACCEL_DATA_Y0);
		uint8_t ayH = SPI_read(ACCEL_DATA_Y1);
		uint8_t azL = SPI_read(ACCEL_DATA_Z0);
		uint8_t azH = SPI_read(ACCEL_DATA_Z1);

		this->raw_ax = static_cast<float>(((int16_t)(((int16_t)axH<<8) | axL)));
		this->raw_ay = static_cast<float>(((int16_t)(((int16_t)ayH<<8) | ayL)));
		this->raw_az = static_cast<float>(((int16_t)(((int16_t)azH<<8) | azL)));

		this->axOffset += this->raw_ax;
		this->ayOffset += this->raw_ay;
		this->azOffset += this->raw_az;
	}

	this->axOffset /= static_cast<float>(count);
	this->ayOffset /= static_cast<float>(count);
	this->azOffset = this->azOffset / static_cast<float>(count) - 8192.0F;
}

void ICM42688P::calibrate(uint32_t count)
{
	this->buzz->beep(3000U,100U,2U);
	this->computeGyroDrift(count);
	this->computeAccOffset(count);
	this->buzz->beep(200U,100U,1U);
}

float ICM42688P::getEulerX()
{
	return this->euler_x;
}

float& ICM42688P::getEulerXref()
{
	return this->euler_x;
}

float ICM42688P::getEulerY()
{
	return this->euler_y;
}

float& ICM42688P::getEulerYref()
{
	return this->euler_y;
}

float ICM42688P::getEulerZ()
{
	return this->euler_z;
}

float& ICM42688P::getEulerZref()
{
	return this->euler_z;
}

uint8_t ICM42688P::WhoAmI()
{
	return this->SPI_read(WHO_AM_I);
}

int16_t ICM42688P::getGyroX()
{
	return this->gx;
}

int16_t ICM42688P::getGyroY()
{
	return this->gy;
}

int16_t ICM42688P::getGyroZ()
{
	return this->gz;
}

int16_t ICM42688P::getAccX()
{
	return this->raw_ax;
}

int16_t ICM42688P::getAccY()
{
	return this->ay;
}

int16_t ICM42688P::getAccZ()
{
	return this->az;
}

int16_t ICM42688P::getTempX()
{
	return this->temp;
}

uint8_t ICM42688P::getIntStatus()
{
	return this->SPI_read(INT_STATUS);
}

void ICM42688P::SPI_write(uint8_t reg,uint8_t data)
{
	HAL_GPIO_WritePin(ICM_CS_PORT,ICM_CS_PIN,GPIO_PIN_RESET);
	this->spiTxBuff[0] = reg;
	this->spiTxBuff[1] = data;
	HAL_SPI_Transmit_DMA(spi_port, (uint8_t*)spiTxBuff,2);
	HAL_GPIO_WritePin(ICM_CS_PORT,ICM_CS_PIN,GPIO_PIN_SET);
}

uint8_t ICM42688P::SPI_read(uint8_t reg)
{
	HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
	this->spiTxBuff[0]=reg|0x80;

	HAL_SPI_Transmit_DMA(this->spi_port, (uint8_t*)spiTxBuff, 1);
	HAL_SPI_Receive_DMA(this->spi_port, (uint8_t*)spiRxBuff, 1);
	HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);

	return this->spiRxBuff[0];
}
