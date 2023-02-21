/*
 * ICM42688P.cpp
 *
 *  Created on: Sep 17, 2022
 *      Author: Asus
 */

#include "ICM42688P.h"

ICM42688P::ICM42688P(SPI_HandleTypeDef *spi_port)
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

	if (!initAndCheck(ACCEL_CONFIG0,ACCEL_CONFIG0_ACCEL_ODR_1KHZ|ACCEL_CONFIG0_ACCEL_FS_SEL_2G,10))
		return false;

	if (!initAndCheck(PWR_MGMT0,0x0F,10))
		return false;

	HAL_Delay(50);

	//computeGyroDrift(1000);

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

	//if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_RAW_GX)!=senorsList.end())
	{
		strcat(packet,toCharArray(euler_x));
		strcat(packet,",");
	}

	//if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_RAW_GY)!=senorsList.end())
	{
		strcat(packet,toCharArray(euler_y));
		strcat(packet,",");
	}

	//if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_RAW_GZ)!=senorsList.end())
	{
		strcat(packet,toCharArray(euler_z));
		strcat(packet,",");
	}

	//if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_RAW_AX)!=senorsList.end())
	{
		strcat(packet,toCharArray(-ax));
		strcat(packet,",");
	}

	//if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_RAW_AY)!=senorsList.end())
	{
		strcat(packet,toCharArray(ay));
		strcat(packet,",");
	}

	//if (senorsList.find(HC05::SENSOR_DATA_PARAMETER::ICM_RAW_AZ)!=senorsList.end())
	{
		strcat(packet,toCharArray(az));
		strcat(packet,",");
	}

	return packet;
}

void ICM42688P::update()
{
	axL = SPI_read(ACCEL_DATA_X0);
	axH = SPI_read(ACCEL_DATA_X1);

	ayL = SPI_read(ACCEL_DATA_Y0);
	ayH = SPI_read(ACCEL_DATA_Y1);

	azL = SPI_read(ACCEL_DATA_Z0);
	azH = SPI_read(ACCEL_DATA_Z1);

	gxL = SPI_read(GYRO_DATA_X0);
	gxH = SPI_read(GYRO_DATA_X1);

	gyL = SPI_read(GYRO_DATA_Y0);
	gyH = SPI_read(GYRO_DATA_Y1);

	gzL = SPI_read(GYRO_DATA_Z0);
	gzH = SPI_read(GYRO_DATA_Z1);

	tempL = SPI_read(TEMP_DATA0);
	tempH = SPI_read(TEMP_DATA1);

	raw_ax = ((int16_t)(((int16_t)axH<<8) | axL) - axOffset)*axScale;
	raw_ay = ((int16_t)(((int16_t)ayH<<8) | ayL) - ayOffset)*ayScale;
	raw_az = ((int16_t)(((int16_t)azH<<8) | azL) - azOffset)*azScale;
	raw_gx = (int16_t)(((int16_t)gxH<<8) | gxL) - gxDrift;
	raw_gy = (int16_t)(((int16_t)gyH<<8) | gyL) - gyDrift;
	raw_gz = (int16_t)(((int16_t)gzH<<8) | gzL) - gzDrift;
	temp = (int16_t)(((int16_t)tempH<<8) | tempL)/132.48+25;

	toEuler();

	SPI_read(INT_STATUS);
}

void ICM42688P::toEuler()
{
	gx = gx + raw_gx*(DT/GYRO_FULLSCALE);
	gy = gy + raw_gy*(DT/GYRO_FULLSCALE);
	gz = gz + raw_gz*(DT/GYRO_FULLSCALE);

	euler_x = euler_x + raw_gx*(DT/GYRO_FULLSCALE);
	euler_y = euler_y + raw_gy*(DT/GYRO_FULLSCALE);
	euler_z = euler_z + raw_gz*(DT/GYRO_FULLSCALE);

	ax = atan2(raw_ax,sqrt(raw_ay*raw_ay + raw_az*raw_az))*RADIANS_TO_DEGREES;
	ay = atan2(raw_ay,sqrt(raw_ax*raw_ax + raw_az*raw_az))*RADIANS_TO_DEGREES;
	az = atan2(raw_az,sqrt(raw_ax*raw_ax + raw_ay*raw_ay))* RADIANS_TO_DEGREES;

	euler_x = euler_x*0.9999+ay*0.0001;
	euler_y = euler_y*0.9999-ax*0.0001;
	euler_z = euler_z*0.9999+az*0.0001;
}

void ICM42688P::computeGyroDrift(uint16_t count)
{
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

	for (int i=0;i<count;i++)
	{
		gxL = SPI_read(GYRO_DATA_X0);
		gxH = SPI_read(GYRO_DATA_X1);
		gyL = SPI_read(GYRO_DATA_Y0);
		gyH = SPI_read(GYRO_DATA_Y1);
		gzL = SPI_read(GYRO_DATA_Z0);
		gzH = SPI_read(GYRO_DATA_Z1);

		axL = SPI_read(ACCEL_DATA_X0);
		axH = SPI_read(ACCEL_DATA_X1);
		ayL = SPI_read(ACCEL_DATA_Y0);
		ayH = SPI_read(ACCEL_DATA_Y1);
		azL = SPI_read(ACCEL_DATA_Z0);
		azH = SPI_read(ACCEL_DATA_Z1);

		raw_ax = (((int16_t)axH<<8) | axL);
		raw_ay = (((int16_t)ayH<<8) | ayL);
		raw_az = (((int16_t)azH<<8) | azL);

		gxDrift += (int16_t)(((int16_t)gxH<<8) | gxL);
		gyDrift += (int16_t)(((int16_t)gyH<<8) | gyL);
		gzDrift += (int16_t)(((int16_t)gzH<<8) | gzL);
	}

	euler_x = atan2(raw_ax,raw_az)*RADIANS_TO_DEGREES;
	euler_y = atan2(-raw_ax,sqrt(raw_ay*raw_ay + raw_az*raw_az))*RADIANS_TO_DEGREES;

	gxDrift/=count;
	gyDrift/=count;
	gzDrift/=count;
}

bool ICM42688P::selfTest()
{

	if (!initAndCheck(SELF_TEST_CONFIG,0b1111111,10))
		return false;

	HAL_Delay(500);

	SPI_write(REG_BANK_SEL,0b001);
	uint8_t xg_st_data = SPI_read(XG_ST_DATA);
	SPI_write(REG_BANK_SEL,0b001);
	uint8_t yg_st_data = SPI_read(YG_ST_DATA);
	SPI_write(REG_BANK_SEL,0b001);
	uint8_t zg_st_data = SPI_read(ZG_ST_DATA);
	SPI_write(REG_BANK_SEL,0b001);
	uint8_t xa_st_data = SPI_read(XA_ST_DATA);
	SPI_write(REG_BANK_SEL,0b001);
	uint8_t ya_st_data = SPI_read(YA_ST_DATA);
	SPI_write(REG_BANK_SEL,0b001);
	uint8_t za_st_data = SPI_read(ZA_ST_DATA);

	SPI_write(REG_BANK_SEL,0b000);
	if (!initAndCheck(SELF_TEST_CONFIG,0b0000000,10))
		return false;

	return true;
}

float ICM42688P::getEulerX()
{
	return euler_x;
}

float ICM42688P::getEulerY()
{
	return euler_y;
}

float ICM42688P::getEulerZ()
{
	return euler_z;
}

uint8_t ICM42688P::WhoAmI()
{
	return SPI_read(WHO_AM_I);
}

int16_t ICM42688P::getGyroX()
{
	return gx;
}

int16_t ICM42688P::getGyroY()
{
	return gy;
}

int16_t ICM42688P::getGyroZ()
{
	return gz;
}

int16_t ICM42688P::getAccX()
{
	return raw_ax;
}

int16_t ICM42688P::getAccY()
{
	return ay;
}

int16_t ICM42688P::getAccZ()
{
	return az;
}

int16_t ICM42688P::getTempX()
{
	return temp;
}

uint8_t ICM42688P::getIntStatus()
{
	return SPI_read(INT_STATUS);
}

void ICM42688P::SPI_write(uint8_t reg,uint8_t data)
{
	HAL_GPIO_WritePin(ICM_CS_PORT,ICM_CS_PIN,GPIO_PIN_RESET);
	spiTxBuff[0] = reg;
	spiTxBuff[1] = data;
	HAL_SPI_Transmit_DMA(spi_port, (uint8_t*)spiTxBuff,2);
	HAL_GPIO_WritePin(ICM_CS_PORT,ICM_CS_PIN,GPIO_PIN_SET);
}

uint8_t ICM42688P::SPI_read(uint8_t reg)
{
	HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
	spiTxBuff[0]=reg|0x80;

	HAL_SPI_Transmit_DMA(spi_port, (uint8_t*)spiTxBuff, 1);
	HAL_SPI_Receive_DMA(spi_port, (uint8_t*)spiRxBuff, 1);
	HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);

	return spiRxBuff[0];
}
