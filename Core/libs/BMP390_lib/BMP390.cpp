/*
 * BMP390.cpp
 *
 *  Created on: Sep 21, 2022
 *      Author: Asus
 */

#include "BMP390.hpp"

BMP390::BMP390(SPI_HandleTypeDef* spiPort):
	_spiPort {spiPort}
	,_spiTxBuff {0U,0U}
	,_spiRxBuff {0U,0U}
	,_pressure {0.0}
	,_temp {0.0}
	,_rawPressure {0U}
	,_rawTemp {0U}
{
}

bool BMP390::defaultInit()
{
	this->SPI_write(CMD,CMD_SOFTRESET);
	HAL_Delay(20);

	int debug = SPI_read(STATUS);
	while ((debug= SPI_read(STATUS))==0)
		HAL_Delay(50);

	if (!this->initAndCheck(OSR,OSR_OSR_P_X16|OSR_OSR_T_X2,10))
		return false;

	if (!this->initAndCheck(CONFIG,CONFIG_COEF_3,10))
		return false;

	if (!this->initAndCheck(ODR,ODR_ODR_25,10))
		return false;

	if (!this->initAndCheck(INT_CTRL,INT_CTRL_DRDY_EN|INT_CTRL_INT_LEVEL,10))
		return false;

	if (!this->initAndCheck(PWR_CTRL,PWR_CTRL_PRESS_EN|PWR_CTRL_TEMP_EN|PWR_CTRL_MODE_NORMAL,10))
		return false;

	return true;
}

bool BMP390::initAndCheck(uint8_t addr,uint8_t val,uint8_t numberOfTries,bool read_only)
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

void BMP390::update()
{
	uint8_t DATA0=SPI_read(DATA_0);
	uint8_t DATA1=SPI_read(DATA_1);
	uint8_t DATA2=SPI_read(DATA_2);

	uint8_t TEMP0=SPI_read(DATA_3);
	uint8_t TEMP1=SPI_read(DATA_4);
	uint8_t TEMP2=SPI_read(DATA_5);

	this->_rawPressure = ((uint32_t)DATA2<<16)|((uint16_t)DATA1<<8)|DATA0;
	this->_rawTemp = ((uint32_t)TEMP2<<16)|((uint16_t)TEMP1<<8)|TEMP0;

	this->read_calib_data();
	this->compensate_data();
}

void BMP390::compensate_data()
{
	this->read_calib_data();
	this->compensate_temperature();
	this->compensate_pressure();
}

void BMP390::compensate_temperature()
{
    double partial_data1 = 0;
    double partial_data2 = 0;

    partial_data1 = static_cast<double>(this->_rawTemp) - quantizedCalibCoef.par_t1;
    partial_data2 = partial_data1 * quantizedCalibCoef.par_t2;

    this->quantizedCalibCoef.t_lin = partial_data2 + (partial_data1 * partial_data1) * quantizedCalibCoef.par_t3;

    this->_temp = static_cast<double>(quantizedCalibCoef.t_lin);
}

void BMP390::compensate_pressure()
{
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;

    partial_data1 = this->quantizedCalibCoef.par_p6 * this->quantizedCalibCoef.t_lin;
    partial_data2 = this->quantizedCalibCoef.par_p7 * (this->quantizedCalibCoef.t_lin * this->quantizedCalibCoef.t_lin);
    partial_data3 = this->quantizedCalibCoef.par_p8 * (this->quantizedCalibCoef.t_lin * this->quantizedCalibCoef.t_lin * this->quantizedCalibCoef.t_lin);
    partial_out1 = this->quantizedCalibCoef.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = this->quantizedCalibCoef.par_p2 * this->quantizedCalibCoef.t_lin;
    partial_data2 = this->quantizedCalibCoef.par_p3 * (this->quantizedCalibCoef.t_lin * this->quantizedCalibCoef.t_lin);
    partial_data3 = this->quantizedCalibCoef.par_p4 * (this->quantizedCalibCoef.t_lin * this->quantizedCalibCoef.t_lin * this->quantizedCalibCoef.t_lin);
    partial_out2 = static_cast<double>(this->_rawPressure) * (this->quantizedCalibCoef.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = static_cast<double>(this->_rawPressure) * static_cast<double>(this->_rawPressure);
    partial_data2 = this->quantizedCalibCoef.par_p9 + this->quantizedCalibCoef.par_p10 * this->quantizedCalibCoef.t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + (static_cast<double>(this->_rawPressure) * static_cast<double>(this->_rawPressure) * static_cast<double>(this->_rawPressure)) * this->quantizedCalibCoef.par_p11;

    this->_pressure = (partial_out1 + partial_out2 + partial_data4) / 100.0f;
}

const char* BMP390::getSensorValues_str(std::set<SENSOR_DATA_PARAMETER> &senorsList)
{
	strcpy(packet,"");

	if (senorsList.find(SENSOR_DATA_PARAMETER::BMP_RAW_PRESS)!=senorsList.end())
	{
		strcat(packet,toCharArray(_pressure));
		strcat(packet,",");
	}

	return packet;
}

int32_t BMP390::getPressure()
{
	return this->_pressure;
}

int32_t BMP390::getTemp()
{
	return this->_temp;
}

uint8_t BMP390::getChipID()
{
	return this->SPI_read(CHIP_ID);
}

void BMP390::SPI_write(uint8_t reg,uint8_t data)
{
	HAL_GPIO_WritePin(BMP_CS_PORT,BMP_CS_PIN,GPIO_PIN_RESET);
	this->_spiTxBuff[0] = reg;
	this->_spiTxBuff[1] = data;
	HAL_SPI_Transmit_DMA(this->_spiPort, (uint8_t*)_spiTxBuff,2);
	HAL_GPIO_WritePin(BMP_CS_PORT,BMP_CS_PIN,GPIO_PIN_SET);
}

uint8_t BMP390::SPI_read(uint8_t reg)
{
	HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_RESET);
	this->_spiTxBuff[0]=reg|0x80;
	this->_spiTxBuff[1]=0x00;
	HAL_SPI_Transmit_DMA(this->_spiPort, (uint8_t*)_spiTxBuff, 2);
	HAL_SPI_Receive_DMA(this->_spiPort, (uint8_t*)_spiRxBuff, 1);
	HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_SET);

	return this->_spiRxBuff[0];
}

void BMP390::read_calib_data()
{
	double temp_var = 0;

	uint8_t NVM_PAR_T1_1_value = this->SPI_read(NVM_PAR_T1_1);
	uint8_t NVM_PAR_T1_2_value = this->SPI_read(NVM_PAR_T1_2);
	uint8_t NVM_PAR_T2_1_value = this->SPI_read(NVM_PAR_T2_1);
	uint8_t NVM_PAR_T2_2_value = this->SPI_read(NVM_PAR_T2_2);
	uint8_t NVM_PAR_T3_value = this->SPI_read(NVM_PAR_T3);
	uint8_t NVM_PAR_P1_1_value = this->SPI_read(NVM_PAR_P1_1);
	uint8_t NVM_PAR_P1_2_value = this->SPI_read(NVM_PAR_P1_2);
	uint8_t NVM_PAR_P2_1_value = this->SPI_read(NVM_PAR_P2_1);
	uint8_t NVM_PAR_P2_2_value = this->SPI_read(NVM_PAR_P2_2);
	uint8_t NVM_PAR_P3_value = this->SPI_read(NVM_PAR_P3);
	uint8_t NVM_PAR_P4_value = this->SPI_read(NVM_PAR_P4);
	uint8_t NVM_PAR_P5_1_value = this->SPI_read(NVM_PAR_P5_1);
	uint8_t NVM_PAR_P5_2_value = this->SPI_read(NVM_PAR_P5_2);
	uint8_t NVM_PAR_P6_1_value = this->SPI_read(NVM_PAR_P6_1);
	uint8_t NVM_PAR_P6_2_value = this->SPI_read(NVM_PAR_P6_2);
	uint8_t NVM_PAR_P7_value = this->SPI_read(NVM_PAR_P7);
	uint8_t NVM_PAR_P8_value = this->SPI_read(NVM_PAR_P8);
	uint8_t NVM_PAR_P9_1_value = this->SPI_read(NVM_PAR_P9_1);
	uint8_t NVM_PAR_P9_2_value = this->SPI_read(NVM_PAR_P9_2);
	uint8_t NVM_PAR_P10_value = this->SPI_read(NVM_PAR_P10);
	uint8_t NVM_PAR_P11_value = this->SPI_read(NVM_PAR_P11);

    this->calibCoef.par_t1 = drone::utils::functions::concatBytes(NVM_PAR_T1_2_value, NVM_PAR_T1_1_value);
    this->calibCoef.par_t2 = drone::utils::functions::concatBytes(NVM_PAR_T2_2_value, NVM_PAR_T2_1_value);
    this->calibCoef.par_t3 = static_cast<int8_t>(NVM_PAR_T3_value);
    this->calibCoef.par_p1 = drone::utils::functions::concatBytes(NVM_PAR_P1_2_value, NVM_PAR_P1_1_value);
    this->calibCoef.par_p2 = drone::utils::functions::concatBytes(NVM_PAR_P2_2_value, NVM_PAR_P2_1_value);
    this->calibCoef.par_p3 = static_cast<int8_t>(NVM_PAR_P3_value);
    this->calibCoef.par_p4 = static_cast<int8_t>(NVM_PAR_P4_value);
    this->calibCoef.par_p5 = drone::utils::functions::concatBytes(NVM_PAR_P5_2_value, NVM_PAR_P5_1_value);
    this->calibCoef.par_p6 = drone::utils::functions::concatBytes(NVM_PAR_P6_2_value, NVM_PAR_P6_1_value);
    this->calibCoef.par_p7 = static_cast<int8_t>(NVM_PAR_P7_value);
    this->calibCoef.par_p8 = static_cast<int8_t>(NVM_PAR_P8_value);
    this->calibCoef.par_p9 = drone::utils::functions::concatBytes(NVM_PAR_P9_2_value, NVM_PAR_P9_1_value);
    this->calibCoef.par_p10 = static_cast<int8_t>(NVM_PAR_P10_value);
    this->calibCoef.par_p11 = static_cast<int8_t>(NVM_PAR_P11_value);

    temp_var = 0.00390625f;
    this->quantizedCalibCoef.par_t1 = static_cast<double>(this->calibCoef.par_t1) / temp_var;
    temp_var = 1073741824.0f;
    this->quantizedCalibCoef.par_t2 = static_cast<double>(this->calibCoef.par_t2) / temp_var;
    temp_var = 281474976710656.0f;
    this->quantizedCalibCoef.par_t3 = static_cast<double>(this->calibCoef.par_t3) / temp_var;
    temp_var = 1048576.0f;
    this->quantizedCalibCoef.par_p1 = static_cast<double>(this->calibCoef.par_p1 - 16384) / temp_var;
    temp_var = 536870912.0f;
    this->quantizedCalibCoef.par_p2 = static_cast<double>(this->calibCoef.par_p2 - 16384) / temp_var;
    temp_var = 4294967296.0f;
    this->quantizedCalibCoef.par_p3 = static_cast<double>(this->calibCoef.par_p3) / temp_var;
    temp_var = 137438953472.0f;
    this->quantizedCalibCoef.par_p4 = static_cast<double>(this->calibCoef.par_p4) / temp_var;

    temp_var = 0.125f;
    this->quantizedCalibCoef.par_p5 = static_cast<double>(this->calibCoef.par_p5) / temp_var;
    temp_var = 64.0f;
    this->quantizedCalibCoef.par_p6 = static_cast<double>(this->calibCoef.par_p6) / temp_var;
    temp_var = 256.0f;
    this->quantizedCalibCoef.par_p7 = static_cast<double>(this->calibCoef.par_p7) / temp_var;
    temp_var = 32768.0f;
    this->quantizedCalibCoef.par_p8 = static_cast<double>(this->calibCoef.par_p8) / temp_var;
    temp_var = 281474976710656.0f;
    this->quantizedCalibCoef.par_p9 = static_cast<double>(this->calibCoef.par_p9) / temp_var;
    temp_var = 281474976710656.0f;
    this->quantizedCalibCoef.par_p10 = static_cast<double>(this->calibCoef.par_p10) / temp_var;
    temp_var = 36893488147419103232.0f;
    this->quantizedCalibCoef.par_p11 = static_cast<double>(this->calibCoef.par_p11) / temp_var;
}

