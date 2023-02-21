/*
 * BMP390_reg.h
 *
 *  Created on: Sep 21, 2022
 *      Author: Asus
 */

#ifndef BMP390_LIB_BMP390_REG_H_
#define BMP390_LIB_BMP390_REG_H_

#include <stdint.h>

#define BMP_CS_PIN GPIO_PIN_12
#define BMP_CS_PORT GPIOB
#define BMP_WHO_AM_I 96

static const uint8_t CMD = 0x7E;
static const uint8_t CONFIG = 0x1F;
static const uint8_t ODR = 0x1D;
static const uint8_t OSR = 0x1C;
static const uint8_t PWR_CTRL = 0x1B;
static const uint8_t IF_CONF = 0x1A;
static const uint8_t INT_CTRL = 0x19;
static const uint8_t FIFO_CONFIG_2 = 0x18;
static const uint8_t FIFO_CONFIG_1 = 0x17;
static const uint8_t FIFO_WTM_1 = 0x16;
static const uint8_t FIFO_WTM_0 = 0x15;
static const uint8_t FIFO_DATA = 0x14;
static const uint8_t FIFO_LENGTH_1 = 0x13;
static const uint8_t FIFO_LENGTH_0 = 0x12;
static const uint8_t BMP390_INT_STATUS = 0x11;
static const uint8_t EVENT = 0x10;
static const uint8_t SENSORTIME_2 = 0x0E;
static const uint8_t SENSORTIME_1 = 0x0D;
static const uint8_t SENSORTIME_0 = 0x0C;
static const uint8_t DATA_5 = 0x09;
static const uint8_t DATA_4 = 0x08;
static const uint8_t DATA_3 = 0x07;
static const uint8_t DATA_2 = 0x06;
static const uint8_t DATA_1 = 0x05;
static const uint8_t DATA_0 = 0x04;
static const uint8_t STATUS = 0x03;
static const uint8_t ERR_REG = 0x02;
static const uint8_t REV_ID = 0x01;
static const uint8_t CHIP_ID = 0x00;

static const uint8_t PWR_CTRL_PRESS_EN = 0b00000001;
static const uint8_t PWR_CTRL_TEMP_EN = 0b00000010;
static const uint8_t PWR_CTRL_MODE_SLEEP = 0b00000000;
static const uint8_t PWR_CTRL_MODE_FORCED = 0b00100000;
static const uint8_t PWR_CTRL_MODE_NORMAL = 0b00110000;

static const uint8_t OSR_OSR_P_X1 = 0b00000000;
static const uint8_t OSR_OSR_P_X2 = 0b00000001;
static const uint8_t OSR_OSR_P_X4 = 0b00000010;
static const uint8_t OSR_OSR_P_X8 = 0b00000011;
static const uint8_t OSR_OSR_P_X16 = 0b00000100;
static const uint8_t OSR_OSR_P_X32 = 0b00000101;
static const uint8_t OSR_OSR_T_X1 = 0b00000000;
static const uint8_t OSR_OSR_T_X2 = 0b00001000;
static const uint8_t OSR_OSR_T_X4 = 0b00010000;
static const uint8_t OSR_OSR_T_X8 = 0b00011000;
static const uint8_t OSR_OSR_T_X16 = 0b00100000;
static const uint8_t OSR_OSR_T_X32 = 0b00101000;

static const uint8_t ODR_ODR_200 = 0x00;
static const uint8_t ODR_ODR_100 = 0x01;
static const uint8_t ODR_ODR_50 = 0x02;
static const uint8_t ODR_ODR_25 = 0x03;
static const uint8_t ODR_ODR_12p5 = 0x04;
static const uint8_t ODR_ODR_6p25 = 0x05;
static const uint8_t ODR_ODR_3p1 = 0x06;
static const uint8_t ODR_ODR_1p5 = 0x07;
static const uint8_t ODR_ODR_0p78 = 0x08;
static const uint8_t ODR_ODR_0p39 = 0x09;
static const uint8_t ODR_ODR_0p2 = 0x0A;
static const uint8_t ODR_ODR_0p1 = 0x0B;
static const uint8_t ODR_ODR_0p05 = 0x0C;
static const uint8_t ODR_ODR_0p02 = 0x0D;
static const uint8_t ODR_ODR_0p01 = 0x0E;
static const uint8_t ODR_ODR_0p003 = 0x0F;
static const uint8_t ODR_ODR_0p0015 = 0x11;

static const uint8_t CONFIG_COEF_0 = 0b000;
static const uint8_t CONFIG_COEF_1 = 0b001;
static const uint8_t CONFIG_COEF_3 = 0b010;
static const uint8_t CONFIG_COEF_7 = 0b011;
static const uint8_t CONFIG_COEF_15 = 0b100;
static const uint8_t CONFIG_COEF_31 = 0b101;
static const uint8_t CONFIG_COEF_63 = 0b110;
static const uint8_t CONFIG_COEF_127 = 0b111;

static const uint8_t CMD_FIFO_FLUSH = 0xB0;
static const uint8_t CMD_SOFTRESET = 0xB6;

static const uint8_t IF_CONF_SPI3_4WIRE = 0;
static const uint8_t IF_CONF_SPI3_3WIRE = 1;
static const uint8_t IF_CONF_I2C_WDT_EN_DISABLE = 0;
static const uint8_t IF_CONF_I2C_WDT_EN_ENABLE = 1<<1;
static const uint8_t IF_CONF_I2C_WDT_SEL_WDT_SHORT = 0;
static const uint8_t IF_CONF_I2C_WDT_SEL_WDT_LONG = 1<<2;

static const uint8_t INT_CTRL_INT_OD = 1;
static const uint8_t INT_CTRL_INT_LEVEL = 1<<1;
static const uint8_t INT_CTRL_INT_LATCH = 1<<2;
static const uint8_t INT_CTRL_FWTM_EN = 1<<3;
static const uint8_t INT_CTRL_FFULL_EN = 1<<4;
static const uint8_t INT_CTRL_INT_DS = 1<<5;
static const uint8_t INT_CTRL_DRDY_EN = 1<<6;

#endif /* BMP390_LIB_BMP390_REG_H_ */
