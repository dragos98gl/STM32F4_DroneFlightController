/*
 * LIS3MDLTR_reg.h
 *
 *  Created on: Sep 21, 2022
 *      Author: Asus
 */

#ifndef LIS3MDLTR_LIB_LIS3MDLTR_REG_H_
#define LIS3MDLTR_LIB_LIS3MDLTR_REG_H_

#include <stdint.h>

#define LIS_CS_PIN GPIO_PIN_5
#define LIS_CS_PORT GPIOA
#define LIS_WHO_AM_I_VALUE 0b00111101

static const uint8_t LIS_WHO_AM_I = 0x0F;
static const uint8_t CTRL_REG1 = 0x20;
static const uint8_t CTRL_REG2 = 0x21;
static const uint8_t CTRL_REG3 = 0x22;
static const uint8_t CTRL_REG4 = 0x23;
static const uint8_t CTRL_REG5 = 0x24;
static const uint8_t STATUS_REG = 0x27;
static const uint8_t OUT_X_L = 0x28;
static const uint8_t OUT_X_H = 0x29;
static const uint8_t OUT_Y_L = 0x2A;
static const uint8_t OUT_Y_H = 0x2B;
static const uint8_t OUT_Z_L = 0x2C;
static const uint8_t OUT_Z_H = 0x2D;
static const uint8_t TEMP_OUT_L = 0x2E;
static const uint8_t TEMP_OUT_H = 0x2F;
static const uint8_t INT_CFG = 0x30;
static const uint8_t INT_SRT = 0x31;
static const uint8_t INT_THS_L = 0x32;
static const uint8_t INT_THS_H = 0x33;

static const uint8_t CTRL_REG1_ST_EN = 1;
static const uint8_t CTRL_REG1_FAST_ODR_EN = 1<<1;
static const uint8_t CTRL_REG1_DO_1_25HZ = 0b001<<2;
static const uint8_t CTRL_REG1_DO_2_5HZ = 0b010<<2;
static const uint8_t CTRL_REG1_DO_5HZ = 0b011<<2;
static const uint8_t CTRL_REG1_DO_10HZ = 0b100<<2;
static const uint8_t CTRL_REG1_DO_20HZ = 0b101<<2;
static const uint8_t CTRL_REG1_DO_40HZ = 0b110<<2;
static const uint8_t CTRL_REG1_DO_80HZ = 0b111<<2;
static const uint8_t CTRL_REG1_OM_LOWPOWER = 0;
static const uint8_t CTRL_REG1_OM_MEDIUMPOWER = 0b01<<5;
static const uint8_t CTRL_REG1_OM_HIGHPOWER = 0b10<<5;
static const uint8_t CTRL_REG1_OM_ULTRAPOWER = 0b11<<5;
static const uint8_t CTRL_REG1_TEMP_EN = 1<<7;

static const uint8_t CTRL_REG2_SOFT_RST = 1<<2;
static const uint8_t CTRL_REG2_REBOOT = 1<<3;
static const uint8_t CTRL_REG2_FS_4GAUSS = 0;
static const uint8_t CTRL_REG2_FS_8GAUSS = 0b01<<5;
static const uint8_t CTRL_REG2_FS_12GAUSS = 0b10<<5;
static const uint8_t CTRL_REG2_FS_16GAUSS = 0b11<<5;

static const uint8_t CTRL_REG3_MD_CONTINOUS = 0;
static const uint8_t CTRL_REG3_MD_SINGLE = 1;
static const uint8_t CTRL_REG3_M_POWER_DOWN = 0b11;
static const uint8_t CTRL_REG3_SPI_3WIRE = 1<<2;
static const uint8_t CTRL_REG3_SPI_4WIRE = 0;
static const uint8_t CTRL_REG3_LP = 1<<5;

static const uint8_t CTRL_REG4_BLE_MSB_LADDR = 1<<1;
static const uint8_t CTRL_REG4_BLE_LSB_LADDR = 1<<1;
static const uint8_t CTRL_REG4_OMZ_LOWPOWER = 0;
static const uint8_t CTRL_REG4_OMZ_MEDIUMPOWER = 0<<2;
static const uint8_t CTRL_REG4_OMZ_HIGHPOWER = 0b10<<2;
static const uint8_t CTRL_REG4_OMZ_ULTRAPOWER = 0b11<<2;

static const uint8_t CTRL_REG5_BDU = 1<<6;
static const uint8_t CTRL_REG5_FAST_READ = 1<<7;

static const uint8_t INT_CFG_IEN = 1;

#endif /* LIS3MDLTR_LIB_LIS3MDLTR_REG_H_ */
