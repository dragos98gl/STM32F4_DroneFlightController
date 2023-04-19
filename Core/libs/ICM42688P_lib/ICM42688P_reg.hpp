/*
 * ICM42688P_reg.h
 *
 *  Created on: Sep 17, 2022
 *      Author: Asus
 */

#ifndef ICM42688P_LIB_ICM42688P_REG_H_
#define ICM42688P_LIB_ICM42688P_REG_H_

#include <stdint.h>

#define ICM_CS_PIN GPIO_PIN_5
#define ICM_CS_PORT GPIOC

/*
 * UNIVERSAL
 */

static const uint8_t REG_BANK_SEL = 0x76;

/*
 * BANK0
 */

static const uint8_t DEVICE_CONFIG = 0x11;
static const uint8_t INT_CONFIG = 0x14;
static const uint8_t TEMP_DATA1 = 0x1D;
static const uint8_t TEMP_DATA0 = 0x1E;
static const uint8_t ACCEL_DATA_X1 = 0x1F;
static const uint8_t ACCEL_DATA_X0 = 0x20;
static const uint8_t ACCEL_DATA_Y1 = 0x21;
static const uint8_t ACCEL_DATA_Y0 = 0x22;
static const uint8_t ACCEL_DATA_Z1 = 0x23;
static const uint8_t ACCEL_DATA_Z0 = 0x24;
static const uint8_t GYRO_DATA_X1 = 0x25;
static const uint8_t GYRO_DATA_X0 = 0x26;
static const uint8_t GYRO_DATA_Y1 = 0x27;
static const uint8_t GYRO_DATA_Y0 = 0x28;
static const uint8_t GYRO_DATA_Z1 = 0x29;
static const uint8_t GYRO_DATA_Z0 = 0x2A;
static const uint8_t INT_STATUS = 0x2D;
static const uint8_t PWR_MGMT0 = 0x4E;
static const uint8_t INTF_CONFIG1 = 0x4D;
static const uint8_t GYRO_CONFIG0 = 0x4F;
static const uint8_t ACCEL_CONFIG0 = 0x50;
static const uint8_t GYRO_CONFIG = 0x51;
static const uint8_t GYRO_ACCEL_CONFIG0 = 0x52;
static const uint8_t ACCEL_CONFIG1 = 0x53;
static const uint8_t INT_CONFIG0 = 0x63;
static const uint8_t INT_CONFIG1 = 0x64;
static const uint8_t INT_SOURCE0 = 0x65;
static const uint8_t SELF_TEST_CONFIG = 0x70;
static const uint8_t WHO_AM_I = 0x75;

/*
 * BANK1
 */

static const uint8_t GYRO_CONFIG_STATIC2 = 0x0B;
static const uint8_t GYRO_CONFIG_STATIC3 = 0x0C;
static const uint8_t GYRO_CONFIG_STATIC4 = 0x0D;
static const uint8_t GYRO_CONFIG_STATIC5 = 0x0E;
static const uint8_t GYRO_CONFIG_STATIC6 = 0x0F;
static const uint8_t GYRO_CONFIG_STATIC7 = 0x10;
static const uint8_t GYRO_CONFIG_STATIC8 = 0x11;
static const uint8_t GYRO_CONFIG_STATIC9 = 0x12;
static const uint8_t GYRO_CONFIG_STATIC10 = 0x13;
static const uint8_t XG_ST_DATA = 0x5F;
static const uint8_t YG_ST_DATA = 0x60;
static const uint8_t ZG_ST_DATA = 0x61;

/*
 * BANK2
 */

static const uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
static const uint8_t ACCEL_CONFIG_STATIC3 = 0x04;
static const uint8_t ACCEL_CONFIG_STATIC4 = 0x05;
static const uint8_t XA_ST_DATA = 0x3B;
static const uint8_t YA_ST_DATA = 0x3C;
static const uint8_t ZA_ST_DATA = 0x3D;

/*
 * BANK4
 */

static const uint8_t OFFSET_USER0 = 0x77;
static const uint8_t OFFSET_USER1 = 0x78;
static const uint8_t OFFSET_USER2 = 0x79;
static const uint8_t OFFSET_USER3 = 0x7A;
static const uint8_t OFFSET_USER4 = 0x7B;
static const uint8_t OFFSET_USER5 = 0x7C;
static const uint8_t OFFSET_USER6 = 0x7D;
static const uint8_t OFFSET_USER7 = 0x7E;
static const uint8_t OFFSET_USER8 = 0x7F;


static const uint8_t DEVICE_CONFIG_SOFT_RESET_CONFIG = 1;
static const uint8_t DEVICE_CONFIG_SPI_MODE = 1;

static const uint8_t INT_CONFIG_INT1_MODE_LATCHED = 1<<2;
static const uint8_t INT_CONFIG_INT1_DRIVE_CIRCUIT_PUSH_PULL = 1<<1;
static const uint8_t INT_CONFIG_INT1_POLARITY_ACTIVE_HIGH = 1;

static const uint8_t POWER_MGMT0_ACCEL_LOW_NOISE = 0b11;
static const uint8_t POWER_MGMT0_GYRO_LOW_NOISE = 0b11<<2;
static const uint8_t POWER_MGMT0_GYRO_STANDBY = 0b01<<2;

static const uint8_t GYRO_CONFIG0_GYRO_ODR_32KHZ = 1;
static const uint8_t GYRO_CONFIG0_GYRO_ODR_16KHZ = 0b10;
static const uint8_t GYRO_CONFIG0_GYRO_ODR_8KHZ = 0b11;
static const uint8_t GYRO_CONFIG0_GYRO_ODR_4KHZ = 0b100;
static const uint8_t GYRO_CONFIG0_GYRO_ODR_2KHZ = 0b101;
static const uint8_t GYRO_CONFIG0_GYRO_ODR_1KHZ = 0b110;
static const uint8_t GYRO_CONFIG0_GYRO_ODR_500HZ = 0b1111;
static const uint8_t GYRO_CONFIG0_GYRO_ODR_200HZ = 0b111;
static const uint8_t GYRO_CONFIG0_GYRO_ODR_100HZ = 0b1000;
static const uint8_t GYRO_CONFIG0_GYRO_ODR_50HZ = 0b1001;
static const uint8_t GYRO_CONFIG0_GYRO_FS_SEL_2000DPS=0;
static const uint8_t GYRO_CONFIG0_GYRO_FS_SEL_1000DPS=1<<5;
static const uint8_t GYRO_CONFIG0_GYRO_FS_SEL_500DPS=0b10<<5;
static const uint8_t GYRO_CONFIG0_GYRO_FS_SEL_250DPS=0b11<<5;
static const uint8_t GYRO_CONFIG0_GYRO_FS_SEL_125DPS=0b100<<5;
static const uint8_t GYRO_CONFIG0_GYRO_FS_SEL_62_5DPS=0b101<<5;
static const uint8_t GYRO_CONFIG0_GYRO_FS_SEL_31_25DPS=0b110<<5;
static const uint8_t GYRO_CONFIG0_GYRO_FS_SEL_15_625DPS=0b111<<5;

static const uint8_t ACCEL_CONFIG0_ACCEL_ODR_32KHZ = 1;
static const uint8_t ACCEL_CONFIG0_ACCEL_ODR_16KHZ = 0b10;
static const uint8_t ACCEL_CONFIG0_ACCEL_ODR_8KHZ = 0b11;
static const uint8_t ACCEL_CONFIG0_ACCEL_ODR_4KHZ = 0b100;
static const uint8_t ACCEL_CONFIG0_ACCEL_ODR_2KHZ = 0b101;
static const uint8_t ACCEL_CONFIG0_ACCEL_ODR_1KHZ = 0b110;
static const uint8_t ACCEL_CONFIG0_ACCEL_ODR_500HZ = 0b1111;
static const uint8_t ACCEL_CONFIG0_ACCEL_ODR_200HZ = 0b111;
static const uint8_t ACCEL_CONFIG0_ACCEL_ODR_100HZ = 0b1000;
static const uint8_t ACCEL_CONFIG0_ACCEL_ODR_50HZ = 0b1001;
static const uint8_t ACCEL_CONFIG0_ACCEL_FS_SEL_16G=0;
static const uint8_t ACCEL_CONFIG0_ACCEL_FS_SEL_8G=1<<5;
static const uint8_t ACCEL_CONFIG0_ACCEL_FS_SEL_4G=0b10<<5;
static const uint8_t ACCEL_CONFIG0_ACCEL_FS_SEL_2G=0b11<<5;

static const uint8_t INT_CONFIG0_UI_DRDY_INT_CLEAR_ONSENSORREGREAD = 0b10<<4;
static const uint8_t INT_CONFIG0_UI_DRDY_INT_CLEAR_ONSTATUSBITREAD = 0b00<<4;

static const uint8_t INT_SOURCE0_UI_AGC_RDY_INT1_EN=1;
static const uint8_t INT_SOURCE0_FIFO_FULL_INT1_EN=1<<1;
static const uint8_t INT_SOURCE0_FIFO_THS_INT1_EN=1<<2;
static const uint8_t INT_SOURCE0_UI_DRDY_INT1_EN=1<<3;
static const uint8_t INT_SOURCE0_RESET_DONE_INT1_EN=1<<4;
static const uint8_t INT_SOURCE0_PLL_RDY_INT1_EN=1<<5;
static const uint8_t INT_SOURCE0_UI_FSYNC_INT1_EN=1<<6;


#endif /* ICM42688P_LIB_ICM42688P_REG_H_ */
