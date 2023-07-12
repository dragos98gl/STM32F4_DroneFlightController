/*
 * Implementation.hpp
 *
 *  Created on: Apr 18, 2023
 *      Author: DDarie
 */

#ifndef SRC_IMPLEMENTATION_FLIGHTCONTROLLERIMPLEMENTATION_HPP_
#define SRC_IMPLEMENTATION_FLIGHTCONTROLLERIMPLEMENTATION_HPP_

#include "stm32f4xx_hal.h"
#include <string>
#include "math.h"
#include <mutex>

#include "Enums.hpp"
#include "BMP390.hpp"
#include "ICM42688P.hpp"
#include "LIS3MDLTR.hpp"
#include "PMW3901UY.hpp"
#include "HC05.hpp"
#include "MB1043.hpp"
#include "FrSkyRX.hpp"
#include "PID_Control.hpp"
#include "Buzzer.hpp"
#include "BatteryManagement.hpp"
#include "FlashMemoryBlock.hpp"
#include "VL53L0X.hpp"

class FlightControllorImplementation
{
private:
	ADC_HandleTypeDef *hadc1;
	SPI_HandleTypeDef *hspi2;
	UART_HandleTypeDef *huart1;
	UART_HandleTypeDef *huart2;
	UART_HandleTypeDef *huart3;
	UART_HandleTypeDef *huart4;
	UART_HandleTypeDef *huart6;
	DMA_HandleTypeDef *hdma_usart2_rx;
	DMA_HandleTypeDef *hdma_usart3_rx;
	DMA_HandleTypeDef *hdma_uart4_rx;
	DMA_HandleTypeDef *hdma_usart6_rx;

	MemoryManagement nvmInstance;
	Buzzer buzz;
	LIS3MDLTR lis;
	BMP390 bmp;
	HC05 bt;
	FrSkyRX remote_rx;
	MB1043 sonar;
	VL53L0X vl53;
	BatteryManagement battMgmt;
	PID_Control roll_pid;
	PID_Control pitch_pid;
	PID_Control yaw_pid;
	PID_Control xPosition_pid;
	PID_Control yPosition_pid;
	ICM42688P icm;
	PMW3901UY pmw;
	FaultsStatus _currentFaultsStatus;

	TaskHandle_t _faultsCheckHandler = NULL;
	TaskHandle_t _sensorsDataReadHandler = NULL;
	TaskHandle_t _dynamicsProcessHandler = NULL;
public:
	float zeroRef = 0.0F;

	FlightControllorImplementation (
			ADC_HandleTypeDef *hadc1,
			SPI_HandleTypeDef *hspi2,
			UART_HandleTypeDef *huart1,
			UART_HandleTypeDef *huart2,
			UART_HandleTypeDef *huart3,
			UART_HandleTypeDef *huart4,
			UART_HandleTypeDef *huart6,
			DMA_HandleTypeDef *hdma_usart2_rx,
			DMA_HandleTypeDef *hdma_usart3_rx,
			DMA_HandleTypeDef *hdma_uart4_rx,
			DMA_HandleTypeDef *hdma_usart6_rx
			)
	: hadc1 {hadc1},
	  hspi2 {hspi2},
	  huart1 {huart1},
	  huart2 {huart2},
	  huart3 {huart3},
	  huart4 {huart4},
	  huart6 {huart6},
	  hdma_usart2_rx {hdma_usart2_rx},
	  hdma_usart3_rx {hdma_usart3_rx},
	  hdma_uart4_rx {hdma_uart4_rx},
	  hdma_usart6_rx {hdma_usart6_rx},
	  nvmInstance{MAINMEMORY_ADDRESS},
	  lis (hspi2),
	  bmp (hspi2),
	  bt (huart1),
	  remote_rx (huart3, hdma_usart3_rx, &buzz, 1),
	  sonar (huart4,hdma_uart4_rx,255U),
	  vl53(huart6,hdma_usart6_rx,255U),
	  battMgmt (hadc1,&buzz,1000U),
	  roll_pid(icm.getEulerYref(),zeroRef,6,0,5000),
	  pitch_pid(icm.getEulerXref(),zeroRef,6,0,5000),
	  yaw_pid(icm.getEulerZref(),zeroRef,10,0,0),
	  xPosition_pid(pmw.getXpos(),remote_rx.target_roll,0.5F,0,100),
	  yPosition_pid(pmw.getYpos(),remote_rx.target_pitch,0.5F,0,100),
	  icm (hspi2,&buzz,roll_pid,pitch_pid,yaw_pid),
	  pmw (huart2, hdma_usart2_rx, 255U, icm, vl53,xPosition_pid,yPosition_pid),
	  _currentFaultsStatus {FaultsStatus::NOT_READY}
	{

	}

	MemoryManagement& getNvmInstance();
	Buzzer& getBuzzerinstance();
	LIS3MDLTR& getLIS3MDLTRinstance();
	BMP390& getBMP390instance();
	ICM42688P& getICM42688Pinstance();
	HC05& getHC05instance();
	PMW3901UY& getPMW3901UYinstance();
	FrSkyRX& getFrSkyRXinstance();
	MB1043& getMB1043instance();
	VL53L0X& getVL53L0Xinstance();
	BatteryManagement& getBatteryManagementinstance();
	PID_Control& getRollPidInstance();
	PID_Control& getPitchPidInstance();
	PID_Control& getYawPidInstance();
	PID_Control& getXPositionPidInstance();
	PID_Control& getYPositionPidInstance();

	TaskHandle_t* getFaultsCheckHandlerPtr();
	TaskHandle_t* getSensorsDataReadHandlerPtr();
	TaskHandle_t* getDynamicsProcessHandlerPtr();

	void initialization();
	FaultsStatus getCurrentFaultsStatus() const;
	void setCurrentFaultsStatus(FaultsStatus faultsStatus);

	static FlightControllorImplementation *getInstance();

	FlightControllorImplementation(FlightControllorImplementation &other) = delete; // not cloneable
	void operator=(const FlightControllorImplementation &) = delete; // not assignable
};

#endif /* SRC_IMPLEMENTATION_FLIGHTCONTROLLERIMPLEMENTATION_HPP_ */
