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
	ADC_HandleTypeDef* _hadc1;
	SPI_HandleTypeDef* _hspi2;
	UART_HandleTypeDef* _huart1;
	UART_HandleTypeDef* _huart2;
	UART_HandleTypeDef* _huart3;
	UART_HandleTypeDef* _huart4;
	UART_HandleTypeDef* _huart6;
	DMA_HandleTypeDef* _hdma_usart2_rx;
	DMA_HandleTypeDef* _hdma_usart3_rx;
	DMA_HandleTypeDef* _hdma_uart4_rx;
	DMA_HandleTypeDef* _hdma_usart6_rx;
	MemoryManagement _nvmInstance;
	Buzzer _buzz;
	LIS3MDLTR _lis;
	BMP390 _bmp;
	HC05 _bt;
	FrSkyRX _remote_rx;
	MB1043 _sonar;
	VL53L0X _vl53;
	BatteryManagement _battMgmt;
	PID_Control _rollPID;
	PID_Control _pitchPID;
	PID_Control _yawPID;
	PID_Control _xPositionPID;
	PID_Control _yPositionPID;
	ICM42688P _icm;
	PMW3901UY _pmw;
	FaultsStatus _currentFaultsStatus;
	TaskHandle_t _faultsCheckHandler;
	TaskHandle_t _sensorsDataReadHandler;
	TaskHandle_t _dynamicsProcessHandler;
public:
	float zeroRef = 0.0F;

	FlightControllorImplementation (
			ADC_HandleTypeDef* hadc1,
			SPI_HandleTypeDef* hspi2,
			UART_HandleTypeDef* huart1,
			UART_HandleTypeDef* huart2,
			UART_HandleTypeDef* huart3,
			UART_HandleTypeDef* huart4,
			UART_HandleTypeDef* huart6,
			DMA_HandleTypeDef* hdma_usart2_rx,
			DMA_HandleTypeDef* hdma_usart3_rx,
			DMA_HandleTypeDef* hdma_uart4_rx,
			DMA_HandleTypeDef* hdma_usart6_rx
			)
	: _hadc1 {hadc1},
	  _hspi2 {hspi2},
	  _huart1 {huart1},
	  _huart2 {huart2},
	  _huart3 {huart3},
	  _huart4 {huart4},
	  _huart6 {huart6},
	  _hdma_usart2_rx {hdma_usart2_rx},
	  _hdma_usart3_rx {hdma_usart3_rx},
	  _hdma_uart4_rx {hdma_uart4_rx},
	  _hdma_usart6_rx {hdma_usart6_rx},
	  _nvmInstance{MAINMEMORY_ADDRESS},
	  _lis (hspi2),
	  _bmp (hspi2),
	  _bt (huart1),
	  _remote_rx (huart3, hdma_usart3_rx, &_buzz, 1),
	  _sonar (huart4,hdma_uart4_rx,255U),
	  _vl53(huart6,hdma_usart6_rx,255U),
	  _battMgmt (hadc1,&_buzz,1000U),
	  _rollPID(_icm.getEulerYref(),zeroRef,6,0,4000),//6,0,5000),
	  _pitchPID(_icm.getEulerXref(),zeroRef,6,0,4000),//6,0,5000),
	  _yawPID(_icm.getEulerZref(),zeroRef,10,0,0),//10,0,0),
	  _xPositionPID(_pmw.getXpos(),_remote_rx.target_roll,0.5F,0,200),
	  _yPositionPID(_pmw.getYpos(),_remote_rx.target_pitch,0.5F,0,200),
	  _icm (hspi2,&_buzz,_rollPID,_pitchPID,_yawPID),
	  _pmw (huart2, hdma_usart2_rx, 255U,_icm,_vl53,_xPositionPID,_yPositionPID),
	  _currentFaultsStatus {FaultsStatus::NOT_READY},
	  _faultsCheckHandler (NULL),
	  _sensorsDataReadHandler (NULL),
	  _dynamicsProcessHandler (NULL)
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
