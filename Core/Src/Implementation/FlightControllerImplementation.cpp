/*
 * FlightControllerImplementation.cpp
 *
 *  Created on: Apr 18, 2023
 *      Author: DDarie
 */

#include "FlightControllerImplementation.hpp"

enum class FaultsStatus
{
	OKAY,
	NOT_READY,
	FAILURE,
	CRITICAL
};
/*
FaultsStatus currentFaultsStatus {FaultsStatus::NOT_READY};

TaskHandle_t FaultsCheckHandler = NULL;
TaskHandle_t SensorsDataReadHandler = NULL;
TaskHandle_t DynamicsProcessHandler = NULL;
void FaultsCheckTask(void *pvParameters);
void SensorsDataReadTask(void *pvParameters);
void DynamicsProcessTask(void *pvParameters);

int icmCounter = 0;
int bmpCounter = 0;
int lisCounter = 0;
int remoteCounter = 0;
int pmwCounter = 0;
int sonarCounter = 0;
int icmCounter1 = 0;
int bmpCounter1 = 0;
int lisCounter1 = 0;
int remoteCounter1 = 0;
int pmwCounter1 = 0;
int sonarCounter1 = 0;
int taskCounter = 0;
int timCounter = 0;

float euler_x{0.0F};
float euler_y{0.0F};
float euler_z{0.0F};
float throttle{0.0F};
//PID_Control roll_pid(euler_y, 10, 0.001, 5000);
PID_Control roll_pid(
		euler_y,
		remote_rx.target_roll,
		10,
		0,
		0);
//PID_Control pitch_pid(euler_x, 10, 0.001, 5000);
PID_Control pitch_pid(
		euler_x,
		remote_rx.target_pitch,
		10,
		0,
		0);
PID_Control yaw_pid(
		euler_z,
		remote_rx.target_yaw,
		0,
		0,
		0);

float roll, pitch, heading;
float test1=roll_pid.out();
int tick1=0;
int icmCouter2 = 0;
int duplicates = 0;
int duplicatesCounter = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
    	tick1++;

    	bt.printfSensorsValues();
    }
}

int timCounter2 = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }

  if (htim->Instance == TIM4)
  {
	  timCounter2++;
  }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	switch (GPIO_Pin)
	{
	case (GPIO_PIN_4):
		xTaskNotifyFromISR(SensorsDataReadHandler, EnumSensorsInterrupt::ICM42688P_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		icmCounter1++;
		break;

	case (GPIO_PIN_8):
		xTaskNotifyFromISR(SensorsDataReadHandler, EnumSensorsInterrupt::BMP390_t, eSetBits, &pxHigherPriorityTaskWoken);
		if (pxHigherPriorityTaskWoken)
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		bmpCounter1++;
		break;

	case (GPIO_PIN_2):
		xTaskNotifyFromISR(SensorsDataReadHandler, EnumSensorsInterrupt::LIS3MDLTR_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		lisCounter1++;
		break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	if (huart->Instance == USART2)
	{
		xTaskNotifyFromISR(SensorsDataReadHandler, EnumSensorsInterrupt::PMW_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		pmwCounter1++;
	} else if (huart->Instance == USART3)
	{
		xTaskNotifyFromISR(SensorsDataReadHandler, EnumSensorsInterrupt::REMOTERX_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		remoteCounter1++;
	} else if (huart->Instance == UART4)
	{
		xTaskNotifyFromISR(SensorsDataReadHandler, EnumSensorsInterrupt::SONAR_t, eSetBits, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		sonarCounter1++;

	}
}

void SensorsDataReadTask(void *pvParameters)
{
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_SetPriorityGrouping(0);

	bool startup;
	if(!lis.defaultInit())
	  startup = false;
	lis.update();

	if (!icm.defaultInit())
	  startup = false;
	icm.update();

	if (!bmp.defaultInit())
	  startup = false;
	bmp.update();

	remote_rx.begin();
	sonar.begin();
	pmw.begin();

	uint32_t currentSensor = 0;
	xTaskCreate(DynamicsProcessTask,"DynamicsProcessTask",256,NULL,tskIDLE_PRIORITY+2, &DynamicsProcessHandler);
	xTaskCreate(FaultsCheckTask,"FaultsCheckTask",256,NULL,tskIDLE_PRIORITY+2, &FaultsCheckHandler);

	while (1)
	{
		if (xTaskNotifyWait(0x00, 0xFFFFFFFFUL, &currentSensor, portMAX_DELAY) == pdTRUE)
		{
			if (currentSensor & EnumSensorsInterrupt::ICM42688P_t)
			{
				icm.update();
				icmCounter++;
				icmCouter2++;
				int testt = icm.getAccX();
				if (testt==duplicates)
					duplicatesCounter++;
				duplicates = testt;
			}

			if (currentSensor & EnumSensorsInterrupt::BMP390_t)
			{
				bmp.update();
				bmpCounter++;
			}

			if (currentSensor & EnumSensorsInterrupt::LIS3MDLTR_t)
			{
				lis.update();
				lisCounter++;
			}

			if (currentSensor & EnumSensorsInterrupt::PMW_t)
			{
				__HAL_UART_FLUSH_DRREGISTER(&huart2);
				pmw.update();
				pmwCounter++;
			}

			if (currentSensor & EnumSensorsInterrupt::REMOTERX_t)
			{
				__HAL_UART_FLUSH_DRREGISTER(&huart3);
				remote_rx.update();
				remoteCounter++;
			}

			if (currentSensor & EnumSensorsInterrupt::SONAR_t)
			{
			   __HAL_UART_FLUSH_DRREGISTER(&huart4);
			   sonar.update();
			   sonarCounter++;
			}

			taskCounter++;
		}
	}
}

void FaultsCheckTask(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1;

	xLastWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		pmw.incrementTimeoutCounter();
		remote_rx.incrementTimeoutCounter();
		sonar.incrementTimeoutCounter();

		buzz.run();
		BattMgmt.run();

		if (remote_rx.getCurrentState() == FrSkyRXState::READY)
		{
			currentFaultsStatus = FaultsStatus::OKAY;
		}

		vTaskDelayUntil( &xLastWakeTime, xFrequency);
	}
}

void DynamicsProcessTask(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1;

	xLastWakeTime = xTaskGetTickCount();

	for( ;; )
	{

		euler_x = icm.getEulerX();
		euler_y = icm.getEulerY();
		euler_z = icm.getEulerZ();
		throttle = remote_rx.throttle;

		timCounter++;

		if (currentFaultsStatus == FaultsStatus::OKAY)
		{
			/*
			CR1    CR3
			   \  /
				\/
				/\
			   /  \
			CR2    CR4
			 */
			/*float CCR1_value = 3000 + remote_rx.throttle + roll_pid.out() + pitch_pid.out() - yaw_pid.out();
			float CCR2_value = 3000 + remote_rx.throttle + roll_pid.out() - pitch_pid.out() + yaw_pid.out();
			float CCR3_value = 3000 + remote_rx.throttle - roll_pid.out() + pitch_pid.out() + yaw_pid.out();
			float CCR4_value = 3000 + remote_rx.throttle - roll_pid.out() - pitch_pid.out() - yaw_pid.out();

			if (CCR1_value<3300)
				TIM3 -> CCR1 = 3300;
			else
				TIM3 -> CCR1 = CCR1_value;

			if (CCR2_value<3300)
				TIM3 -> CCR2 = 3300;
			else
				TIM3 -> CCR2 = CCR2_value;

			if (CCR3_value<3300)
				TIM3 -> CCR3 = 3300;
			else
				TIM3 -> CCR3 = CCR3_value;

			if (CCR4_value<3300)
				TIM3 -> CCR4 = 3300;
			else
				TIM3 -> CCR4 = CCR4_value;

		// ... = base_throttle + alt_compensation + roll/pitch/yaw_pid;
		}

		if (currentFaultsStatus == FaultsStatus::FAILURE)
		{

		}

		if (currentFaultsStatus == FaultsStatus::CRITICAL)


		vTaskDelayUntil( &xLastWakeTime, xFrequency);
	}
}
*/
