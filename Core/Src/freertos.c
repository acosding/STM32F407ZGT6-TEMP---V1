/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "adc.h"
#include "motor.h"
#include "oled.h"
#include "encoder.h"
#include "pid.h"
#include "kalman_filter.h"
#include "uart_ringbuffer.h"
#include "servo.h"
#include "st7735.h"
#include "keyscan.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void Uart6_printf(char *str);
void Uart2_printf(char *str);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern DUT_USART_FIFO g_tUART_6;
extern DUT_USART_FIFO g_tUART_2;

int ok = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED */
osThreadId_t LEDHandle;
const osThreadAttr_t LED_attributes = {
  .name = "LED",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for WheelDrive */
osThreadId_t WheelDriveHandle;
const osThreadAttr_t WheelDrive_attributes = {
  .name = "WheelDrive",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADC */
osThreadId_t ADCHandle;
const osThreadAttr_t ADC_attributes = {
  .name = "ADC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OLED */
osThreadId_t OLEDHandle;
const osThreadAttr_t OLED_attributes = {
  .name = "OLED",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART */
osThreadId_t UARTHandle;
const osThreadAttr_t UART_attributes = {
  .name = "UART",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for keyScan */
osThreadId_t keyScanHandle;
const osThreadAttr_t keyScan_attributes = {
  .name = "keyScan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for servo */
osThreadId_t servoHandle;
const osThreadAttr_t servo_attributes = {
  .name = "servo",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for oled_msg */
osMessageQueueId_t oled_msgHandle;
const osMessageQueueAttr_t oled_msg_attributes = {
  .name = "oled_msg"
};
/* Definitions for degree */
osMessageQueueId_t degreeHandle;
const osMessageQueueAttr_t degree_attributes = {
  .name = "degree"
};
/* Definitions for oled_d */
osMessageQueueId_t oled_dHandle;
const osMessageQueueAttr_t oled_d_attributes = {
  .name = "oled_d"
};
/* Definitions for I2C_1 */
osMutexId_t I2C_1Handle;
const osMutexAttr_t I2C_1_attributes = {
  .name = "I2C_1"
};
/* Definitions for xKey01 */
osSemaphoreId_t xKey01Handle;
const osSemaphoreAttr_t xKey01_attributes = {
  .name = "xKey01"
};
/* Definitions for xKey02 */
osSemaphoreId_t xKey02Handle;
const osSemaphoreAttr_t xKey02_attributes = {
  .name = "xKey02"
};
/* Definitions for xKey03 */
osSemaphoreId_t xKey03Handle;
const osSemaphoreAttr_t xKey03_attributes = {
  .name = "xKey03"
};
/* Definitions for xKey04 */
osSemaphoreId_t xKey04Handle;
const osSemaphoreAttr_t xKey04_attributes = {
  .name = "xKey04"
};
/* Definitions for xKey05 */
osSemaphoreId_t xKey05Handle;
const osSemaphoreAttr_t xKey05_attributes = {
  .name = "xKey05"
};
/* Definitions for xKey06 */
osSemaphoreId_t xKey06Handle;
const osSemaphoreAttr_t xKey06_attributes = {
  .name = "xKey06"
};
/* Definitions for xKey07 */
osSemaphoreId_t xKey07Handle;
const osSemaphoreAttr_t xKey07_attributes = {
  .name = "xKey07"
};
/* Definitions for xKey08 */
osSemaphoreId_t xKey08Handle;
const osSemaphoreAttr_t xKey08_attributes = {
  .name = "xKey08"
};
/* Definitions for xKey09 */
osSemaphoreId_t xKey09Handle;
const osSemaphoreAttr_t xKey09_attributes = {
  .name = "xKey09"
};
/* Definitions for xKey10 */
osSemaphoreId_t xKey10Handle;
const osSemaphoreAttr_t xKey10_attributes = {
  .name = "xKey10"
};
/* Definitions for xKey11 */
osSemaphoreId_t xKey11Handle;
const osSemaphoreAttr_t xKey11_attributes = {
  .name = "xKey11"
};
/* Definitions for xKey12 */
osSemaphoreId_t xKey12Handle;
const osSemaphoreAttr_t xKey12_attributes = {
  .name = "xKey12"
};
/* Definitions for xKey13 */
osSemaphoreId_t xKey13Handle;
const osSemaphoreAttr_t xKey13_attributes = {
  .name = "xKey13"
};
/* Definitions for xKey14 */
osSemaphoreId_t xKey14Handle;
const osSemaphoreAttr_t xKey14_attributes = {
  .name = "xKey14"
};
/* Definitions for xKey15 */
osSemaphoreId_t xKey15Handle;
const osSemaphoreAttr_t xKey15_attributes = {
  .name = "xKey15"
};
/* Definitions for xKey16 */
osSemaphoreId_t xKey16Handle;
const osSemaphoreAttr_t xKey16_attributes = {
  .name = "xKey16"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void LEDTask(void *argument);
void WheelDriveTask(void *argument);
void ADCtask(void *argument);
void OLEDdriveTask(void *argument);
void UARTTask(void *argument);
void keyScanTask(void *argument);
void servoTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of I2C_1 */
  I2C_1Handle = osMutexNew(&I2C_1_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of xKey01 */
  xKey01Handle = osSemaphoreNew(1, 1, &xKey01_attributes);

  /* creation of xKey02 */
  xKey02Handle = osSemaphoreNew(1, 1, &xKey02_attributes);

  /* creation of xKey03 */
  xKey03Handle = osSemaphoreNew(1, 1, &xKey03_attributes);

  /* creation of xKey04 */
  xKey04Handle = osSemaphoreNew(1, 1, &xKey04_attributes);

  /* creation of xKey05 */
  xKey05Handle = osSemaphoreNew(1, 1, &xKey05_attributes);

  /* creation of xKey06 */
  xKey06Handle = osSemaphoreNew(1, 1, &xKey06_attributes);

  /* creation of xKey07 */
  xKey07Handle = osSemaphoreNew(1, 1, &xKey07_attributes);

  /* creation of xKey08 */
  xKey08Handle = osSemaphoreNew(1, 1, &xKey08_attributes);

  /* creation of xKey09 */
  xKey09Handle = osSemaphoreNew(1, 1, &xKey09_attributes);

  /* creation of xKey10 */
  xKey10Handle = osSemaphoreNew(1, 1, &xKey10_attributes);

  /* creation of xKey11 */
  xKey11Handle = osSemaphoreNew(1, 1, &xKey11_attributes);

  /* creation of xKey12 */
  xKey12Handle = osSemaphoreNew(1, 1, &xKey12_attributes);

  /* creation of xKey13 */
  xKey13Handle = osSemaphoreNew(1, 1, &xKey13_attributes);

  /* creation of xKey14 */
  xKey14Handle = osSemaphoreNew(1, 1, &xKey14_attributes);

  /* creation of xKey15 */
  xKey15Handle = osSemaphoreNew(1, 1, &xKey15_attributes);

  /* creation of xKey16 */
  xKey16Handle = osSemaphoreNew(1, 1, &xKey16_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of oled_msg */
  oled_msgHandle = osMessageQueueNew (4, sizeof(float), &oled_msg_attributes);

  /* creation of degree */
  degreeHandle = osMessageQueueNew (5, sizeof(int), &degree_attributes);

  /* creation of oled_d */
  oled_dHandle = osMessageQueueNew (5, sizeof(int), &oled_d_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LED */
  LEDHandle = osThreadNew(LEDTask, NULL, &LED_attributes);

  /* creation of WheelDrive */
  WheelDriveHandle = osThreadNew(WheelDriveTask, NULL, &WheelDrive_attributes);

  /* creation of ADC */
  ADCHandle = osThreadNew(ADCtask, NULL, &ADC_attributes);

  /* creation of OLED */
  OLEDHandle = osThreadNew(OLEDdriveTask, NULL, &OLED_attributes);

  /* creation of UART */
  UARTHandle = osThreadNew(UARTTask, NULL, &UART_attributes);

  /* creation of keyScan */
  keyScanHandle = osThreadNew(keyScanTask, NULL, &keyScan_attributes);

  /* creation of servo */
  servoHandle = osThreadNew(servoTask, NULL, &servo_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char led = 0;
	char timer = 0;
	
	osSemaphoreAcquire(xKey01Handle,1);
  /* Infinite loop */
  for(;;)
  {
		if(led){
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
		}
		led = !led;
		
		if(osSemaphoreAcquire(xKey16Handle,1)==osOK){
			timer = 1;
			//Start BEEP;
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
		}
		if(timer != 0){
			if(timer>=2){
				timer = 0;
				//STOP BEEP!
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
			}else{
				timer++;
			}
		}
		
		if(osSemaphoreAcquire(xKey01Handle,1) == osOK){
			Uart2_printf("reboot");
			printf("reboot!\r\n");
			ok = 0;
		}
		HAL_IWDG_Refresh(&hiwdg);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LEDTask */
/**
* @brief Function implementing the LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDTask */
void LEDTask(void *argument)
{
  /* USER CODE BEGIN LEDTask */
	osDelay(5000);
	osSemaphoreAcquire(xKey13Handle,1);
	osSemaphoreAcquire(xKey14Handle,1);
	osSemaphoreAcquire(xKey08Handle,1);
	osSemaphoreAcquire(xKey12Handle,1);
  /* Infinite loop */
  for(;;)
  {		
		if(osSemaphoreAcquire(xKey13Handle,1) == osOK){

			osSemaphoreRelease(xKey06Handle);
			osDelay(1000);
			//Auto park int to the ringht side
			osSemaphoreRelease(xKey16Handle);
			//go forward
			osSemaphoreRelease(xKey02Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(4)<370){
				osDelay(1);
			}
			//turn right and go backward
			osSemaphoreRelease(xKey06Handle);
			osDelay(200);
			osSemaphoreRelease(xKey10Handle);
			osSemaphoreRelease(xKey07Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(4)>-560){
				osDelay(1);
			}
			//go back straight
			osSemaphoreRelease(xKey03Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(3)>-440){
				osDelay(1);
			}
			//go back left
			osSemaphoreRelease(xKey05Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(3)>-530){
				osDelay(1);
			}
			//stop and turn
			osSemaphoreRelease(xKey06Handle);
			osSemaphoreRelease(xKey03Handle);
			osSemaphoreRelease(xKey16Handle);
			//end
			osDelay(2000);
			osSemaphoreRelease(xKey14Handle);
		}else if(osSemaphoreAcquire(xKey14Handle,1) == osOK){
			//set up from the side parking slot
			//go forward
			osSemaphoreRelease(xKey02Handle);
//			encoder_reset(3);
//			encoder_reset(4);
//			while(readencoderlength(3)<460){
//				osDelay(1);
//			}
			//turn left and continue to go 
			osSemaphoreRelease(xKey05Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(4)<460){
				osDelay(1);
			}
			//go straight
			osSemaphoreRelease(xKey03Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(3)<390){
				osDelay(1);
			}
			//go right
			osSemaphoreRelease(xKey07Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(3)<500){
				osDelay(1);
			}
			//turn then stop
			osSemaphoreRelease(xKey03Handle);
			osDelay(500);
			osSemaphoreRelease(xKey06Handle);
			//Uart2_printf("pid");
			//end

		}else if(osSemaphoreAcquire(xKey08Handle,1) == osOK){

			//Auto park to the back
			osSemaphoreRelease(xKey06Handle);
			osDelay(1000);
			osSemaphoreRelease(xKey16Handle);
			//go straight
			osSemaphoreRelease(xKey02Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(4)<40){
				osDelay(1);
			}
			//turn right and go back
			osSemaphoreRelease(xKey07Handle);
			osDelay(200);
			osSemaphoreRelease(xKey10Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(4)>-1320){
				osDelay(1);
			}
			//turn straight
			osSemaphoreRelease(xKey03Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(4)>-380){
				osDelay(1);
			}
			//stop!
			osSemaphoreRelease(xKey06Handle);
			osSemaphoreRelease(xKey16Handle);
			//end
			osDelay(2000);
			osSemaphoreRelease(xKey12Handle);
		}else if(osSemaphoreAcquire(xKey12Handle,1) == osOK){
			//go straight
			osSemaphoreRelease(xKey02Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(4)<320){
				osDelay(1);
			}
			//turn right
			osSemaphoreRelease(xKey07Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(4)<1320){
				osDelay(1);
			}
			//change distance
			osSemaphoreRelease(xKey07Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(4)<350){
				osDelay(1);
			}
			osSemaphoreRelease(xKey05Handle);
			encoder_reset(3);
			encoder_reset(4);
			while(readencoderlength(4)<320){
				osDelay(1);
			}
			//turn straight
			osSemaphoreRelease(xKey03Handle);
			Uart2_printf("pid");
			//end

		}
		
		osDelay(1);
  }
  /* USER CODE END LEDTask */
}

/* USER CODE BEGIN Header_WheelDriveTask */
/**
* @brief Function implementing the WheelDrive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WheelDriveTask */
void WheelDriveTask(void *argument)
{
  /* USER CODE BEGIN WheelDriveTask */
	osDelay(500);

	float base_speed = 0.4;
	
	pid_t motor_3_v_pid = {
		.setPoint = base_speed,
		.proportion = 500,
		.integral = 2700,
		.derivative = 0,
		.Ts = 0.01,
		.maxOutput = 1000,
		.minOutput = -1000,
		.base = 0,
		.sumILimit = 700,
	};
	pid_t motor_4_v_pid = {
		.setPoint = base_speed,
		.proportion = 500,
		.integral = 2700,
		.derivative = 0,
		.Ts = 0.01,
		.maxOutput = 1000,
		.minOutput = -1000,
		.base = 0,
		.sumILimit = 700,
	};
	//difference of length or velocity
	pid_t start_stable_pid = {
		.setPoint = 0,
		.proportion = 5,
		.integral = 2,
		.derivative = 0,
		.Ts = 0.01,
		.maxOutput = 1000,
		.minOutput = -1000,
		.base = 0,
		.sumILimit = 700,
	};
	float output_3;
	float output_4;
	float motor_3_velocity=0;
	float motor_4_velocity=0;
	int delta = 0;
	int delta_output = 0;
	int delta_lock = 1;
	//Encoder initialization
	encoder_init(3);
	encoder_init(4);
	int delta_clear_timer = 0;
  /* Infinite loop */
  for(;;)
  {
		
		
		if(osSemaphoreAcquire(xKey02Handle,1)==osOK){
			PID_Reset(&motor_3_v_pid);
			PID_Reset(&motor_4_v_pid);
			motor_3_v_pid.setPoint = base_speed;
			motor_4_v_pid.setPoint = base_speed;
		}else if(osSemaphoreAcquire(xKey10Handle,1)==osOK){
			PID_Reset(&motor_3_v_pid);
			PID_Reset(&motor_4_v_pid);
			motor_3_v_pid.setPoint = - base_speed;
			motor_4_v_pid.setPoint = - base_speed;
		}else if(osSemaphoreAcquire(xKey06Handle,1)==osOK){
			PID_Reset(&motor_3_v_pid);
			PID_Reset(&motor_4_v_pid);
			motor_3_v_pid.setPoint = 0;
			motor_4_v_pid.setPoint = 0;
			motor(3,0);
			motor(4,0);
			osDelay(200);
		}
		if(osSemaphoreAcquire(xKey15Handle,1) == osOK){
			encoder_reset(3);
			encoder_reset(4);
		}

		motor_3_velocity = readencoder(3);
		motor_4_velocity = readencoder(4);
		output_3 = PID_Calculate(&motor_3_v_pid,motor_3_velocity);
		output_4 = PID_Calculate(&motor_4_v_pid,motor_4_velocity);
		
//		if(delta_clear_timer>=300){
//			encoder_reset(3);
//			encoder_reset(4);
//			delta_clear_timer = 0;
////			delta_lock = 0;
//		}else{
//			delta_clear_timer++;
//		}
		if(delta_lock){
			delta = readencoderlength(3) - readencoderlength(4);
			delta_output = PID_Calculate(&start_stable_pid,delta);
		}else{
			delta_output = 0;
		}
		
		//output = PID_Calculate(&motor_1_pid,m1v);
//		printf("OK\r\n");
		motor(3,output_3+delta_output);
		motor(4,output_4-delta_output);
//		printf("v:%6.4f,%6.4f\r\n",motor_3_velocity,motor_4_velocity);
    osDelay(10);
  }
  /* USER CODE END WheelDriveTask */
}

/* USER CODE BEGIN Header_ADCtask */
/**
* @brief Function implementing the ADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADCtask */
void ADCtask(void *argument)
{
  /* USER CODE BEGIN ADCtask */
	float adc_power = 0;
	float adc_out = 0;
	kalman1_state adcfilter;
	kalman1_init(&adcfilter,11.5,0.1);
//	float filter[5];
  /* Infinite loop */
  for(;;)
  {
		adc_power = get_adc_value_pow()*(19.96+100.5)/19.96;
		adc_out = kalman1_filter(&adcfilter,adc_power);
//		filter[0] = get_adc_value_pow();
//		filter[0] = filter[0]*(19.96+100.5)/19.96;
//		adc_power = 0.1f*filter[0] + 0.2f*filter[1] + 0.4f*filter[2] + 0.2f * filter[3] + 0.1f * filter[4];
//		filter[4] = filter[3];
//		filter[3] = filter[2];
//		filter[2] = filter[1];
//		filter[1] = filter[0];
//		sprintf(output,"U:%5.3f,%5.3f\r\n",adc_power,adc_out);
//		Uart6_printf(output);
		osMessageQueuePut( oled_msgHandle,&adc_out, 0,1);
    osDelay(200);
  }
  /* USER CODE END ADCtask */
}

/* USER CODE BEGIN Header_OLEDdriveTask */
/**
* @brief Function implementing the OLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLEDdriveTask */
void OLEDdriveTask(void *argument)
{
  /* USER CODE BEGIN OLEDdriveTask */
	OLED_Display_On();
	OLED_Clear();
	char str[16];
	float temp = 0;
	float adcpower = 0;


	OLED_ShowString(0,2,(uint8_t *)"SIT-K13",0);
  /* Infinite loop */
  for(;;)
  {
		if(osMessageQueueGet( oled_msgHandle, &temp, 0, 5) == osOK){
			adcpower = temp;
		}

//      P()
		osMutexWait( I2C_1Handle , 0xffffff);
		sprintf(str,"Battery:%5.2fV",adcpower);
		OLED_ShowString(0,2,(uint8_t *)str,0);
		if(!ok){
			sprintf(str,"SystemNotOK! ");
		}else{
			sprintf(str,"SystemOK!    ");
		}
		OLED_ShowString(0,0,(uint8_t *)str,0);
		sprintf(str,"L:%6d",readencoderlength(3));
		OLED_ShowString(0,4,(uint8_t *)str,0);

		sprintf(str,"R:%6d",readencoderlength(4));
		OLED_ShowString(0,6,(uint8_t *)str,0);
		
		osMutexRelease( I2C_1Handle );
//      V()
		
    osDelay(50);
  }
  /* USER CODE END OLEDdriveTask */
}

/* USER CODE BEGIN Header_UARTTask */
/**
* @brief Function implementing the UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UARTTask */
void UARTTask(void *argument)
{
  /* USER CODE BEGIN UARTTask */
	unsigned char tempstr[128] = {0};
	unsigned int templength = 0;
	int turn = 0;
	int flag = 1;
  /* Infinite loop */
  for(;;)
  {
		////////////////UART6//////////////
		if(g_tUART_6.rec_end_flag==1){
			g_tUART_6.rec_end_flag = 0;
			GetBuf(&g_tUART_6,tempstr,&templength);
			printf("ECHO:%s",tempstr);
			///////////////////////////////////////
			if(tempstr[0]=='g'){
				if(tempstr[1]=='o'){
					osSemaphoreRelease(xKey02Handle);
				}
			}
			///////////////////////////////////////
			if(tempstr[0]=='s'){
				if(tempstr[1]=='t'){
					osSemaphoreRelease(xKey06Handle);
				}
			}
			///////////////////////////////////////
			if(tempstr[0]=='t'){
				if(tempstr[1]=='l'){
					osSemaphoreRelease(xKey05Handle);
				}
			}
			///////////////////////////////////////
			if(tempstr[0]=='t'){
				if(tempstr[1]=='r'){
					osSemaphoreRelease(xKey07Handle);
				}
			}
			///////////////////////////////////////
			if(tempstr[0]=='b'){
				if(tempstr[1]=='c'){
					osSemaphoreRelease(xKey10Handle);
				}
			}
			///////////////////////////////////////
			if(tempstr[0]=='d'){
				if(tempstr[1]=='i'){
					osSemaphoreRelease(xKey16Handle);
				}
			}
			///////////////////////////////////////
			if((tempstr[0] == '0')&&(tempstr[1] == '1')){
				osSemaphoreRelease(xKey01Handle);
			}else if((tempstr[0] == '0')&&(tempstr[1] == '2')){
				osSemaphoreRelease(xKey02Handle);
			}else if((tempstr[0] == '0')&&(tempstr[1] == '3')){
				osSemaphoreRelease(xKey03Handle);
			}else if((tempstr[0] == '0')&&(tempstr[1] == '4')){
				osSemaphoreRelease(xKey04Handle);
			}else if((tempstr[0] == '0')&&(tempstr[1] == '5')){
				osSemaphoreRelease(xKey05Handle);
			}else if((tempstr[0] == '0')&&(tempstr[1] == '6')){
				osSemaphoreRelease(xKey06Handle);
			}else if((tempstr[0] == '0')&&(tempstr[1] == '7')){
				osSemaphoreRelease(xKey07Handle);
			}else if((tempstr[0] == '0')&&(tempstr[1] == '8')){
				osSemaphoreRelease(xKey08Handle);
			}else if((tempstr[0] == '0')&&(tempstr[1] == '9')){
				osSemaphoreRelease(xKey09Handle);
			}else if((tempstr[0] == '1')&&(tempstr[1] == '0')){
				osSemaphoreRelease(xKey10Handle);
			}else if((tempstr[0] == '1')&&(tempstr[1] == '1')){
				osSemaphoreRelease(xKey11Handle);
			}else if((tempstr[0] == '1')&&(tempstr[1] == '2')){
				osSemaphoreRelease(xKey12Handle);
			}else if((tempstr[0] == '1')&&(tempstr[1] == '3')){
				osSemaphoreRelease(xKey13Handle);
			}else if((tempstr[0] == '1')&&(tempstr[1] == '4')){
				osSemaphoreRelease(xKey14Handle);
			}else if((tempstr[0] == '1')&&(tempstr[1] == '5')){
				osSemaphoreRelease(xKey15Handle);
			}else if((tempstr[0] == '1')&&(tempstr[1] == '6')){
				osSemaphoreRelease(xKey16Handle);
			}
			
			
			memset(tempstr,0,templength);
		}
		
		////////////////UART2////////////////
		if(g_tUART_2.rec_end_flag==1){
			g_tUART_2.rec_end_flag = 0;
			GetBuf(&g_tUART_2,tempstr,&templength);
			//printf("E2:%s\r\n",tempstr);
			///////////////////////////////////////
			if((tempstr[0] == '0')&&(tempstr[1] == '1')){
				osSemaphoreRelease(xKey08Handle);
				printf("get1\r\n");
			}
			if((tempstr[0] == '0')&&(tempstr[1] == '2')){
				osSemaphoreRelease(xKey13Handle);
				printf("get2\r\n");
			}
			
			if((tempstr[0] == 'o')&&(tempstr[1] == 'k')){
				ok = 1;
			}
			
			if(tempstr[0] == tempstr[1]){
				turn = 0;
				if(tempstr[0] == 'l'){
					if(tempstr[1] == 'l'){
						flag = -1;
					}
				}else if(tempstr[0] == 'r'){
					if(tempstr[1] == 'r'){
						flag = 1;
					}
				}
				if(tempstr[2]>='0'){
					turn = (tempstr[2] - '0')*10;
				}
				if(tempstr[3]>='0'){
					turn += (tempstr[3] - '0');
				}
				turn *= flag;
//				printf("dir:%d\r\n",turn);
				osMessageQueuePut( degreeHandle, &turn, 0,1);
				osMessageQueuePut( oled_dHandle, &turn, 0,1);
			}
			
			memset(tempstr,0,templength);
		}
    osDelay(1);
  }
  /* USER CODE END UARTTask */
}

/* USER CODE BEGIN Header_keyScanTask */
/**
* @brief Function implementing the keyScan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_keyScanTask */
void keyScanTask(void *argument)
{
  /* USER CODE BEGIN keyScanTask */
	char key_state = 0;
	int keypress = 0;
	int key_cnt = 0;
	int lastkey_cnt = 0;
	int llkey_cnt = 0;
	osSemaphoreId_t keypressSema;
  /* Infinite loop */
  for(;;)
  {
		keypress = keyscan();
		if(osSemaphoreAcquire(xKey04Handle,1) == osOK){
			key_state = !key_state;
		}
		if(keypress != 0){
			//key pressed!
			key_cnt = 1;
		}else{
			key_cnt = 0;
		}
		if((key_cnt == 1)&&(lastkey_cnt==1)&&(llkey_cnt == 0)){
			//one push
			printf("keypress:%d\r\n",keypress);
			//normal
			if(keypress==1){
				keypressSema=xKey01Handle;
			}else if(keypress==2){
				keypressSema=(xKey02Handle);
			}else if(keypress==3){
				keypressSema=(xKey03Handle);
			}else if(keypress==4){
				keypressSema=(xKey04Handle);
			}else if(keypress==5){
				keypressSema=(xKey05Handle);
			}else if(keypress==6){
				keypressSema=(xKey06Handle);
			}else if(keypress==7){
				keypressSema=(xKey07Handle);
			}else if(keypress==8){
				keypressSema=(xKey08Handle);
			}else if(keypress==9){
				keypressSema=(xKey09Handle);
			}else if(keypress==10){
				keypressSema=(xKey10Handle);
			}else if(keypress==11){
				keypressSema=(xKey11Handle);
			}else if(keypress==12){
				keypressSema=(xKey12Handle);
			}else if(keypress==13){
				keypressSema=(xKey13Handle);
			}else if(keypress==14){
				keypressSema=(xKey14Handle);
			}else if(keypress==15){
				keypressSema=(xKey15Handle);
			}else if(keypress==16){
				keypressSema=(xKey16Handle);
			}
			osSemaphoreRelease(keypressSema);
			while(keyscan()){
				if(keypressSema == xKey09Handle){
				  osSemaphoreRelease(keypressSema);
				}else if(keypressSema == xKey11Handle){
					osSemaphoreRelease(keypressSema);
				}
				osDelay(100);
			}
		}
		llkey_cnt = lastkey_cnt;
		lastkey_cnt = key_cnt;
    osDelay(5);
  }
  /* USER CODE END keyScanTask */
}

/* USER CODE BEGIN Header_servoTask */
/**
* @brief Function implementing the servo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servoTask */
void servoTask(void *argument)
{
  /* USER CODE BEGIN servoTask */
	float degree = 0;
	int temp = 0;	
  /* Infinite loop */
  for(;;)
  {
		if(osMessageQueueGet( degreeHandle, &temp, 0, 1) == osOK){
			degree = temp;
		}else if(osSemaphoreAcquire(xKey07Handle,1) == osOK){
			degree = 45;
		}else if(osSemaphoreAcquire(xKey05Handle,1) == osOK){
			degree = -50;
		}else if(osSemaphoreAcquire(xKey03Handle,1) == osOK){
			degree = 4;
		}else if(osSemaphoreAcquire(xKey11Handle,1) == osOK){
			degree+=5;
		}else if(osSemaphoreAcquire(xKey09Handle,1) == osOK){
			degree-=5;
		}else{
		}
		if(degree >= 45){
			degree = 45;
		}else if(degree <= -50){
			degree = -50;
		}
		servo(1,degree);
//		printf("servo:%d\r\n",(int)degree);
    osDelay(50);
  }
  /* USER CODE END servoTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Uart6_printf(char *str)
{
	while(*str!=0)
	{
	HAL_UART_Transmit(&huart6, (uint8_t *)str, 1, 0xffff);
		str++;
	}
}

void Uart2_printf(char *str)
{
	while(*str!=0)
	{
	HAL_UART_Transmit(&huart2, (uint8_t *)str, 1, 0xffff);
		str++;
	}
}

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
/* USER CODE END Application */

