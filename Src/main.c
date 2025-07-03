/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "stdio.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
const motor_measure_t *motor_2006_t;
spin_record_typedef spin_record_t;
int set_speed = 1000;
struct {
	float kp;
	float ki;
	float kd;
	int16_t err;
	int16_t out;
}pid_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int64_t motor_ecd_handler(void)
{
	static uint16_t temp_last_ecd; 
	static int ecd_count ;  		//��̬���ۼ�
	static int64_t total_ecd;
	
	int32_t diff = (int32_t)motor_2006_t->ecd - (int32_t)temp_last_ecd;
	
	if(diff > 4096)	 
	{
		ecd_count--;
		diff -= 8192;
		
	}
	if(diff < -4096) 
	{
		ecd_count++;
		diff += 8192;
	}
	
	total_ecd += diff;
	temp_last_ecd = motor_2006_t->ecd;
	return total_ecd;
}

void pid_cacl(int16_t set,int16_t feed){
	static int16_t iout;
	pid_t.err = set - feed;
	int32_t pout = (pid_t.kp*pid_t.err);
	if(pid_t.err > 20)iout += (int16_t)(pid_t.ki*pid_t.err);
	else iout = 0;
	int16_t dout = (int16_t)(pid_t.kd*pid_t.err);
	
	int32_t out = pout+iout+dout;
	
	if(out >10000)out = 10000;
	else if(out < -10000)out = -10000;
	
	pid_t.out = (int16_t)out;
}

void motor_foward(){
	pid_cacl(set_speed,motor_2006_t->speed_rpm);
	CAN_cmd_chassis(pid_t.out,0,0,0);
}

void motor_back(){
	pid_cacl(-set_speed,motor_2006_t->speed_rpm);
	CAN_cmd_chassis(pid_t.out,0,0,0);
}
void motor_stop(){
	pid_cacl(0,motor_2006_t->speed_rpm);
	CAN_cmd_chassis(pid_t.out,0,0,0);
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	int init_flag = 1;
	float init_degree = 0;
	motor_2006_t = get_chassis_motor_measure_point(0);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
    can_filter_init();
	pid_t.kp = 10;
	set_speed = 8000;
	HAL_Delay(2000);
	usb_printf("will begin after 1 second ....");
	HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  spin_record_t.output_degree = (motor_ecd_handler()/8192.0f * 360.0f)/36.0f;
	  
	  if(init_flag == 1) 
	  {
		  init_degree = spin_record_t.output_degree;
		  init_flag = 0;
	  }
	  
//	  if(spin_record_t.output_degree < 360.0f * cnt){
//			motor_foward();
//		  
//	  }
//	  else motor_stop();
//	  
//	  HAL_Delay(1);
	  
		if (spin_record_t.cnt_120 < 300000) {
			if (spin_record_t.foward_flag == 1) { // ��ת
				if (spin_record_t.output_degree < 120.0f + init_degree) {
					 motor_foward();// ��ת���
				} else {  
					motor_stop();
					spin_record_t.foward_flag = 0; // �л�Ϊ��ת

				}
			} else { // ��ת
				if (spin_record_t.output_degree > 0.0f + init_degree) {
					 motor_back();// ��ת���
				} else {
					motor_stop();
					spin_record_t.foward_flag = 1; // �л�Ϊ��ת
					spin_record_t.cnt_120++;
					usb_printf("120count: %d / 300000",spin_record_t.cnt_120);
				}
			}
		}
		if (spin_record_t.cnt_540 < 3000) {
			if (spin_record_t.foward_flag == 1) { // ��ת
				if (spin_record_t.output_degree < 540.0f +init_degree) {
					 motor_foward();// ��ת���
				} else {
					motor_stop();
					spin_record_t.foward_flag = 0; // �л�Ϊ��ת

				}
			} else { // ��ת
				if (spin_record_t.output_degree > 0.0f + init_degree) {
					 motor_back();// ��ת���
				} else {
					motor_stop();
					spin_record_t.foward_flag = 1; // �л�Ϊ��ת
					spin_record_t.cnt_540++;
					usb_printf("540count: %d / 3000",spin_record_t.cnt_540);
				}
			}
		}
		if (spin_record_t.cnt_540 >= 3000 && spin_record_t.cnt_120 >= 300000)motor_stop();
	  HAL_Delay(1);
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
