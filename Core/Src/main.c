/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb.h"
#include "FOC.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ANGLE_RES 400
float delta_theta = _2PI / ANGLE_RES;
int y;
static void (*ADC_Callback_Handler)(ADC_HandleTypeDef *hadc) = NULL;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_FDCAN1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	__HAL_TIM_ENABLE(&htim1);  
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); 
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);   
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2); 
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3); 
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

	
	ADC_Handler_Init();
	
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); 	
	HAL_ADCEx_InjectedStart(&hadc2);
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	__HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);		
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//usb_printf("ok\n");
//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
//HAL_Delay(500);		

	//HAL_Delay(1);	
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	 if (ADC_Callback_Handler != NULL)
    {
        ADC_Callback_Handler(hadc);   // �����Զ�����У׼ �� ��������
    }	
}

void ADC_Handler_Init(void)
{
	//
    ADC_Callback_Handler = ADC_Calibration_Handler;	
}


#define ADC_CALIB_SAMPLES  100
float ADCvalue_Offset[3];
float ADCvalue[3];
uint16_t cnt;

static void ADC_Calibration_Handler(ADC_HandleTypeDef *hadc)
{
	static uint16_t calib_count = 0;
	static float  sum[3] = {0};
	sum[0] += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) * 3.3f / 4095.0f*2.5f; 
	sum[1] += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2) * 3.3f / 4095.0f*2.5f;	
	sum[2] += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3) * 3.3f / 4095.0f*2.5f;	
	calib_count++;
	if (calib_count >= ADC_CALIB_SAMPLES)
	{
        ADCvalue_Offset[0] = sum[0] / ADC_CALIB_SAMPLES;
        ADCvalue_Offset[1] = sum[1] / ADC_CALIB_SAMPLES;
        ADCvalue_Offset[2] = sum[2] / ADC_CALIB_SAMPLES;
//        USART1_Printf("ADC Offset: ia=%.4f, ib=%.4f, ic=%.4f\n",
//                      ADCvalue_Offset[0], ADCvalue_Offset[1], ADCvalue_Offset[2]);	
        // �л����������д�����
        ADC_Callback_Handler = ADC_Normal_Handler;	
	}

}

static void ADC_Normal_Handler(ADC_HandleTypeDef *hadc)
{
	ADCvalue[0]= HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1)* 3.3f / 4095.0f*2.5f  - ADCvalue_Offset[0];//t1:start
	ADCvalue[1]= HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2)* 3.3f / 4095.0f*2.5f  - ADCvalue_Offset[1];
	ADCvalue[2]= HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3)* 3.3f / 4095.0f*2.5f  - ADCvalue_Offset[2];	
	cnt= __HAL_TIM_GET_COUNTER(&htim3);
	
	y%=400;
	y++;
	setPhaseVoltage(1,0,y*delta_theta);
	usb_printf("%4f,%4f,%4f,%d\n",ADCvalue[0],ADCvalue[1],ADCvalue[2],cnt);	
	
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
