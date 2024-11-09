/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
DAC_ChannelConfTypeDef sConfig;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/**
 * Part 1
 */
//// sample every 0.3 ms from an array of size 50
// int index = 0;
//// triangle
//uint16_t triangle_wave[50] = {0, 163, 327, 491, 655, 819, 982, 1146, 1310, 1474, 1638, 1801, 1965, 2129, 2293, 2457,
//                              2620, 2784, 2948, 3112, 3276, 3439, 3603, 3767, 3931, 4095, 3931, 3767, 3603, 3439, 3276,
//                              3112, 2948, 2784, 2620, 2457, 2293, 2129, 1965, 1801, 1638, 1474, 1310, 1146, 982, 819,
//                              655, 491, 327, 163};
//uint16_t triangle_signal;
//// saw
//uint16_t sawtooth_wave[50] = {0, 81, 163, 245, 327, 409, 491, 573, 655, 737, 819, 900, 982, 1064, 1146, 1228, 1310,
//                              1392, 1474, 1556, 1638, 1719, 1801, 1883, 1965, 2047, 2129, 2211, 2293, 2375, 2457, 2538,
//                              2620, 2702, 2784, 2866, 2948, 3030, 3112, 3194, 3276, 3357, 3439, 3521, 3603, 3685, 3767,
//                              3849, 3931, 4013};
//uint16_t saw_signal;
//// sine
//uint16_t sine_wave[50];
//uint16_t sine_signal;






/*
 * Part 2 Step 2
 * We want to achieve a 44.1 kHz sample rate therefore the counter period should be (clock frequency)/(44.1 kHz)
 * which means that the period of the counter should be 2721
 * thus with this counter we can reproduced a signal with a maximum frequency of 22kHz
 */
//uint16_t array[29]; //we want to reproduce a tone of 1.5kHz therefore there will be a total of 29 sample for 1 period
//int array_index = 0;


/**
 * Part 2 Step 3 and 4
 */
int tone; //keeps track of tone
int C6_size = 44; // C6 is 1 kHz, so 44 samples per period
int E6_size = 33; // E6 is 1.3 kHz, so 33 samples per period
int G6_size = 28; // G6 is 1.57 kHz, so 28 samples per period
uint16_t C6_data[44];
uint16_t E6_data[33];
uint16_t G6_data[28];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == my_Button_Pin) {

		//Everytime we press we change tone
		tone = (tone + 1)%3;
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

		//C6 tone
		if (tone == 0) {
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) C6_data, (uint32_t) C6_size, DAC_ALIGN_12B_R);

		//E6 tone
		} else if (tone == 1) {
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) E6_data, (uint32_t) E6_size, DAC_ALIGN_12B_R);

		//G6 tone
		} else if (tone == 2) {
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) G6_data, (uint32_t) G6_size, DAC_ALIGN_12B_R);
		}

		HAL_GPIO_TogglePin(my_Led_GPIO_Port, my_Led_Pin);
	}
}

/**
 * Part 2 Step 2 Implemennting timer
 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim == &htim2) {
//		uint16_t signal = array[array_index];
//		array_index = (array_index+1)%29;
//		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, signal);
//
//	}
//}


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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_DACEx_SelfCalibrate(&hdac1, &sConfig, DAC1_CHANNEL_1);

//  /**
//   * demo 1
//   */
//  // generate sine array of size 50
//  float segment = 2 * PI / 50;
//  for (int i = 0; i < 50; i++) {
//      float angle = i * segment;
//      float sine_value = arm_sin_f32(angle);
//      sine_wave[i] = (uint16_t)((sine_value + 1.0f)*(4094/2));
//  }

/**
 * Part 1 wave on DAC
 */
//  // Start the DAC for both channels
//  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim2);

/**
 * Part 2 Sine wave generation Step 2
 */
//  for (int i = 0; i < 29; i++) {
//	  float angle = 2 * PI * i / 29;
//	  sine_array[i] = (uint16_t)((arm_sin_f32(angle) + 1) * 1365);
//  }

  /**
   *  Populate array of tones (part 2, step 3 & 4)
   */
  //
  for (int i = 0; i < C6_size; i++) {
	  float angleC6 = 2 * PI * i / C6_size;
	  C6_data[i] = (uint16_t) ((arm_sin_f32(angleC6)+1)*(1365)); // 1365 multiplier as 4095 max output, max sine output of 2, scale down to 2/3 to reduce distortion i.e. (4095/2)*(2/3)
  }
  for (int i = 0; i < E6_size; i++) {
	  float angleE6 = 2 * PI * i / E6_size;
	  E6_data[i] = (uint16_t) ((arm_sin_f32(angleE6)+1)*(1365));
  }
  for (int i = 0; i < G6_size; i++) {
	  float angleG6 = 2*PI*i/ G6_size;
	  G6_data[i] = (uint16_t) ((arm_sin_f32(angleG6)+1)*(1365));
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  pt 1
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//    // get values from arrays
//    saw_signal = saw_wave[index];
//    triangle_signal = triangle_wave[index];
//    sine_signal = sine_wave[index];
//    index = index + 1;
//    // out the signals to the DAC
////    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, triangle_signal);
//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, saw_signal);
//    // I think it's ok?
////    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sine_signal);
//
//    // freq
//    HAL_Delay(0.3);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2721;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(my_Led_GPIO_Port, my_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : my_Button_Pin */
  GPIO_InitStruct.Pin = my_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(my_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : my_Led_Pin */
  GPIO_InitStruct.Pin = my_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(my_Led_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
