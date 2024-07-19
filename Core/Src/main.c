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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/**
 * The SPEED function scales the input value POTI_VALUE, which is in the range of in_min to in_max,
 * to a new range of out_min to out_max. This is useful when you want to scale an analog input value
 * (like the value read from a potentiometer) to a different range.
 *
 * @param POTI_VALUE The input value to be scaled. This value should be in the range of in_min to in_max.
 * @return The scaled value, which is in the range of out_min to out_max.
 */
int SPEED(int POTI_VALUE)
{
    int in_min = 0; // The minimum value of the input range
    int in_max = 1023; // The maximum value of the input range
    int out_min = 0; // The minimum value of the output range
    int out_max = 10; // The maximum value of the output range

    // Scaling the input value to the output range
    return (POTI_VALUE - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Liest den aktuellen Wert des Potentiometers
 *
 * Diese Funktion startet eine ADC-Konvertierung und liest den aktuellen Wert
 * des Potentiometers. Die Funktion blockiert, bis die Konvertierung abgeschlossen ist.
 *
 * @return Der gelesene ADC-Wert
 */
uint32_t POTI_Read() {
  /* Startet die ADC-Konvertierung */
  HAL_ADC_Start(&ADCx);

  /* Wartet, bis die Konvertierung abgeschlossen ist */
  HAL_ADC_PollForConversion(&ADCx, 100);

  /* Gibt den gelesenen ADC-Wert zurück */
  return HAL_ADC_GetValue(&ADCx);
}

/**
 * @brief This function controls the movement of a motor.
 * 
 * @param pinState The state of the DIR pin, which can be either GPIO_PIN_RESET or GPIO_PIN_SET.
 * 
 * The function performs the following steps:
 * 1. Sets the EN_Pin to GPIO_PIN_RESET to enable the driver chip.
 * 2. Sets the DIR_Pin to the state specified by pinState. This determines the direction of rotation of the motor.
 * 3. Sets the STEP_Pin to GPIO_PIN_SET to trigger a step of the motor.
 * 4. Waits for a duration determined by SPEED(POTI_Read()). This determines the speed of the motor.
 * 5. Sets the STEP_Pin to GPIO_PIN_RESET to indicate the end of the step.
 * 6. Waits again for the same duration to prepare for the next step.
 */
void TURN(GPIO_PinState pinState)
{
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, pinState);
  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
  HAL_Delay(SPEED(POTI_Read()));
  HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
  HAL_Delay(SPEED(POTI_Read()));
}

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int stepsL = 0;
  int stepsR = 0;
  int TASTE_L_DISABLE = 0;
  int TASTE_R_DISABLE = 0;
  while (1)
  {
    // Check if both the left and right buttons are not pressed
    if(HAL_GPIO_ReadPin(TASTE_L_GPIO_Port, TASTE_L_Pin) == HAL_GPIO_ReadPin(TASTE_R_GPIO_Port, TASTE_R_Pin))
    {
      // If no buttons are pressed, disable the driver chip
      HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
    }

    // While the left button is pressed and the right button is not pressed and the interval between steps is greater than 1 millisecond
    while(HAL_GPIO_ReadPin(TASTE_L_GPIO_Port, TASTE_L_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(TASTE_R_GPIO_Port, TASTE_R_Pin) == GPIO_PIN_RESET && SPEED(POTI_Read())>1 && TASTE_L_DISABLE != 1 ) 
    {
      TASTE_R_DISABLE=0;
      stepsR = 0;
      // Turn on the driver chip and set the direction to RESET (counterclockwise)
      TURN(GPIO_PIN_RESET);

      // Increment the number of steps taken
      stepsL++;

      if(stepsL >= 800 && HAL_GPIO_ReadPin(TASTE_L_GPIO_Port, TASTE_L_Pin) == GPIO_PIN_SET)
      {
        stepsL = 0; // Reset the number of steps when the left button is released
        TASTE_L_DISABLE=1;
        HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);

        while (HAL_GPIO_ReadPin(TASTE_L_GPIO_Port, TASTE_L_Pin) == GPIO_PIN_SET)
        {
          //Do nothing
        }
      }

    }

    // While the right button is pressed and the right button is not pressed and the interval between steps is greater than 1 millisecond
    while(HAL_GPIO_ReadPin(TASTE_L_GPIO_Port, TASTE_L_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(TASTE_R_GPIO_Port,TASTE_R_Pin) == GPIO_PIN_SET && SPEED(POTI_Read())>1 && TASTE_R_DISABLE != 1)
    {
      TASTE_L_DISABLE=0;
      stepsL = 0;
      // Turn on the driver chip and set the direction to SET (clockwise)
      TURN(GPIO_PIN_SET);

      // Increment the number of steps taken
      stepsR++;

      if(stepsR >= 800 && HAL_GPIO_ReadPin(TASTE_R_GPIO_Port, TASTE_R_Pin) == GPIO_PIN_SET)
      {
        stepsR = 0; // Reset the number of steps when the left button is released
        TASTE_R_DISABLE=1;
        HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
        while (HAL_GPIO_ReadPin(TASTE_R_GPIO_Port, TASTE_R_Pin) == GPIO_PIN_SET)
        {
          //Do nothing
        }
      }
    }

    // The keys are prioritized over the MVQ sensors in order to be able to create errors
    // While MVQ wants to turn left, the right button is not pressed and the interval between steps is greater than 1 milliseconds
    while(HAL_GPIO_ReadPin(MVQ_L_GPIO_Port, MVQ_L_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(MVQ_R_GPIO_Port, MVQ_R_Pin) == GPIO_PIN_RESET && SPEED(POTI_Read())>1 && HAL_GPIO_ReadPin(TASTE_R_GPIO_Port, TASTE_R_Pin) == GPIO_PIN_RESET)
    {
      TASTE_R_DISABLE=0;
      TURN(GPIO_PIN_RESET);
    }
    
    // While MVQ wants to turn right, the left button is not pressed and the interval between steps is greater than 1 milliseconds
    while(HAL_GPIO_ReadPin(MVQ_L_GPIO_Port, MVQ_L_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(MVQ_R_GPIO_Port, MVQ_R_Pin) == GPIO_PIN_SET && SPEED(POTI_Read())>1 && HAL_GPIO_ReadPin(TASTE_L_GPIO_Port, TASTE_L_Pin) == GPIO_PIN_RESET)
    {
      TASTE_L_DISABLE=0;
      TURN(GPIO_PIN_SET);
    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|DIR_Pin|STEP_Pin|MS3_Pin
                          |EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MS2_Pin|MS1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SLP_GPIO_Port, SLP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : TASTE_L_Pin */
  GPIO_InitStruct.Pin = TASTE_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TASTE_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin STEP_Pin MS3_Pin MS2_Pin
                           MS1_Pin EN_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|STEP_Pin|MS3_Pin|MS2_Pin
                          |MS1_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TASTE_R_Pin */
  GPIO_InitStruct.Pin = TASTE_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TASTE_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SLP_Pin */
  GPIO_InitStruct.Pin = SLP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SLP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MVQ_R_Pin MVQ_L_Pin */
  GPIO_InitStruct.Pin = MVQ_R_Pin|MVQ_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
