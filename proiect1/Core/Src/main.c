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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define R1_PORT GPIOB
#define R1_PIN GPIO_PIN_4

#define R2_PORT GPIOB
#define R2_PIN GPIO_PIN_9

#define R3_PORT GPIOB
#define R3_PIN GPIO_PIN_12

#define R4_PORT GPIOC
#define R4_PIN GPIO_PIN_2

#define C1_PORT GPIOC
#define C1_PIN GPIO_PIN_12

#define C2_PORT GPIOC
#define C2_PIN GPIO_PIN_11

#define C3_PORT GPIOC
#define C3_PIN GPIO_PIN_10

#define PIN_LENGTH 4
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char pinBuffer[PIN_LENGTH] = {0};
uint8_t pinIndex = 0;
const char correctPin[PIN_LENGTH] = {'1', '2', '3', '4'}; // Codul PIN corect
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t segmentNumber[10] = {
        0x3f,  // 0
        0x06,  // 1
        0x5b,  // 2
        0x4f,  // 3
        0x66,  // 4
        0x6d,  // 5
        0x7d,  // 6
        0x07,  // 7
        0x7f,  // 8
        0x67   // 9
};


void SevenSegment_Update(uint8_t number){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, ((number>>0)&0x01));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, ((number>>1)&0x01));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, ((number>>2)&0x01));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, ((number>>3)&0x01));
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, ((number>>4)&0x01));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, ((number>>5)&0x01));
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, ((number>>6)&0x01));
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t key;

	void resetPinBuffer(void) {
	    for (uint8_t i = 0; i < PIN_LENGTH; i++) {
	        pinBuffer[i] = 0;
	    }
	    pinIndex = 0;
	}

	void checkPin(void) {
	    if (memcmp(pinBuffer, correctPin, PIN_LENGTH) == 0) {
	        // Cod PIN corect
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // Aprinde LED
	    } else {
	        // Cod PIN incorect
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Stinge LED
	    }
	    resetPinBuffer(); // Resetează buffer-ul după verificare
	}

	uint8_t read_keypad (void){


			HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);

			if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))
			{
				while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));
				return 1;
			}

			if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))
			{
				while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));
				return 2;
			}

			if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))
			{
				while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));
				return 3;
			}



			HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);

			if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))
			{
				while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));
				return 4;
			}

			if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))
			{
				while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));
				return 5;
			}

			if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))
			{
				while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));
				return 6;
			}


			HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);

			if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))
			{
				while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));
				return 7;
			}

			if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))
			{
				while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));
				return 8;
			}

			if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))
			{
				while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));
				return 9;
			}


			HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_RESET);

			if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))
			{
				while (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)));
				return 42;
			}

			if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))
			{
				while (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)));
				return 0;
			}

			if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))
			{
				while (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)));
				return 35;
			}

			 return 0xFF;
		}
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /* Așteaptă până când este apăsată o tastă */
	   key = read_keypad();

	   /* Dacă o tastă a fost apăsată și nu am ajuns la lungimea maximă a PIN-ului */
	   if (key != 0xFF) // Verifică dacă a fost citită o valoare validă
	       {
	           if (pinIndex < PIN_LENGTH)
	           {
	               pinBuffer[pinIndex++] = key + '0'; // Convertește numărul în caracter
	               SevenSegment_Update(segmentNumber[key]); // Afișează pe display
	               HAL_Delay(500); // Așteaptă 500ms pentru a putea vedea cifra pe display
	           }

	           if (pinIndex == PIN_LENGTH) // Dacă s-a introdus întregul PIN
	           {
	               checkPin(); // Verifică codul PIN
	               HAL_Delay(500); // Așteaptă 500ms pentru a vedea rezultatul
	               pinIndex = 0; // Resetează indexul pentru următorul PIN
	           }
	       }
	       else
	       {
	           SevenSegment_Update(0x00); // Curăță display-ul dacă nu este apăsată nicio tastă
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_6
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 PA8
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB4 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
