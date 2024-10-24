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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "I2C_LCD.h"
#include "I2C_LCD_cfg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY 200

#define MyI2C_LCD I2C_LCD_1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int Statesystem = 0;
int State = 0;
int Button;
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
volatile uint32_t counter = 0; /*Sensor counter*/
volatile uint32_t Push = 0;
int SystemActive = 0;
uint16_t ADC_Data = 0;
int Detect;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t debounceButton(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	I2C_LCD_Init(MyI2C_LCD);
	I2C_LCD_SetCursor(MyI2C_LCD, 3, 0);
	I2C_LCD_WriteString(MyI2C_LCD, "Push Button");
	I2C_LCD_SetCursor(MyI2C_LCD, 3, 1);
	I2C_LCD_WriteString(MyI2C_LCD, "Start");



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (debounceButton(GPIOA, GPIO_PIN_9)) {
			Push++;
			if (Push > 3) {
				Push = 0; // Cycle back to state 0
			}

			switch (Push) {
			case 0: // System not operating
				SystemActive = 0;
				counter = 0;
				I2C_LCD_SetCursor(MyI2C_LCD, 3, 0);
				I2C_LCD_WriteString(MyI2C_LCD, "Push Button");
				I2C_LCD_SetCursor(MyI2C_LCD, 3, 1);
				I2C_LCD_WriteString(MyI2C_LCD, "Start");
				I2C_LCD_SetCursor(MyI2C_LCD, 8, 1);
				I2C_LCD_WriteString(MyI2C_LCD, "              ");
				break;

			case 1: // System operating
				SystemActive = 1;
				I2C_LCD_SetCursor(MyI2C_LCD, 3, 0);
				I2C_LCD_WriteString(MyI2C_LCD, "Detect:");
				I2C_LCD_SetCursor(MyI2C_LCD, 3, 1);
				I2C_LCD_WriteString(MyI2C_LCD, "counter:");
				break;

			case 2: // System stopped
				SystemActive = 0;
				I2C_LCD_SetCursor(MyI2C_LCD, 3, 0);
				I2C_LCD_WriteString(MyI2C_LCD, "Detect:");
				I2C_LCD_SetCursor(MyI2C_LCD, 3, 1);
				I2C_LCD_WriteString(MyI2C_LCD, "counter:");
				break;

			case 3: // System continues operation
				SystemActive = 1;
				I2C_LCD_SetCursor(MyI2C_LCD, 3, 0);
				I2C_LCD_WriteString(MyI2C_LCD, "Detect:");
				I2C_LCD_SetCursor(MyI2C_LCD, 3, 1);
				I2C_LCD_WriteString(MyI2C_LCD, "counter:");
				break;
			}
		}



	  	/*#Alarm Buzzer#*/
		HAL_ADC_Start_DMA(&hadc1, &ADC_Data, 1);
		HAL_Delay(50);
		if (ADC_Data >= 0 && ADC_Data < 2900) { //ADC = 2913 , Voltage = 9.55V
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_Delay(250);
		}

		if (ADC_Data >= 2900) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		}


		/*#Button Reset Counter#*/
	    if (debounceButton(GPIOA, GPIO_PIN_10)) {
	        counter = 0;  // Reset the counter
	        // Update the display immediately after resetting
	        char counterStr[10];
	        snprintf(counterStr, sizeof(counterStr), "%lu", counter);
	        I2C_LCD_SetCursor(MyI2C_LCD, 11, 1);
	        I2C_LCD_WriteString(MyI2C_LCD, counterStr);
	        I2C_LCD_SetCursor(MyI2C_LCD, 11, 1);
	        I2C_LCD_WriteString(MyI2C_LCD, "              ");
	    }


//	    /*#Button Start and Stop operation#*/
//		if (debounceButton(GPIOA, GPIO_PIN_9))
//		{
//			Push++;
//			if (Push > 3) {
//				Push = 0;
//			}
//		}

//		if (Push == 0 && State == 0)
//		{
//			SystemActive = 0;
//			State = 1;
//			I2C_LCD_SetCursor(MyI2C_LCD, 3, 0);
//			I2C_LCD_WriteString(MyI2C_LCD, "Push Button");
//			I2C_LCD_SetCursor(MyI2C_LCD, 3, 1);
//			I2C_LCD_WriteString(MyI2C_LCD, "Start"); // Clear the second line
//			I2C_LCD_SetCursor(MyI2C_LCD, 8, 1);
//			I2C_LCD_WriteString(MyI2C_LCD, "              ");
//		}
//		if (Push == 1 && State == 1)
//		{
//			SystemActive = 1;
//			State = 2;
//		}
//		if(Push == 2 && State == 2)
//		{
//			SystemActive = 2;
//			State = 3;
//		}
//		if (Push == 3 && State == 3) {
//			SystemActive = 3;
//			State = 0;
//		}


		/*#When system operation#*/
		if (SystemActive)

		{
			I2C_LCD_SetCursor(MyI2C_LCD, 3, 0);
			I2C_LCD_WriteString(MyI2C_LCD, "Detect:");
			I2C_LCD_SetCursor(MyI2C_LCD, 3, 1);
			I2C_LCD_WriteString(MyI2C_LCD, "counter:");
			Detect = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
			if (Detect == 0) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				I2C_LCD_SetCursor(MyI2C_LCD, 10, 0);
				I2C_LCD_WriteString(MyI2C_LCD, "No");
				I2C_LCD_SetCursor(MyI2C_LCD, 12, 0);
				I2C_LCD_WriteString(MyI2C_LCD, "              ");
			}
			else if (Detect == 1) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				I2C_LCD_SetCursor(MyI2C_LCD, 10, 0);
				I2C_LCD_WriteString(MyI2C_LCD, "Yes");
			}

			// Display the counter value
			char counterStr[10];
			snprintf(counterStr, sizeof(counterStr), "%lu", counter);
			I2C_LCD_SetCursor(MyI2C_LCD, 11, 1);
			I2C_LCD_WriteString(MyI2C_LCD, counterStr);
		}

		HAL_Delay(1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*# Debounce Function #*/
uint8_t debounceButton(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	static uint32_t lastDebounceTime = 0;
	uint8_t buttonState = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
	if (buttonState == 1) {
		if (HAL_GetTick() - lastDebounceTime > DEBOUNCE_DELAY) {
			lastDebounceTime = HAL_GetTick();
			return 1; // Button is pressed and debounced
		}
	}
	return 0; // Button not pressed or still bouncing
}

/*#Use Interrupt on PinA8 for SENSOR Counter#*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_8 && SystemActive)
	{
		currentMillis = HAL_GetTick();
		if(currentMillis - previousMillis > DEBOUNCE_DELAY)
		{
		counter++;
		previousMillis = currentMillis;
		}
	}
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

