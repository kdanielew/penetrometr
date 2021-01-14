/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h> //dodaje biblioteke do obliczen termistora
#include <stdbool.h>
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

/* USER CODE BEGIN PV */
bool state = true; //zmienna pomocnicza do debouncingu

volatile uint32_t PomiarADC = 0; //zmienna do przechowywania pomiarów z termistora

float Rt;
float Temperature;
float Vsense;
float TinC;

const float R = 6660; // [Om]
const float SupplyVoltage = 3.3; // [Volts]
const float ADCResolution = 4095.0;
const float B = 3977; // [K]
const float T0 = 298.15; //[stp C]

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC4_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc4, &PomiarADC, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  Vsense = (SupplyVoltage*PomiarADC)/ADCResolution;
	  Rt = -((Vsense*R)/(Vsense-SupplyVoltage));//przeliczenie na opór termistora
	  Temperature = B/(log(Rt/(R*(pow((2.718),(-B/T0))))));// Obliczenie temperatury
	  TinC = Temperature - 273.15 ; //Zmiana temperatury z Kelwinow na Celcjusze


/*jezeli z przerwania zostanie uruchomiona dioda, warunek if powoduje odliczanie x milisekund okreslonych w HAL_Delay (ponizej wytłumaczony sposob odliczania),
 * po tym czasie wylacza diode i przekaznik */

	  if (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) == GPIO_PIN_SET){
	  	HAL_Delay(5000);
	  	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	  	HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);
	  	}

	  else if (HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin) == GPIO_PIN_SET){
	  	HAL_Delay(5000);
	  	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  	HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);
	  	}

	  else if (HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin) == GPIO_PIN_SET){
	  	HAL_Delay(5000);
	  	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	  	HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);
	  	}

	  else if((HAL_GPIO_ReadPin(SWITCH4_GPIO_Port, SWITCH4_Pin) == GPIO_PIN_RESET ) && (HAL_GPIO_ReadPin(LED4_GPIO_Port, LED4_Pin) == 1)){
		HAL_Delay(300); //w przypadku przyciku on-off drugi warunek sprawdzajacy po 300 ms czy przycisk faktycznie wcisniety (debouncing)

	  	if ((HAL_GPIO_ReadPin(SWITCH4_GPIO_Port, SWITCH4_Pin) == GPIO_PIN_RESET ) && (HAL_GPIO_ReadPin(LED4_GPIO_Port, LED4_Pin) == 1)){
	  		HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);
	  		}
	  	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* przerwania wywolane nacisnieciem przycisku powoduja wlaczenie odpowiadajacej diody led i przekaznika,
 * dodatkowo zaimplementowany debouncing w postaci timera TIM1 o okresie 50 ms
 * timer obliczany jest wg:
 * częstotliwosć (freq) = 32 000 000 Hz
 * a prescaler = 32 000
 * stad czestotliwosc TIM1 = 32 000 000 Hz / 32 000 = 1000 Hz
 * żeby uzyskać okres licznika 0,05 s musimy ustawić counter period na 50, wynika to z proporcji:
 * 1 sekunda to 1000 tyknięć, więc 0,05 sekundy to x
 * (0,05 s * 1000 tyknięć)/1 sekundę = 50 tyknięć  */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  if(GPIO_Pin == GPIO_PIN_0 && state == true && ((HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin)||HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin)||HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin)||HAL_GPIO_ReadPin(LED4_GPIO_Port, LED4_Pin)) == 0))
  {
	  HAL_TIM_Base_Start_IT(&htim1); //licznik zaczyna odliczać
	  state = false;}

  else if(GPIO_Pin == GPIO_PIN_1 && ((HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin)||HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin)||HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin)||HAL_GPIO_ReadPin(LED4_GPIO_Port, LED4_Pin)) == 0))
  {
	  HAL_TIM_Base_Start_IT(&htim1);
	  state = false;}

  else if(GPIO_Pin == GPIO_PIN_2 && ((HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin)||HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin)||HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin)||HAL_GPIO_ReadPin(LED4_GPIO_Port, LED4_Pin)) == 0))
  {
	  HAL_TIM_Base_Start_IT(&htim1);
	  	  state = false;}
  else if(GPIO_Pin == GPIO_PIN_4 &&((HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin)||HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin)||HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin) == 0)))
  {
	  HAL_TIM_Base_Start_IT(&htim1);
	  state = false;
   }
  else {
	  // No Operation does nothing.
	 __NOP();}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

	if(HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin) == GPIO_PIN_RESET){
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_SET);
		state = true;
		HAL_TIM_Base_Stop_IT(&htim1);
	}
	else if(HAL_GPIO_ReadPin(SWITCH4_GPIO_Port, SWITCH4_Pin) == GPIO_PIN_RESET){
		HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_SET);
		state = true;
		HAL_TIM_Base_Stop_IT(&htim1);
	}
	else if(HAL_GPIO_ReadPin(SWITCH3_GPIO_Port, SWITCH3_Pin) == GPIO_PIN_RESET){
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_SET);
			state = true;
			HAL_TIM_Base_Stop_IT(&htim1);
		}
	else if(HAL_GPIO_ReadPin(SWITCH2_GPIO_Port, SWITCH2_Pin) == GPIO_PIN_RESET){
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_SET);
			state = true;
			HAL_TIM_Base_Stop_IT(&htim1);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
