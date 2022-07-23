/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>

bool flag = 0;
uint32_t Time = 0;

volatile uint8_t Counter = 0;

volatile uint32_t SysTimer_ms = 0; //Переменная, аналогичная HAL_GetTick();
volatile uint32_t Delay_counter_ms = 0; //Счетчик для функции Delay_ms
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Delay_ms(uint32_t Milliseconds) {
	Delay_counter_ms = Milliseconds;
	while (Delay_counter_ms != 0) ;
}

void SysTick_Handler(void) {
	SysTimer_ms++;
	
	if (Delay_counter_ms) {
		Delay_counter_ms--;
	}
	
}


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
  //HAL_Init();
	//Настройка системного таймера
	CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);//На всякий случай выключим счетчик
	SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);//Разрешим прерывания по таймеру
	SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk); //Выберем без делителя. будет 72 MHz
	MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, 71999 << SysTick_LOAD_RELOAD_Pos); //Настройка на 1 мс
	MODIFY_REG(SysTick->LOAD, SysTick_VAL_CURRENT_Msk, 71999 << SysTick_VAL_CURRENT_Pos); //Начнем считать с 71999 до 0
	SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //На всякий случай выключим счетчик
	
	

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  //SystemClock_Config();
	
	//Настройка микроконтроллера на 72 MHz
	SET_BIT(RCC->CR, RCC_CR_HSEON); //Запустим внутренний RC генератор на 8 МГц
	while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == RESET); //Дождемся поднятия флага о готовности
	
	SET_BIT(RCC->CR, RCC_CR_HSEON); //Запустим внешний кварцевый резонатор. Он у нас на 8 MHz
	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET) ; //Дождемся поднятия флага о готовности
	
	CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP); //Сбросим бит байпаса в 0
	
	SET_BIT(RCC->CR, RCC_CR_CSSON); //Запустим Clock detector
	
	//SET_BIT(RCC->CR, RCC_CR_PLLON); //Запустим PLL
	//while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET);//Ожидаем готовность включения PLL
	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL); //Выберем PLL в качестве System clock
	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL); //Используем PLL в качестве System_clock
	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1); //AHB Prescaler /1
	
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2); //010 Two wait states, if 48 MHz < SYSCLK <= 72 MHz
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE); //1: Prefetch is enabled
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBS); //1: Prefetch buffer is enabled
	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2); //APB1 Prescaler /2, т.к. PCLR 1 max 36 MHz
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1); //APB2 Prescaler /1
	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_DIV6); //72/6 = 12 Мгц. 14 максимум
	SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC); //В качестве входного сигнала для PLL выберем HSE
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLXTPRE); //Никакое предделение перед PLL нам не нужно. Поэтому /1
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMULL, RCC_CFGR_PLLMULL9); //т.к. кварц у нас на 8 MHz, а нам нужно 72, то нужно сделать умножение на 9. 8*9 = 72
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_USBPRE); // Настроили USB на 48 MHz
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, RCC_CFGR_MCO_PLLCLK_DIV2);
	
	SET_BIT(RCC->CR, RCC_CR_PLLON); //Запустим PLL
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET);//Ожидаем готовность включения PLL
	
	
	
	
	

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	//Настройка PC13
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN);//Запускаем тактирование порта С
	MODIFY_REG(GPIOC->CRH, GPIO_CRH_CNF13_Msk, 0b00 << GPIO_CRH_CNF13_Pos); //PC13 Output Push-Pull
	MODIFY_REG(GPIOC->CRH, GPIO_CRH_MODE13_Msk, 0b11 << GPIO_CRH_MODE13_Pos); //50 MHz
	
	//Настройка PA8
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Запускаем тактирование порта С
	MODIFY_REG(GPIOA->CRH, GPIO_CRH_CNF8_Msk, 0b10 << GPIO_CRH_CNF8_Pos); //PC13 Output Push-Pull
	MODIFY_REG(GPIOA->CRH, GPIO_CRH_MODE8_Msk, 0b11 << GPIO_CRH_MODE8_Pos); //50 MHz
	
	
  //MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	  //SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
	  //Delay_ms(100);
	  //HAL_Delay(100);
	  
	  //SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
	  //Delay_ms(100);
	  //HAL_Delay(1000);
	  
	  
	  if (SysTimer_ms - Time >= 10) {
		  flag = !flag;
		  Time = SysTimer_ms;
		  
		  if (flag) {
			  SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
		  } else {
			  SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
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
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

