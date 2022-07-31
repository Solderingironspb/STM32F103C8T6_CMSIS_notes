#include "main.h"

/*
 ***************************************************************************************
 *  @breif Настройка МК STM32F103C8T6 на частоту 72MHz от внешнего кварцевого резонатора
 *  Внешний кварцевый резонатор на 8 MHz
 *  ADC настроен на 12MHz
 *  USB настроен на 48MHz
 *  MCO подключен к PLL и тактируется от 36MHz
 *  Reference Manual/см. п.7.3 RCC registers (стр. 99)
 *  В настройке также необходимо настроить FLASH на работу, совместимую с 72MHz:
 *  Reference Manual/см. п.3.2 Memory organization (стр. 49)
 ***************************************************************************************
 */

void CMSIS_RCC_SystemClock_72MHz(void) {

	/* Начнем с п. 7.3.1 Clock control register (RCC_CR)*/
	SET_BIT(RCC->CR, RCC_CR_HSION); //Запустим внутренний RC генератор на 8 МГц
	while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0); //Дождемся поднятия флага о готовности
	CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP); //Просто сбросим этот бит в 0(Хотя изначально он и так должен быть в 0).
	SET_BIT(RCC->CR, RCC_CR_HSEON); //Запустим внешний кварцевый резонатор. Он у нас на 8 MHz.
	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0); //Дождемся поднятия флага о готовности
	SET_BIT(RCC->CR, RCC_CR_CSSON); //Включим CSS
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL); //Выберем PLL в качестве System Clock
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL); //Используем PLL в качестве system clock
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1); //APB Prescaler /1
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2); //010 Two wait states, if 48 MHz < SYSCLK <= 72 MHz
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE); //Prefetch is enabled(В Cube MX включено и я включил...)
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBS); //Prefetch buffer is enabled(В Cube MX включено и я включил...)
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2); //APB1 Prescaler /2, т.к. PCLK1 max 36MHz
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1); //APB2 Prescaler /1. Тут нас ничего не ограничивает. Будет 72MHz.
	MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_DIV6); //ADC Prescaler /6, чтоб было 12MHz, т.к. максимальная частота тут 14 MHz
	SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC); //В качестве входного сигнала для PLL выберем HSE
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLXTPRE_HSE); //Никакое предделение перед PLL нам не нужно. Поэтому /1.
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMULL, RCC_CFGR_PLLMULL9); //т.к. кварц у нас 8Mhz, а нам нужно 72MHz, то в PLL нужно сделать умножение на 9. 8MHz * 9 = 72MHz.
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_USBPRE); //Для USB 72MHz/1.5 = 48MHz
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, RCC_CFGR_MCO_PLLCLK_DIV2); //В качестве тактирования для MCO выбрал PLL. Будет 36 MHz.
	SET_BIT(RCC->CR, RCC_CR_PLLON); //Запустим PLL
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0); //Дожидемся поднятия флага включения PLL
}



/**
 ***************************************************************************************
 *  @breif Настройка SysTick на микросекунды
 *  На этом таймере мы настроим Delay и аналог HAL_GetTick()
 *  PM0056 STM32F10xxx/20xxx/21xxx/L1xxxx Cortex®-M3 programming manual/
 *  см. п.4.5 SysTick timer (STK) (стр. 150)
 ***************************************************************************************
 */
void CMSIS_SysTick_Timer_init(void) {
	/* п. 4.5.1 SysTick control and status register (STK_CTRL) (стр. 151)*/

	CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //Выключим таймер для проведения настроек.
	SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk); //Разрешим прерывания по таймеру
	SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk); //Выберем без делителя. Будет 72MHz
	MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, 71999 << SysTick_LOAD_RELOAD_Pos); //Настроим прерывание на частоту в 1 кГц(т.е. сработка будет каждую мс)
	MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, 71999 << SysTick_VAL_CURRENT_Pos); //Начнем считать с 71999
	SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //Запускаем таймер

}

/**
 ***************************************************************************************
 *  @breif Настройка Delay и аналог HAL_GetTick()
 ***************************************************************************************
 */

volatile uint32_t SysTimer_ms = 0; //Переменная, аналогичная HAL_GetTick()
volatile uint32_t Delay_counter_ms = 0; //Счетчик для функции Delay_ms


/**
 ******************************************************************************
 *  @breif Delay_ms
 *  @param   uint32_t Milliseconds - Длина задержки в миллисекундах
 ******************************************************************************
 */
void Delay_ms(uint32_t Milliseconds) {
	Delay_counter_ms = Milliseconds;
	while (Delay_counter_ms != 0);
}

/**
 ******************************************************************************
 *  @breif Прерывание по флагу COUNTFLAG (см. п. 4.5.1 SysTick control and status register (STK_CTRL))
 *  Список векторов(прерываний) можно найти в файле startup_stm32f103c8tx.S
 ******************************************************************************
 */
void SysTick_Handler(void) {

	SysTimer_ms++;

	if (Delay_counter_ms) {
		Delay_counter_ms--;
	}
}

/**
 ***************************************************************************************
 *  @breif Инициализация PIN PC13 на выход в режиме Push-Pull с максимальной скоростью 50 MHz
 *  Reference Manual/см. п.9.2 GPIO registers (стр. 171)
 *  Перед настройкой (GPIOs and AFIOs) нужно включить тактирование порта.
 ***************************************************************************************
 */
void CMSIS_PC13_OUTPUT_Push_Pull_init(void) {
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN); //Запуск тактирования порта C
	MODIFY_REG(GPIOC->CRH, GPIO_CRH_MODE13, 0b10 << GPIO_CRH_MODE13_Pos); //Настройка GPIOC порта 13 на выход со максимальной скоростью в 50 MHz
	MODIFY_REG(GPIOC->CRH, GPIO_CRH_CNF13, 0b00 << GPIO_CRH_CNF13_Pos); //Настройка GPIOC порта 13 на выход в режиме Push-Pull
}

/**
 ***************************************************************************************
 *  @breif Настройка MCO c выходом на ножку PA8
 *  Reference Manual/см. п.9.2 GPIO registers (стр. 171)
 *  Перед настройкой (GPIOs and AFIOs) нужно включить тактирование порта.
 ***************************************************************************************
 */
void CMSIS_PA8_MCO_init(void) {
	//Тактирование MCO должно быть настроено в регистре RCC
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, RCC_CFGR_MCO_PLLCLK_DIV2); //В качестве тактирования для MCO выбрал PLL. Будет 36 MHz.
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Запуск тактирования порта A
	MODIFY_REG(GPIOA->CRH, GPIO_CRH_MODE8, 0b11 << GPIO_CRH_MODE8_Pos); //Настройка GPIOA порта 8 на выход со максимальной скоростью в 50 MHz
	MODIFY_REG(GPIOA->CRH, GPIO_CRH_CNF8, 0b10 << GPIO_CRH_CNF8_Pos); //Настройка GPIOA порта 8, как альтернативная функция, в режиме Push-Pull
}
