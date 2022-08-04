/**
 ******************************************************************************
 *  @file stm32f103xx_CMSIS.c
 *  @brief CMSIS на примере МК STM32F103C8T6
 *  @author Волков Олег
 *  @date 17.07.2022
 *
  ******************************************************************************
 * @attention
 * 
 *  Библиотека помогает разобраться с библиотекой CMSIS на примере 
 *  МК STM32F103C8T6
 *  
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub: https://github.com/Solderingironspb/Lessons-Stm32/blob/master/README.md
 *  Группа ВК: https://vk.com/solderingiron.stm32
 *  Работал по Reference Manual: https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 *  
 ******************************************************************************
 */

#include "stm32f103xx_CMSIS.h"

/*================================= НАСТРОЙКА DEBUG ============================================*/
	
/**
***************************************************************************************
*  @breif Debug port mapping
*  Reference Manual/см. п.9.3.5 JTAG/SWD alternate function remapping (стр. 177)
*  Наверное это нужно настраивать в самом начале, еще до тактирования...
***************************************************************************************
*/

void CMSIS_Debug_init(void) { 
/**
*  Alternate function GPIO port
*  JTMS / SWDIO PA13
*  JTCK / SWCLK PA14
*  JTDI PA15
*  JTDO / TRACESWO PB3
*  NJTRST PB4
*/
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Включим тактирование порта A
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);//Включим тактирование альтернативных функций

/**
 *  Выберем режим отладки см.п 9.4.2 AF remap and debug I/O configuration register (AFIO_MAPR)(стр 184)
 */

/**
*  Bits 26:24 SWJ_CFG[2:0]: Serial wire JTAG configuration
*  These bits are write-only (when read, the value is undefined). They are used to configure the
*  SWJ and trace alternate function I/Os. The SWJ (Serial Wire JTAG) supports JTAG or SWD
*  access to the Cortex® debug port. The default state after reset is SWJ ON without trace.
*  This allows JTAG or SW mode to be enabled by sending a specific sequence on the JTMS / JTCK pin.
*  000: Full SWJ (JTAG-DP + SW-DP): Reset State          (JTAG 5 pins ) по-умолчанию
*  001: Full SWJ (JTAG-DP + SW-DP) but without NJTRST    (JTAG 4 pins)
*  010: JTAG-DP Disabled and SW-DP Enabled               (Serial wire)
*  100: JTAG-DP Disabled and SW-DP Disabled              (No Debug)
*  Other combinations: no effect
*/

	MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG, 0b010 << AFIO_MAPR_SWJ_CFG_Pos); //Serial wire

/**
*  При выборе Serial wire:
*  PA13 /JTMS/SWDIO 
*  PA14 /JTCK/SWCLK. 
*  PA15, PB3 и PB4 свободны
*/
	/*Заблокируем доступ для редактирования конфигурации PA13 и PA14*/
	GPIOA->LCKR = GPIO_LCKR_LCKK | GPIO_LCKR_LCK13 | GPIO_LCKR_LCK14;
	GPIOA->LCKR = GPIO_LCKR_LCK13 | GPIO_LCKR_LCK14;
	GPIOA->LCKR = GPIO_LCKR_LCKK | GPIO_LCKR_LCK13 | GPIO_LCKR_LCK14; 
	GPIOA->LCKR;
}


/*============================== НАСТРОЙКА RCC =======================================*/
/**
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

	/**
	*  Bit 0 HSION : Internal high - speed clock enable
	*  Set and cleared by software.
	*  Set by hardware to force the internal 8 MHz RC oscillator ON when leaving Stop or Standby
	*  mode or in case of failure of the external 4 - 16 MHz oscillator used directly or indirectly as
	*  system clock.This bit cannot be reset if the internal 8 MHz RC is used directly or indirectly
	*  as system clock or is selected to become the system clock.
	*  0 : internal 8 MHz RC oscillator OFF
	*  1 : internal 8 MHz RC oscillator ON
	*/
	
	SET_BIT(RCC->CR, RCC_CR_HSION); //Запустим внутренний RC генератор на 8 МГц
	
/** 
*  Bit 1 HSIRDY : Internal high - speed clock ready flag
*  Set by hardware to indicate that internal 8 MHz RC oscillator is stable.After the HSION bit is
*  cleared, HSIRDY goes low after 6 internal 8 MHz RC oscillator clock cycles.
*  0 : internal 8 MHz RC oscillator not ready
*  1 : internal 8 MHz RC oscillator ready
*/
	
	while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0); //Дождемся поднятия флага о готовности
	
/**
*  Bit 2 Reserved, must be kept at reset value.
*/

/**
*  Bits 7:3 HSITRIM[4:0]: Internal high-speed clock trimming
*  These bits provide an additional user-programmable trimming value that is added to the
*  HSICAL[7:0] bits. It can be programmed to adjust to variations in voltage and temperature
*  that influence the frequency of the internal HSI RC.
*  The default value is 16, which, when added to the HSICAL value, should trim the HSI to 8
*  MHz ± 1%. The trimming step (Fhsitrim) is around 40 kHz between two consecutive HSICAL steps.
*/
	//Тут оставим по-умолчанию

/**
*  Bits 15:8 HSICAL[7:0]: Internal high-speed clock calibration
*  These bits are initialized automatically at startup. 
*/
	//Тут тоже пусть будет все по-умолчанию

	//Далее чуть-чуть поменяем порядок выполнения команд, т.к. 18 бит должен быть определен до включения HSE(16-17 бит).

/**
*  Bit 18 HSEBYP: External high-speed clock bypass
*  Set and cleared by software to bypass the oscillator with an external clock. The external
*  clock must be enabled with the HSEON bit set, to be used by the device. The HSEBYP bit
*  can be written only if the HSE oscillator is disabled.
*  0: external 4-16 MHz oscillator not bypassed
*  1: external 4-16 MHz oscillator bypassed with external clock
*/
	
	CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP); //Просто сбросим этот бит в 0(Хотя изначально он и так должен быть в 0).
	
/**
*  Bit 16 HSEON: HSE clock enable
*  Set and cleared by software.
*  Cleared by hardware to stop the HSE oscillator when entering Stop or Standby mode. This
*  bit cannot be reset if the HSE oscillator is used directly or indirectly as the system clock.
*  0: HSE oscillator OFF
*  1: HSE oscillator ON 
*/

	SET_BIT(RCC->CR, RCC_CR_HSEON); //Запустим внешний кварцевый резонатор. Он у нас на 8 MHz.
	
/**
*  Bit 17 HSERDY: External high-speed clock ready flag
*  Set by hardware to indicate that the HSE oscillator is stable. This bit needs 6 cycles of the
*  HSE oscillator clock to fall down after HSEON reset.
*  0: HSE oscillator not ready
*  1: HSE oscillator ready 
*/

	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0); //Дождемся поднятия флага о готовности

/**
*  Bit 19 CSSON: Clock security system enable
*  Set and cleared by software to enable the clock security system. When CSSON is set, the
*  clock detector is enabled by hardware when the HSE oscillator is ready, and disabled by
*  hardware if a HSE clock failure is detected.
*  0: Clock detector OFF
*  1: Clock detector ON (Clock detector ON if the HSE oscillator is ready , OFF if not).
*/
	
	SET_BIT(RCC->CR, RCC_CR_CSSON); //Включим CSS
	
/**
*  Bits 23 : 20 Reserved, must be kept at reset value.
*/	

	
/**
*  Bit 24 PLLON: PLL enable
*  Set and cleared by software to enable PLL.
*  Cleared by hardware when entering Stop or Standby mode. This bit can not be reset if the
*  PLL clock is used as system clock or is selected to become the system clock.
*  0: PLL OFF
*  1: PLL ON
*/
	
	//SET_BIT(RCC->CR, RCC_CR_PLLON); //Запустим PLL, но чуточку позже, т.к. перед его включением нужно настроить другие регистры, иначе придется вкл/выкл постоянно.
	
/**
*  Bit 25 PLLRDY: PLL clock ready flag
*  Set by hardware to indicate that the PLL is locked.
*  0: PLL unlocked
*  1: PLL locked
*/
	
	//while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0) ; //Тут мы должны дожидаться поднятия флага включения PLL

/**
*  Bits 31:26 Reserved, must be kept at reset value.
*/

	// Переходим к следующему пункту 7.3.2 Clock configuration register (RCC_CFGR)
	//Не забываем, что PPL мы пока не включали, чтоб сделать настройки далее.
	
/**
*  Bits 1:0 SW: System clock switch
*  Set and cleared by software to select SYSCLK source.
*  Set by hardware to force HSI selection when leaving Stop and Standby mode or in case of
*  failure of the HSE oscillator used directly or indirectly as system clock (if the Clock Security System is enabled).
*  00: HSI selected as system clock
*  01: HSE selected as system clock
*  10: PLL selected as system clock
*  11: not allowed
*/	
	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL); //Выберем PLL в качестве System Clock

/**
*  Bits 3:2 SWS: System clock switch status
*  Set and cleared by hardware to indicate which clock source is used as system clock.
*  00: HSI oscillator used as system clock
*  01: HSE oscillator used as system clock
*  10: PLL used as system clock
*  11: not applicable 
*/
	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL); //Используем PLL в качестве system clock
	
/**
*  Bits 7:4 HPRE: AHB prescaler
*  Set and cleared by software to control the division factor of the AHB clock.
*  0xxx: SYSCLK not divided
*  1000: SYSCLK divided by 2
*  1001: SYSCLK divided by 4
*  1010: SYSCLK divided by 8
*  1011: SYSCLK divided by 16
*  1100: SYSCLK divided by 64
*  1101: SYSCLK divided by 128
*  1110: SYSCLK divided by 256
*  1111: SYSCLK divided by 512
*  Note: The prefetch buffer must be kept on when using a prescaler different from 1 on the
*  AHB clock. Refer to Reading the Flash memory section for more details.
*/	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1); //APB Prescaler /1
	//Вот тут в Note пишется отсылка к Flash(см. стр. 58)

	//Поэтому прервемся и настроим Flash (Flash access control register (FLASH_ACR))
	
/**
*  Bits 2:0 LATENCY: Latency
*  These bits represent the ratio of the SYSCLK (system clock) period to the Flash access time.
*  000 Zero wait state, if 0 < SYSCLK <= 24 MHz
*  001 One wait state, if 24 MHz < SYSCLK <= 48 MHz
*  010 Two wait states, if 48 MHz < SYSCLK <= 72 MHz
*/	
	
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2); //010 Two wait states, if 48 MHz < SYSCLK <= 72 MHz
	
/**
*  Bit 3 HLFCYA: Flash half cycle access enable
*  0: Half cycle is disabled
*  1: Half cycle is enabled
*/
	//Тут я пока не понял, поэтому не трогал
	
/**
*  Bit 4 PRFTBE: Prefetch buffer enable
*  0: Prefetch is disabled
*  1: Prefetch is enabled
*/
	
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE); //Prefetch is enabled

/**
*  Bit 5 PRFTBS: Prefetch buffer status
*  This bit provides the status of the prefetch buffer.
*  0: Prefetch buffer is disabled
*  1: Prefetch buffer is enabled
*/
	

/**
*  Bits 31:6 Reserved, must be kept at reset value
*/

	// Вот теперь можно вернуться обратно к пункту 7.3.2 Clock configuration register (RCC_CFGR) стр.103
	
/**
*  Bits 10:8 PPRE1: APB low-speed prescaler (APB1)
*  Set and cleared by software to control the division factor of the APB low-speed clock (PCLK1).
*  Warning: the software has to set correctly these bits to not exceed 36 MHz on this domain.
*  0xx: HCLK not divided
*  100: HCLK divided by 2
*  101: HCLK divided by 4
*  110: HCLK divided by 8
*  111: HCLK divided by 16
*/
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2); //APB1 Prescaler /2, т.к. PCLK1 max 36MHz
	
/**
*  Bits 13:11 PPRE2: APB high-speed prescaler (APB2)
*  Set and cleared by software to control the division factor of the APB high-speed clock (PCLK2).
*  0xx: HCLK not divided
*  100: HCLK divided by 2
*  101: HCLK divided by 4
*  110: HCLK divided by 8
*  111: HCLK divided by 16
*/	
	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1); //APB2 Prescaler /1. Тут нас ничего не ограничивает. Будет 72MHz.
	
/**
*  Bits 15:14 ADCPRE: ADC prescaler
*  Set and cleared by software to select the frequency of the clock to the ADCs.
*  00: PCLK2 divided by 2
*  01: PCLK2 divided by 4
*  10: PCLK2 divided by 6
*  11: PCLK2 divided by 8
*/	

	MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_DIV6); //ADC Prescaler /6, чтоб было 12MHz, т.к. максимальная частота тут 14 MHz
	
/**
*  Bit 16 PLLSRC: PLL entry clock source
*  Set and cleared by software to select PLL clock source. This bit can be written only when PLL is disabled.
*  0: HSI oscillator clock / 2 selected as PLL input clock
*  1: HSE oscillator clock selected as PLL input clock
*/
	
	SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC); //В качестве входного сигнала для PLL выберем HSE
	
/**
*  Bit 17 PLLXTPRE: HSE divider for PLL entry
*  Set and cleared by software to divide HSE before PLL entry. This bit can be written only when PLL is disabled.
*  0: HSE clock not divided
*  1: HSE clock divided by 2
*/	
	
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLXTPRE_HSE); //Никакое предделение перед PLL нам не нужно. Поэтому /1.
	
/**
*  Bits 21:18 PLLMUL: PLL multiplication factor
*  These bits are written by software to define the PLL multiplication factor. These bits can be
*  written only when PLL is disabled.
*  Caution: The PLL output frequency must not exceed 72 MHz.
*  0000: PLL input clock x 2
*  0001: PLL input clock x 3
*  0010: PLL input clock x 4
*  0011: PLL input clock x 5
*  0100: PLL input clock x 6
*  0101: PLL input clock x 7
*  0110: PLL input clock x 8
*  0111: PLL input clock x 9
*  1000: PLL input clock x 10
*  1001: PLL input clock x 11
*  1010: PLL input clock x 12
*  1011: PLL input clock x 13
*  1100: PLL input clock x 14
*  1101: PLL input clock x 15
*  1110: PLL input clock x 16
*  1111: PLL input clock x 16
*/
		
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMULL, RCC_CFGR_PLLMULL9); //т.к. кварц у нас 8Mhz, а нам нужно 72MHz, то в PLL нужно сделать умножение на 9. 8MHz * 9 = 72MHz.
	
/**
*  Bit 22 USBPRE: USB prescaler
*  Set and cleared by software to generate 48 MHz USB clock. This bit must be valid before
*  enabling the USB clock in the RCC_APB1ENR register. This bit can’t be reset if the USB clock is enabled.
*  0: PLL clock is divided by 1.5
*  1: PLL clock is not divided
*/

	CLEAR_BIT(RCC->CFGR, RCC_CFGR_USBPRE); //Для USB 72MHz/1.5 = 48MHz

/**
*  Bits 26:24 MCO: Microcontroller clock output
*  Set and cleared by software.
*  0xx: No clock
*  100: System clock (SYSCLK) selected
*  101: HSI clock selected
*  110: HSE clock selected
*  111: PLL clock divided by 2 selected
*  Note: This clock output may have some truncated cycles at startup or during MCO clock source switching.
*  When the System Clock is selected to output to the MCO pin, make sure that this clock does not exceed 50 MHz (the maximum IO speed). 
*/
	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, RCC_CFGR_MCO_PLLCLK_DIV2); //В качестве тактирования для MCO выбрал PLL. Будет 36 MHz.
	//Чтоб воспользоваться выводом MCO, нужно настроить ножку PA8 в альтернативную функцию на выход.

/**
*  Bits 31:27 Reserved, must be kept at reset value.
*/

	//И наконец, после всех настроек, мы можем запустить PLL
		
/**
*  Bit 24 PLLON: PLL enable
*  Set and cleared by software to enable PLL.
*  Cleared by hardware when entering Stop or Standby mode. This bit can not be reset if the
*  PLL clock is used as system clock or is selected to become the system clock.
*  0: PLL OFF
*  1: PLL ON
*/
	
	SET_BIT(RCC->CR, RCC_CR_PLLON); //Запустим PLL
	
/**
*  Bit 25 PLLRDY: PLL clock ready flag
*  Set by hardware to indicate that the PLL is locked.
*  0: PLL unlocked
*  1: PLL locked
*/
	
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0); //Дожидемся поднятия флага включения PLL
	
	//В итоге должно получится:
	//RCC->CR == 0x030B5A83
	//RCC->CFGR == 0x071D840A
	//К сожалению, нельзя просто так взять и сразу применить значения регистров и настроить все в 2 строчки кода, т.к. порядок выполнения команд играет очень большую роль.
}

/*========================= НАСТРОЙКА СИСТЕМНОГО ТАЙМЕРА ==============================*/

/**
*  P.S. С Системным таймером очень долго не мог понять, где брать информацию, т.к. в Reference Manual по нему ничего нет, а оказалось, что нужно 
*  было открыть документацию на МК (PM0056 STM32F10xxx/20xxx/21xxx/L1xxxx Cortex®-M3 programming manual)
*  Ссылка: https://www.st.com/resource/en/programming_manual/pm0056-stm32f10xxx20xxx21xxxl1xxxx-cortexm3-programming-manual-stmicroelectronics.pdf
*  Ищем п. 4.5 SysTick timer (STK) (стр. 150), где все хорошо и доходчиво пояснено
*/

/*The processor has a 24 - bit system timer, SysTick, that counts down from the reload value to
zero, reloads(wraps to) the value in the LOAD register on the next clock edge, then counts
down on subsequent clocks.
When the processor is halted for debugging the counter does not decrement.*/


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

/**
*  Bit 0 ENABLE : Counter enable
*  Enables the counter.When ENABLE is set to 1, the counter loads the RELOAD value from the
*  LOAD register and then counts down.On reaching 0, it sets the COUNTFLAG to 1 and
*  optionally asserts the SysTick depending on the value of TICKINT.It then loads the RELOAD
*  value again, and begins counting.
*  0 : Counter disabled
*  1 : Counter enabled
*/
	
	CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //Выключим таймер для проведения настроек.

/**
*  Bit 1 TICKINT: SysTick exception request enable
*  0: Counting down to zero does not assert the SysTick exception request
*  1: Counting down to zero to asserts the SysTick exception request.
*  Note: Software can use COUNTFLAG to determine if SysTick has ever counted to zero.
*/
	SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk); //Разрешим прерывания по таймеру
	//Прерывание будет происходить каждый раз, когда счетчик отсчитает от заданного значения до 0.
	
/**
*  Bit 2 CLKSOURCE : Clock source selection
*  Selects the clock source.
*  0 : AHB / 8
*  1 : Processor clock(AHB)
*/
	SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk); //Выберем без делителя. Будет 72MHz
	
/**
*  Bit 16 COUNTFLAG:
*  Returns 1 if timer counted to 0 since last time this was read.
*/
	
/**
*  Bits 15:3 Reserved, must be kept cleared.
*/
	
/*Следующий п. 4.5.2 SysTick reload value register (STK_LOAD) (Стр 152)*/
	//Помним, что таймер у нас все еще выключен.
	
/**
*  Bits 23 : 0 RELOAD[23 : 0] : RELOAD value
*  The LOAD register specifies the start value to load into the VAL register when the counter is
*  enabled and when it reaches 0.
*  Calculating the RELOAD value
*  The RELOAD value can be any value in the range 0x00000001 - 0x00FFFFFF. A start value of
*  0 is possible, but has no effect because the SysTick exception request and COUNTFLAG are
*  activated when counting from 1 to 0.
*  The RELOAD value is calculated according to its use :
*  l To generate a multi - shot timer with a period of N processor clock cycles, use a RELOAD
*  value of N - 1. For example, if the SysTick interrupt is required every 100 clock pulses, set
*  RELOAD to 99.
*  l To deliver a single SysTick interrupt after a delay of N processor clock cycles, use a
*  RELOAD of value N.For example, if a SysTick interrupt is required after 400 clock
*  pulses, set RELOAD to 400.
*/
	
	MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, 71999 << SysTick_LOAD_RELOAD_Pos); //Настроим прерывание на частоту в 1 кГц(т.е. сработка будет каждую мс)
	
/**
Bits 31:24 Reserved, must be kept cleared.
*/
	
/*Следующий п. 4.5.3 SysTick current value register (STK_VAL) (Стр. 153)*/
	
/**
*  CURRENT[23:0]: Current counter value
*  The VAL register contains the current value of the SysTick counter.
*  Reads return the current value of the SysTick counter.
*  A write of any value clears the field to 0, and also clears the COUNTFLAG bit in the
*  STK_CTRL register to 0
*/
	
	MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, 71999 << SysTick_VAL_CURRENT_Pos); //Начнем считать с 71999
	
/*Есть там еще регистр калибровки, но я его трогать не буду*/

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



/*============================== НАСТРОЙКА GPIO =======================================*/
/**
 ***************************************************************************************
 *  @breif Настройка GPIO
 *  Reference Manual/см. п.9.2 GPIO registers (стр. 171)  
 *  Перед настройкой (GPIOs and AFIOs) нужно включить тактирование порта.
 ***************************************************************************************
 */

/*Пример настройки портов*/

/*Включение или выключение тактирования порта*/
// SET_BIT(RCC->RCC_APB2ENR_AFIOEN, RCC_APB2ENR_IOPAEN); //Запуск тактирования порта A
//CLEAR_BIT(RCC->RCC_APB2ENR_AFIOEN, RCC_APB2ENR_IOPAEN); //Выключить тактирование порта A

// SET_BIT(RCC->RCC_APB2ENR_AFIOEN, RCC_APB2ENR_IOPBEN); //Запуск тактирования порта B
//CLEAR_BIT(RCC->RCC_APB2ENR_AFIOEN, RCC_APB2ENR_IOPBEN); //Выключить тактирование порта B

// SET_BIT(RCC->RCC_APB2ENR_AFIOEN, RCC_APB2ENR_IOPCEN); //Запуск тактирования порта C
//CLEAR_BIT(RCC->RCC_APB2ENR_AFIOEN, RCC_APB2ENR_IOPCEN); //Выключить тактирование порта C

// SET_BIT(RCC->RCC_APB2ENR_AFIOEN, RCC_APB2ENR_IOPDEN); //Запуск тактирования порта D
//CLEAR_BIT(RCC->RCC_APB2ENR_AFIOEN, RCC_APB2ENR_IOPDEN); //Выключить тактирование порта D

/**
*  Далее нас встречают 2 регистра: 
*  CRL и CRH (см. п 9.2.1 Port configuration register low (GPIOx_CRL) (x=A..G) (стр. 171)
*  и п. 9.2.2 Port configuration register high (GPIOx_CRH) (x=A..G) (стр. 172))
*  CRL предназначен для пина с 0 по 7
*  CRH предназначен для пина с 8 по 15
*/

/**
*  Bits 29 : 28, 25 : 24,
*  21 : 20, 17 : 16, 13 : 12,
*  9 : 8, 5 : 4, 1 : 0
*  MODEy[1 : 0] : Port x mode bits(y = 0 .. 7)
*  These bits are written by software to configure the corresponding I / O port.
*  Refer to Table 20 : Port bit configuration table.
*  00 : Input mode(reset state)
*  01 : Output mode, max speed 10 MHz.
*  10 : Output mode, max speed 2 MHz.
*  11 : Output mode, max speed 50 MHz
*/

//MODIFY_REG(GPIOC->CRH, GPIO_CRH_MODE13, 0b11 << GPIO_CRH_MODE13_Pos); //Настройка GPIOC порта 13 на выход со максимальной скоростью в 50 MHz

/**
*  Bits 31 : 30, 27 : 26,
*  23 : 22, 19 : 18, 15 : 14,
*  11 : 10, 7 : 6, 3 : 2
*  CNFy[1 : 0] : Port x configuration bits(y = 0 .. 7)
*  These bits are written by software to configure the corresponding I / O port.
*  Refer to Table 20 : Port bit configuration table.
*  In input mode(MODE[1 : 0] = 00) :
*  00 : Analog mode
*  01 : Floating input(reset state)
*  10 : Input with pull - up / pull - down
*  11 : Reserved
*  In output mode(MODE[1 : 0] > 00) :
*  00 : General purpose output push - pull
*  01 : General purpose output Open - drain
*  10 : Alternate function output Push - pull
*  11 : Alternate function output Open - drain
*/

//т.к. ножку мы настроили в Output, то мы можем выбрать:
//  In output mode(MODE[1 : 0] > 00) :
//  00 : General purpose output push - pull
//  01 : General purpose output Open - drain
//  10 : Alternate function output Push - pull
//  11 : Alternate function output Open - drain
 
//MODIFY_REG(GPIOC->CRH, GPIO_CRH_CNF13, 0b00 << GPIO_CRH_CNF13_Pos); //Настройка GPIOC порта 13 на выход в режиме Push-Pull

/*А теперь два примера*/

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

/*================================ РЕЖИМ EXTI =======================================*/

/**
 ***************************************************************************************
 *  @breif Режим EXTI.
 *  Reference Manual/см. п. 10 Interrupts and events (стр. 197)
 *  Тема прерываний очень важна, и читать про EXTI нужно именно с 10 пункта.		
 ***************************************************************************************
 */

//  10.1.1 SysTick calibration value register
//  The SysTick calibration value is set to 9000, which gives a reference time base of 1 ms with
//  the SysTick clock set to 9 MHz(max HCLK / 8).
//  Не совсем понял эту тему про 9 MHz, т.к. мы ее можем настроить на любую частоту. В пункте про настройку SysTimer, мы не использовали деление на 8.

//  В пункте 10.1.2 Interrupt and exception vectors можно просмотреть таблицу векторов(прерываний), которые мы можем использовать.
//  Таблицу векторов можно просмотреть в файле startup_stm32f103c8tx.S
//  См. Figure 21. External interrupt/event GPIO mapping (стр. 210)
//  у нас есть несколько EXTI 0-15.
//  К каждому подключены входы ножек.
//  Нельзя создать одно EXTI прерывание с нескольких ног одновременно.
//  Пример: EXTI0. Тогда мы должны выбирать, либо PA0, либо PB0, либо PC0 и т.д.
//  Под маппингом на стр. 210 есть сноска:

/**
*  1. To configure the AFIO_EXTICRx for the mapping of external interrupt/event lines onto GPIOs, the AFIO
*  clock should first be enabled. Refer to Section 7.3.7: APB2 peripheral clock enable register
*  (RCC_APB2ENR) for low-, medium-, high- and XL-density devices and, to Section 8.3.7: APB2 peripheral
*  clock enable register (RCC_APB2ENR) for connectivity line devices. 
*/

//  Это значит, что сначала, перед выбором ножки на EXTI вход, нужно включить тактирование AFIO.

void CMSIS_RCC_AFIO_enable(void) {
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN); //Включить тактирование для альтернативных функций
}
//  После этого, нам нужно настроить регистр AFIO->EXTICR1, чтоб выбрать ножку для создания прерывания.
//  Для примера, мы хотим использовать ножку PB0 для создания прерывания на EXTI0.

// см. п. 9.4.3 External interrupt configuration register 1 (AFIO_EXTICR1) (стр. 191)

/**
*  Bits 15:0 EXTIx[3:0]: EXTI x configuration (x= 0 to 3)
*  These bits are written by software to select the source input for EXTIx external interrupt.
*  Refer to Section 10.2.5: External interrupt/event line mapping
*  0000: PA[x] pin
*  0001: PB[x] pin
*  0010: PC[x] pin
*  0011: PD[x] pin
*  0100: PE[x] pin
*  0101: PF[x] pin
*  0110: PG[x] pin
*/

void CMSIS_AFIO_EXTICR1_B0_select(void) {
	MODIFY_REG(AFIO->EXTICR[0], AFIO_EXTICR1_EXTI0, AFIO_EXTICR1_EXTI0_PB); //AFIO_EXTICR1, EXTI0, выбран порт B.
}

/**
*  Bits 31:16 Reserved
*/

// Далее следует настроить ножку PB0 на вход. Можем сделать подтяжку.

void CMSIS_PB0_INPUT_Pull_Down_init(void) {
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN); //Включим тактирование порта B
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF0, 0b10 << GPIO_CRL_CNF0_Pos); //Настроим ножку PB0 в режим Input with pull-up / pull-down
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_MODE0, 0b00 << GPIO_CRL_MODE0_Pos); //Настройка в режим Input
	GPIOB->BSRR = GPIO_BSRR_BR0; //Подтяжка к земле
}

//  Теперь можем перейти к п. 10.3 (стр. 211)

/**
*  10.3 EXTI registers
*  Refer to Section 2.2 on page 45 for a list of abbreviations used in register descriptions.
*  The peripheral registers have to be accessed by words (32-bit).
*/

//  Здесь есть 2 регистра Interrupt mask register (EXTI_IMR) и Event mask register (EXTI_EMR)

void CMSIS_EXTI_0_init(void) {
	
/**
*  Bits 19:0 MRx: Interrupt Mask on line x
*  0: Interrupt request from Line x is masked
*  1: Interrupt request from Line x is not masked
*  Note: Bit 19 is used in connectivity line devices only and is reserved otherwise
*/
	
	SET_BIT(EXTI->IMR, EXTI_IMR_MR0);//Включаем прерывание EXTI0 по входному сигналу
	
/**
*  Bits 31:20 Reserved, must be kept at reset value (0).
*/

	/*То же самое и для EMR, только оно нам не нужно пока*/
	//CLEAR_BIT(EXTI->EMR, EXTI_EMR_EM0);//Событие нам не нужно, поэтому поставим сюда 0.

/*Реагирование по фронту и спаду сигнала см. п. 10.3.3 Rising trigger selection register (EXTI_RTSR) (Стр. 212)*/
	
/**
*  Bits 19:0 TRx: Rising trigger event configuration bit of line x
*  0: Rising trigger disabled (for Event and Interrupt) for input line
*  1: Rising trigger enabled (for Event and Interrupt) for input line.
*  Note: Bit 19 is used in connectivity line devices only and is reserved otherwise
*  
*  Note: The external wakeup lines are edge triggered, no glitches must be generated on these lines.
*  If a rising edge on external interrupt line occurs during writing of EXTI_RTSR register, the
*  pending bit will not be set.
*  Rising and Falling edge triggers can be set for the same interrupt line. In this configuration,
*  both generate a trigger condition.
*/
	SET_BIT(EXTI->RTSR, EXTI_RTSR_TR0);//Реагирование по фронту вкл.
	//SET_BIT(EXTI->FTSR, EXTI_FTSR_TR0);//Реагирование по спаду вкл.
	
/**
*  Bits 31:20 Reserved, must be kept at reset value (0)
*/

/**
*   10.3.5 Software interrupt event register (EXTI_SWIER)
*   Используется для софтварного запуска прерывания
*/

/**
*  Bits 19:0 SWIERx: Software interrupt on line x
*  If the interrupt is enabled on this line in the EXTI_IMR, writing a '1' to this bit when it is set to
*  '0' sets the corresponding pending bit in EXTI_PR resulting in an interrupt request generation.
*  This bit is cleared by clearing the corresponding bit of EXTI_PR (by writing a 1 into the bit).
*  Note: Bit 19 used in connectivity line devices and is reserved otherwise
*/
	//SET_BIT(EXTI->SWIER, EXTI_SWIER_SWIER0);//Это софтварное включение прерывания
	
/*
*  Bits 31:20 Reserved, must be kept at reset value (0)
*/

/** 
*  п. 10.3.6 Pending register (EXTI_PR)
*  Нужен для выхода из прерывания. Вставляется в Handler. 
*  Если не сбросить, то так в прерывании и зависнем.
*/	
	
/**
*  Bits 19:0 PRx: Pending bit
*  0: No trigger request occurred
*  1: selected trigger request occurred
*  This bit is set when the selected edge event arrives on the external interrupt line. This bit is
*  cleared by writing a ‘1’ into the bit.
*  Note: Bit 19 is used in connectivity line devices only and is reserved otherwise.
*/
	
	// SET_BIT(EXTI->PR, EXTI_PR_PR0); //Команда выхода из прерывания

/**
*  Bits 31:20 Reserved, must be kept at reset value (0)
*/
	
	NVIC_EnableIRQ(EXTI0_IRQn); //Включим прерывание по вектору EXTI0
}

__WEAK void EXTI0_IRQHandler(void) {

	SET_BIT(EXTI->PR, EXTI_PR_PR0); //Выйдем из прерывания

}

/*================================ Таймеры на примере TIM3 =======================================*/

/**
 ***************************************************************************************
 *  @breif General-purpose timers (TIM2 to TIM5)
 *  Reference Manual/см. п. 15 General-purpose timers (TIM2 to TIM5) (стр. 365)
 *  Рассмотрим все регистры, разберемся куда смотреть и прочее	
 ***************************************************************************************
 */
/**
*  Счетчик 16 битный. PSC тоже 16 бит.
*  The time - base unit includes :
*  Counter register(TIMx_CNT) - регистр счетчика
*  Prescaler register(TIMx_PSC) - предделитель частоты
*  Auto - Reload register(TIMx_ARR) - Регистр автоматической перезагрузки
*
*  Счетчик тактируется выходом предделителя CK_CNT, который активируется только тогда, когда бит
*  разрешения счетчика CEN, находищийся в регистре TIMx_CR1 установлен в 1.
*  Важно: Фактический сигнал включения счетчика CNT_EN устанавливается через 1 такт после включения бита CEN
*  В TIMx->PSC можно устанавливать любой коэффициент от 0 до 65535. 
*  Его можно изменять налету, т.к. он буферизирован.
*/
/**
*  Counter modes
*  Upcounting mode
*  В режиме Up, счетчик считает от 0 до значения автоперезагрузки (TIMx->ARR), затем перезапускается с 0
*  и генерирует событие переполнения счетчика.
*  Событие Update может генерироваться при каждом переполнении счетчика, либо
*  при установке бита UG в регистре TIMx->EGR(программно или в slave mode)
*
*  Событие UEV можно отключить программно, если установить бить UDIS в TIMx->CR1.
*  Тогда можно избежать обновления теневых регистров при записи новых значений в регистры
*  предварительной загрузки. Событие Update не будет происходить, пока в UDIS не будет записан 0.
*  Однако счетчик как перезапускался через 0, так и будет перезапускаться при переполнении, но скорость меняться
*  не будет. 
*  Кроме того, если установить бит URS (выбор запроса на обновление) в TIMx->CR1,
*  то установка бита UG генерирует событие update UEV, но без установки флага UIF(Таким образом 
*  не отправляется запрос interrupt или DMA). Это делается для того, чтоб избежать генерации прерываний
*  update, так и прерывания захвата при очистке счетчика события захвата.
*
*  Когда происходит Update event, обновляются все регистры и устанавливается флаг
*  UIF в регистре TIMx->SR
*  Вместе с этим:
*  - буфер TIMx->PSC перезагружается новым значением
*  - теневой регистр автоматической перезагрузки обновляется значением из TIMx->ARR

*  ARPE(auto-reload preload) в TIMx->CR1 отвечает за буферизацию TIMx->ARR
*  т.е. если в TIMx->CR1 бит ARPE будет 1, то если изменить TIMx->ARR, то возникнет тут же Update Event
*  и значение TIMx->ARR применится, не дожидаясь завершения счетчика до переполнения.
*/

/**
*  Center-aligned mode (up/down counting)
*  В режиме с выравнивание по центру, счетчик считает от 0 до значения автоперезагрузки(TIMx->ARR),
*  генерирует событие переполнения счетчика, а затем ведет обратный отсчет до 1, генерирует 
*  событие опустошения счетчика, а затем перезапускает счетчик с 0.
*
*  Center-aligned mode включен, когда биты CMS в регистре TIMx->CR1 не равны 0.
*  01 - Флаг прерывания возникает на счетчике Down
*  02 - Флаг прерывания возникает на счетчика Up
*  03 - Флаг прерывания возникает и на Down, и на Up
*  Примечание: Нельзя из Edge режима перейти в Center-aligned mode, если счетчик запущен.
*  Так же в этом режиме бит направления DIR из регистра TIMx->CR1 не может меняться.
*  Он обновляется аппаратно и выдает текущее направление счетчика.

*/
