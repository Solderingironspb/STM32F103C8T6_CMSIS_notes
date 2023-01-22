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
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN); //Включим тактирование альтернативных функций

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
	
	while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0) ; //Дождемся поднятия флага о готовности
	
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

	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0) ; //Дождемся поднятия флага о готовности

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
	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSE); //Выберем HSE в качестве System Clock(PLL лучше пока не выбирать, он у нас отключен)
	//p.s. Спасибо KARMA Electronics за подсказку.

	/**
	*  Bits 3:2 SWS: System clock switch status
	*  Set and cleared by hardware to indicate which clock source is used as system clock.
	*  00: HSI oscillator used as system clock
	*  01: HSE oscillator used as system clock
	*  10: PLL used as system clock
	*  11: not applicable 
	*/
	
		//Это статус
	
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
	
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, 0b010 << FLASH_ACR_LATENCY_Pos); //010 Two wait states, if 48 MHz < SYSCLK <= 72 MHz
	
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
	
	//Т.к. PLL уже запущен, выберем его в качестве System Clock:
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL); //Выберем PLL в качестве System Clock
	
	/**
	*  Bit 25 PLLRDY: PLL clock ready flag
	*  Set by hardware to indicate that the PLL is locked.
	*  0: PLL unlocked
	*  1: PLL locked
	*/
	
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0) ; //Дожидемся поднятия флага включения PLL
	
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
volatile uint32_t Timeout_counter_ms = 0; //Переменная для таймаута функций

/**
 ******************************************************************************
 *  @breif Delay_ms
 *  @param   uint32_t Milliseconds - Длина задержки в миллисекундах
 ******************************************************************************
 */
void Delay_ms(uint32_t Milliseconds) {
	Delay_counter_ms = Milliseconds;
	while (Delay_counter_ms != 0) ;
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
	if (Timeout_counter_ms) {
		Timeout_counter_ms--;
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
 *  @breif Blink PIN PC13 на выход в режиме Push-Pull
 ***************************************************************************************
 */
void CMSIS_Blink_PC13(uint32_t ms) {
	GPIOC->BSRR = GPIO_BSRR_BR13;
	Delay_ms(ms);
	GPIOC->BSRR = GPIO_BSRR_BS13;
	Delay_ms(ms);
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
	
	SET_BIT(EXTI->IMR, EXTI_IMR_MR0); //Включаем прерывание EXTI0 по входному сигналу
	
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
	SET_BIT(EXTI->RTSR, EXTI_RTSR_TR0); //Реагирование по фронту вкл.
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

void CMSIS_TIM3_init(void) {
	/*Включим тактирование таймера (страница 48)*/
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN); //Запуск тактирования таймера 3
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN); //Запуск тактирования альтернативных функций
	
	/*Настройка таймера 3 (Страница 404)*/
	//15.4.1 TIMx control register 1 (TIMx_CR1)
	
	//SET_BIT(TIM3->CR1, TIM_CR1_CEN);  //Запуск таймера
	CLEAR_BIT(TIM3->CR1, TIM_CR1_UDIS); //Генерировать событие Update
	CLEAR_BIT(TIM3->CR1, TIM_CR1_URS); //Генерировать прерывание
	CLEAR_BIT(TIM3->CR1, TIM_CR1_OPM); //One pulse mode off(Счетчик не останавливается при обновлении)
	CLEAR_BIT(TIM3->CR1, TIM_CR1_DIR); //Считаем вверх
	MODIFY_REG(TIM3->CR1, TIM_CR1_CMS_Msk, 0b00 << TIM_CR1_CMS_Pos); //Выравнивание по краю
	SET_BIT(TIM3->CR1, TIM_CR1_ARPE); //Auto-reload preload enable
	MODIFY_REG(TIM3->CR1, TIM_CR1_CKD_Msk, 0b00 << TIM_CR1_CKD_Pos); //Предделение выключено
	
	/*Настройка прерываний (Страница 409)*/
	//15.4.4 TIMx DMA/Interrupt enable register (TIMx_DIER)
	SET_BIT(TIM3->DIER, TIM_DIER_UIE); //Update interrupt enable
	
	//15.4.5 TIMx status register (TIMx_SR) - Статусные регистры
	
	TIM3->PSC = 7200 - 1;
	TIM3->ARR = 10 - 1;
	
	NVIC_EnableIRQ(TIM3_IRQn); //Разрешить прерывания по таймеру 3
	SET_BIT(TIM3->CR1, TIM_CR1_CEN); //Запуск таймера
}

void CMSIS_TIM3_PWM_CHANNEL1_init(void) {
	/*Настройка ножки PA6 под ШИМ*/
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Включим тактирование порта А
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF6_Msk, 0b10 << GPIO_CRL_CNF6_Pos);
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE6_Msk, 0b11 << GPIO_CRL_MODE6_Pos);
	
	/*Настройка шим(Канал 1)*/
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_CC1S_Msk, 0b00 << TIM_CCMR1_CC1S_Pos); //CC1 channel is configured as output
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1FE); //Fast mode disable
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1PE); //Preload enable
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_OC1M_Msk, 0b110 << TIM_CCMR1_OC1M_Pos); //PWM MODE 1
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1CE); //OC1Ref is not affected by the ETRF input
	
	/*Запуск ШИМ*/
	//15.4.9 TIMx capture/compare enable register (TIMx_CCER)
	SET_BIT(TIM3->CCER, TIM_CCER_CC1E); //On - OC1 signal is output on the corresponding output pin. 
	SET_BIT(TIM3->CCER, TIM_CCER_CC1P); //OC1 active high.
	
	TIM3->CCR1 = 5;
}

void CMSIS_TIM3_PWM_CHANNEL2_init(void) {
	/*Настройка ножки PA7 под ШИМ*/
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Включим тактирование порта А
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF7_Msk, 0b10 << GPIO_CRL_CNF7_Pos);
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE7_Msk, 0b11 << GPIO_CRL_MODE7_Pos);
		
	/*Настройка шим(Канал 2)*/
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_CC2S_Msk, 0b00 << TIM_CCMR1_CC2S_Pos); //CC1 channel is configured as output
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC2FE); //Fast mode disable
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC2PE); //Preload enable
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_OC2M_Msk, 0b110 << TIM_CCMR1_OC2M_Pos); //PWM MODE 1
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC2CE); //OC1Ref is not affected by the ETRF input
	
	/*Запуск ШИМ*/
	//15.4.9 TIMx capture/compare enable register (TIMx_CCER)
	SET_BIT(TIM3->CCER, TIM_CCER_CC2E); //On - OC1 signal is output on the corresponding output pin. 
	CLEAR_BIT(TIM3->CCER, TIM_CCER_CC2P); //OC1 active high.
	
	TIM3->CCR2 = 5;
}

__WEAK void TIM3_IRQHandler(void) {
	if (READ_BIT(TIM3->SR, TIM_SR_UIF)) {
		CLEAR_BIT(TIM3->SR, TIM_SR_UIF); //Сбросим флаг прерывания
	}
}

/*================================= НАСТРОЙКА ADC ============================================*/
	
/**
***************************************************************************************
*  @breif Analog-to-digital converter (ADC)
*  Reference Manual/см. п.11 Analog-to-digital converter (ADC) (стр. 215)
***************************************************************************************
*/
	//STM32 имеет 12 разрядный АЦП последовательного приближения.
	//Он имеет до 18 мультиплексированных каналов, позволяющих измерять сигналы от шестнадцати внешних и двух
	//внутренних источников.
	//Аналого-цифровое преобразование различных каналов может выполняться в одиночном, непрерывном или прерывистом режиме.
	//Результат АЦП сохраняется в 16-разрядном регистре данных с выравниванием по левому или правому краю.
	
	//Входной тактовый сигнал АЦП генерируется из тактового сигнала PCLK2, деленного на prescaler.
	//Максимальная частота не должна превышать 14 МГц.
	
/*                                   Table 65. ADC pins                                                */
/*
     Name     |        Signal type                | Remarks
     VREF+    | Input, analog reference positive  | The higher/positive reference voltage for the ADC,
              |                                   |          2.4 V <= VREF+ <= VDDA
    VDDA(1)   |       Input, analog supply        | Analog power supply equal to VDD and
              |                                   |          2.4 V <= VDDA <= 3.6 V
     VREF-    | Input, analog reference negative  | The lower/negative reference voltage for the ADC,
              |                                   |              VREF- = VSSA
    VSSA(1)   |    Input, analog supply ground    |    Ground for analog power supply equal to VSS
ADCx_IN[15:0] |         Analog signals            |           Up to 21 analog channels(2)
*/

    /*11.3.1 ADC on-off control(стр. 218)*/
	//АЦП можно включить, установив бит ADON в регистре ADC_CR2. 
	//Когда бит ADON устанавливается в первый раз, он пробуждает АЦП из режима Power Down.
	//Преобразование начинается, когда бит ADON устанавливается программой во второй раз по истечении времени включения АЦП(tSTAB).
	//Преобразование можно остановить, а АЦП перевести в режим пониженного энергопотребления, сбросив бит ADON.
	//В этом режиме АЦП почти не потребляет энергии (всего несколько мкА).	
	
	/*11.3.3 Channel selection(стр. 218)*/
	//Имеется 16 мультиплексированных каналов. Конверсии можно разделить на две группы:
	//регулярные и инжектированные. Группа состоит из последовательности преобразований, которые можно
	//выполнять на любом канале и в любом порядке. Например можно выполнить преобразование в следующем порядке:
	//Ch3, Ch8, Ch2, Ch2, Ch0, Ch2, Ch2, Ch15.
	/*
	 * Регулярная группа состоит из 16 преобразований. Регулярные каналы и порядок последовательности
	 * преобразований должен быть настроен в регистрах ADC_SQRx.
	 * Общее количество преобразований в регулярной группе должно быть записано в битах L[3:0] в регистре ADC_SQR1
	 * 
	 * Инжектированная группа состоит из 4 преобразований. Вводимые канали и их порядок последовательности преобразований
	 * настраиваются в регистре ADC_ISQR. Общее количество преобразований в инжектируемой группе должно быть записано в битах 
	 * L[1:0] в регистре ADC_ISQR.
	 * 
	 * Если регистры ADC_SQRx или ADC_ISQR изменяются во время преобразования, то текущее преобразование сбрасывается и на АЦП
	 * отправляется новый стартовый импульс для преобразования новой выбранной группы.
	 * */

	/*Temperature sensor/VREFINT internal channels*/
	/* Датчик температуры подключен к каналу ADCx_IN16, а внутреннее опорное напряжение VREFint подключено к ADCx_IN17.
	 * Эти два внутренних канала можно выбрать и преобразовать через инжектированные или регулярные каналы.*/
	
	/*11.4 Calibration*/
	/* АЦП имеет встроенный режим самокалибровки. Калибровка значительно снижает ошибки точности из-за внутренних
	 * изменений внутреннего конденсатора. Во время калибровки для каждого конденсатора вычисляется код исправления ошибок,
	 * и во время всех последующих преобразований эта ошибка устраняется.
	 * 
	 * Калибровка запускается установкой бита CAL в регистре ADC_CR2.
	 * 
	 * После завершения калибровки, бит CAL сбрасывается аппаратно, и можно выполнить обычное преобразование.
	 * Рекомендуется калибровать АЦП один раз при включении питания. Коды калибровки сохраняются в ADC_DR, как только фаза 
	 * калибровки заканчивается.
	 * 
	 * Важно: перед началом калибровки АЦП должен находиться во включенном состоянии(бит ADON = 1) не менее двух тактов тактовой частоты АЦП.
	 * */
	
	/*11.8 DMA request(стр. 227)*/
	/* Поскольку преобразованные значения регулярных каналов хранятся в уникальном регистре данных, то необходимо
	 * использовать DMA для преобразования более чем одного обычного канала. Это позволяет избежать потери данных,
	 * уже сохраненных в регистре ADC_DR.
	 * 
	 * Только окончание преобразования обычного канала генерирует запрос DMA, который разрешает передачу его преобразованных данных 
	 * из регистра ADC_DR в место назначения, выбранное пользователем.*/
	
	/*11.11 ADC interrupts(стр. 236)*/
	/*
	 * Прерывание может быть создано в конце преобразования для регулярных и инжектированных групп, а также при установке
	 * бита состояния аналогового сторожевого устройства.
	 * 
	 * Прерывания ADC1 и ADC2 отображаются на один и тот же вектор прерывания. Прерывания ADC3 отображаются на отдельный
	 * вектор прерывания.
	 * */
	
	/* 
	 * Два других флага присутствуют в регистре ADC_SR, но с ними не связано прерывание:
	 * - JSTRT (Начало преобразования для инжектированных групповых каналов)
	 * - STRT (Начало преобразования для регулярных групповых каналов)
	 * */
	
	/*Table 71. ADC interrupts*/
/*
 *          Interrupt event          |         Event flag             |        Enable Control bit
 *   End of conversion regular group |             EOC                |              EOCIE           
 *  End of conversion injected group |             JEOC               |             JEOCIE
 * Analog watchdog status bit is set |             AWD                |              AWDIE
 */	

volatile uint16_t ADC_RAW_Data[2] = { 0, }; //Массив, куда будем кидать данные с АЦП


void CMSIS_ADC_DMA_init(void) {
	/* Настройка DMA
	*  Внимание:
	*  Порядок настройки DMA хорошо описан на странице 278 "Channel configuration procedure"*/
	 
	SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN); //Включение тактирования DMA1
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR); //Задаем адрес периферийного устройства
	DMA1_Channel1->CMAR = (uint32_t)ADC_RAW_Data; //Задаем адрес в памяти, куда будем кидать данные.
	DMA1_Channel1->CNDTR = 2; //Настроим количество данных для передачи. После каждого периферийного события это значение будет уменьшаться.
	MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_PL_Msk, 0b00 << DMA_CCR_PL_Pos); //Зададим приоритет канала на высокий
	CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_DIR); //Чтение будем осуществлять с периферии
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_CIRC); //Настроим DMA в Circular mode
	MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_PSIZE_Msk, 0b01 << DMA_CCR_PSIZE_Pos); //Размер данных периферийного устройства 16 бит
	MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_MSIZE_Msk, 0b01 << DMA_CCR_MSIZE_Pos); //Размер данных в памяти 16 бит
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_TCIE); //Включим прерывание по полной передаче
	CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_HTIE); //Отключим прерывание по половинной передаче
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_TEIE); //Включим прерывание по ошибке передачи.
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_MINC); //Включим инкрементирование памяти
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_EN); //DMA ON
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN); //Включение тактирования ADC1.
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Включение тактирования порта А.
	
	/*Настройка ножек PA0 и PA1 на аналоговый вход*/
	/*Pin PA0 - Analog*/
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF0_Msk, 0b00 << GPIO_CRL_CNF0_Pos);
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE0_Msk, 0b00 << GPIO_CRL_MODE0_Pos);
	
	/*Pin PA1 - Analog*/
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF1_Msk, 0b00 << GPIO_CRL_CNF1_Pos);
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE1_Msk, 0b00 << GPIO_CRL_MODE1_Pos);
	
	
	/*11.12 ADC registers(страница 237)*/
	//11.12.2 ADC control register 1 (ADC_CR1)(страница 238)
	
	//Прерывание по АЦП: регулярные каналы (вкл/выкл)
	CLEAR_BIT(ADC1->CR1, ADC_CR1_EOCIE); //EOC interrupt enabled/disabled. An interrupt is generated when the EOC bit is set
	
	//Прерывание по АЦП: analog watchdog (вкл/выкл)
	CLEAR_BIT(ADC1->CR1, ADC_CR1_AWDIE); //Analog watchdog interrupt disabled
	
	//Прерывание по АЦП: инжектированные каналы (вкл/выкл)
	CLEAR_BIT(ADC1->CR1, ADC_CR1_JEOCIE); //JEOC interrupt disabled
	
	SET_BIT(ADC1->CR1, ADC_CR1_SCAN); //Scan mode enabled
	
	/* Примечание:
	* Прерывание EOC или JEOC генерируется только в конце преобразования последнего канала, 
	* если установлен соответствующий бит EOCIE или JEOCIE.*/
	
	CLEAR_BIT(ADC1->CR1, ADC_CR1_AWDSGL); //Analog watchdog enabled on all channels
	CLEAR_BIT(ADC1->CR1, ADC_CR1_JAUTO); //Automatic injected group conversion disabled
	CLEAR_BIT(ADC1->CR1, ADC_CR1_DISCEN); //Discontinuous mode on regular channels disabled
	CLEAR_BIT(ADC1->CR1, ADC_CR1_JDISCEN); //Discontinuous mode on injected channels disabled
	MODIFY_REG(ADC1->CR1, ADC_CR1_DUALMOD_Msk, 0b0110 << ADC_CR1_DUALMOD_Pos); //0110: Regular simultaneous mode only
	CLEAR_BIT(ADC1->CR1, ADC_CR1_JAWDEN); //Analog watchdog disabled on injected channels
	CLEAR_BIT(ADC1->CR1, ADC_CR1_AWDEN); //Analog watchdog disabled on regular channels
	
	/*11.12.3 ADC control register 2 (ADC_CR2)(страница 240)*/
	
	SET_BIT(ADC1->CR2, ADC_CR2_ADON); //Запустить АЦП
	
	/* Примечание:
	* Если в этот же момент изменяется какой-либо другой бит в этом регистре, 
	* кроме ADON, то конверсия не запускается. 
	* Это сделано для предотвращения ошибочного преобразования.*/
	
	SET_BIT(ADC1->CR2, ADC_CR2_CONT); //Continuous conversion mode(непрерывные преобразования) 
	
	SET_BIT(ADC1->CR2, ADC_CR2_CAL); //Enable calibration
    
	/*Примечание:
	 * Этот бит устанавливается программой для запуска калибровки. 
	 * Он сбрасывается аппаратно после завершения калибровки.*/
	
	while (READ_BIT(ADC1->CR2, ADC_CR2_CAL)) ;//Подождем окончания калибровки
	Delay_ms(1); //Задержка для GD32F103CBT6. На STM32F103CBT6 работает и так. 
	
	SET_BIT(ADC1->CR2, ADC_CR2_DMA); //DMA включен
	CLEAR_BIT(ADC1->CR2, ADC_CR2_ALIGN); //Выравнивание по правому краю
	MODIFY_REG(ADC1->CR2, ADC_CR2_EXTSEL_Msk, 0b111 << ADC_CR2_EXTSEL_Pos); //Запускать преобразование программно
	CLEAR_BIT(ADC1->CR2, ADC_CR2_EXTTRIG); //Conversion on external event disabled
	//SET_BIT(ADC1->CR2, ADC_CR2_SWSTART); //Начать преобразование
	SET_BIT(ADC1->CR2, ADC_CR2_TSVREFE); //Temperature sensor and VREFINT channel enabled


	/*Note: 
	 * ADC1 analog Channel16 and Channel 17 are internally connected to the temperature
	 * sensor and to VREFINT, respectively.
	 * ADC2 analog input Channel16 and Channel17 are internally connected to VSS.
	 * ADC3 analog inputs Channel14, Channel15, Channel16 and Channel17 are connected to VSS.*/
	
	// 11.12.5 ADC sample time register 2 (ADC_SMPR2)(страница 245)
	MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP0_Msk, 0b111 << ADC_SMPR2_SMP0_Pos); //239.5 cycles 
	MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP1_Msk, 0b111 << ADC_SMPR2_SMP1_Pos); //239.5 cycles 
	MODIFY_REG(ADC1->SMPR1, ADC_SMPR1_SMP17_Msk, 0b111 << ADC_SMPR1_SMP17_Pos); //239.5 cycles 
	
	// 11.12.9 ADC regular sequence register 1 (ADC_SQR1)(страница 247)
	MODIFY_REG(ADC1->SQR1, ADC_SQR1_L_Msk, 0b0001 << ADC_SQR1_L_Pos); //2 преобразования
	
	// 11.12.11 ADC regular sequence register 3 (ADC_SQR3)(страница 249)
	MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ1_Msk, 0 << ADC_SQR3_SQ1_Pos);
	MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ2_Msk, 17 << ADC_SQR3_SQ2_Pos);
	//NVIC_EnableIRQ(ADC1_IRQn); //Разрешить прерывания по АЦП
	
	//SET_BIT(ADC1->CR2, ADC_CR2_SWSTART); //Начать преобразование. Не нужно запускать, если Continuous conversion mode(непрерывные преобразования) включен
}

__WEAK void ADC1_2_IRQHandler(void) {
	/*This bit is set by hardware at the end of a group channel conversion (regular or injected). It is
    * cleared by software or by reading the ADC_DR.
    * 0: Conversion is not complete
    * 1: Conversion complete*/
	if (READ_BIT(ADC1->SR, ADC_SR_EOC)) {
		ADC1->DR; //Читаем канал, чтоб сбросить флаг
	}
	
}
__WEAK void DMA1_Channel1_IRQHandler(void) {
	if (READ_BIT(DMA1->ISR, DMA_ISR_TCIF1)) {
		SET_BIT(DMA1->IFCR, DMA_IFCR_CGIF1); //Сбросим глобальный флаг.
		/*Здесь можно писать код*/
		
	} else if (READ_BIT(DMA1->ISR, DMA_ISR_TEIF1)) {
		/*Здесь можно сделать какой-то обработчик ошибок*/
		SET_BIT(DMA1->IFCR, DMA_IFCR_CGIF1); //Сбросим глобальный флаг.
	}
}

/*================================= НАСТРОЙКА USART ============================================*/
	
/**
***************************************************************************************
*  @breif Universal synchronous asynchronous receiver transmitter (USART)
*  Reference Manual/см. п.27 Universal synchronous asynchronous receiver transmitter (USART) (стр. 785)
***************************************************************************************
*/

/*Универсальный синхронный/асинхронный приемопередатчик USART дает гибкую возможность 
полнодюплексного обмена данными с внешним оборудованием, требующим промышленный стандарты NRZ.
Код NRZ (Non Return to Zero – без возврата к нулю) – это простейший код, представляющий собой
обычный цифровой сигнал. USART дает очень широкий диапазон скоростей передачи данных, используя
baud rate generator.

Он поддерживает синхронную одностороннюю связь и полудуплексную однопроводную связь, спецификации LIN,
протокол смарт-карт, спецификации SIR ENDEC и IrDA(передача по ИК каналу, а также операции модема (CTS/RTS)).
Возможна высокоскоростная передача данных при использовании DMA для конфигурации с несколькими буферами.*/

/*Основные возможности USART*/
/*
 * -  Full duplex, asynchronous communications
 * -  NRZ стандарт (Mark/Space)
 * -  Настраиваемая скорость(baud rate generator)
 *    Общая программируемая скорость передачи и приема до 4.5 МБит/с
 * -  Программируемая длина слова данных (8 и 9 бит)
 * -  Настраиваемые стоповые биты. Поддержка 1 или 2 стоповых битов.
 * -  Возможность отправки синхронного прерывания ведущего LIN и возможность обнаружения разрыва связи с ведомым LIN.
 * -  Генерация 13-битного разрыва и обнаружение 10/11 битного разрыва, когда USART аппаратно сконфигурирован для LIN.
 * -  Выход тактового сигнала (для синхронной передачи)
 * -  IrDA SIR Энкодер и Декодер
 *    Поддержка 3/16 битной длинный для нормального режима
 * -  Возможность эмуляции смарт карт
 * -  Интерфейс Smartcard поддерживает асинхронный протокол Smartcards, как определено в стандартах ISO 7816-3.
 *    0.5, 1.5 стоповые биты ля работы со смарт-картой
 * -  Поддержка однопроводной полудуплексной работы
 * -  Настроиваемая многобуферная связь с использованием DMA.
 *    Буферизация полученных/отправленных байт в зарезервированном в SRAM месте с использованием прямого доступа к памяти.
 * -  Отдельные биты включения для передачи и приема.
 * -  Флаги обнаружения:
 *      - приемный буфер заполнен
 *      - буфер передачи пуст
 *      - флаг окончания передачи
 * -  Контроль четности:
 *      - Передает бит четности
 *      - Проверяет четность полученного байта данных
 * - Четыре флага обнаружения ошибок:
 *      - Ошибка по переполнению
 *      - Шум на линии
 *      - Ошибка кадра
 *      - Ошибка четности
 * - Десять прерываний по флагам:
 *      – CTS changes
 *      – LIN break detection
 *      – Transmit data register empty
 *      – Transmission complete
 *      – Receive data register full
 *      – Idle line received
 *      – Overrun error
 *      – Framing error
 *      – Noise error
 *      – Parity error
 * - Multiprocessor communication - enter into mute mode if address match does not occur
 * - Wake up from mute mode (by idle line detection or address mark detection)
 * - Two receiver wakeup modes: Address bit (MSB, 9th bit), Idle line
 
 Для любой двунаправленной связи USART требуется как минимум два контакта: вход для приема данных Rx и выход
 для передачи данных Tx.
 
 RX: Вход для приема данных. Методы передискретизации используются для восстановления данных путем различия достоверных данных и шума.
 TX: Вывод данных. Когда передатчик отключен, выходной контакт возвращается к своей конфигурации порта ввода-вывода.
 Когда передатчик включен и ничего не должно передаваться, то вывод TX находится на высоком уровне. В однопроводном режиме
 и в режиме смарт-карт этот вывод-ввод используется для передачи и приема данных (на уровне USART данные поступают на SW_RX)
 
 Через TX и RX передаются и принимаются последовательные данные в обычном режиме USART в виде кадров, содержащих:
 - Свободная линия перед передачей или приемом
 - Стартовый бит
 - Слово данных (8 или 9 бит), начиная с младшего значащего бита
 - 0.5, 1, 1.5, 2 стоповые биты, указывающие на завершение кадра.
 - Baud rate generator с 12 битной мантиссой и 4 битной дробью
 - Регистр статуса (USART_SR)
 - Регистр данных (USART_DR)
 - Регистр скорости передачи (USART_BRR) - 12 битная мантисса и 4 битная дробь.
 - Защитный регистр (USART_GTPR) в случае режима смарт-карты
 
 Для интерфейса в синхронном режиме требуется следующий контакт:
  - CK: тактовый выход передатчика. Этот контакт выводит тактовый сигнал данных передатчика для синхронной передачи,
  соответствующей ведомому режиму SPI(нет тактовых импульсов для стартового и стопового битов, а также 
  есть возможность для отправки тактового импульса для последнего бита данных.) Параллельно данные могут быть получены
  синхронно на RX. Это может быть использовано для управления периферийными устройствами, имеющими регистры сдвига
  (например, драйверами LCD). Фаза и полярность программируются программно. В режиме смарт карты CK может обеспечивать
  синхронизацию с смарт картами.
  
  В режиме аппаратного управления потоком требуются следующие контакты:
  - CTS: (Clear To Send) Блокирует передачу данных в конце текущей передачи при высоком уровне
  - RTS: (Request to send)Запрос на отправку укащывает, что USART готов к приему данных (при низком уровне)
  
  27.3.1 USART character description(Страница 790)
  
  Длина слова может быть выбрана равной 8 или 9 битам путем программирования бита М в регистре USART_CR1.
  Вывод TX находится в низком состоянии во время стартового бита и находится в высоком состоянии во время стопового бита.
  
  Свободная линия интерпретируется, как целый кадр из "1", за которым следует стартовый бит следующего кадра, содержащего
  данные (количество "1" включает количество стоповых битов).
  
  Разбитие кадра интерпретируется при получении "0" за весь период кадра. В конце кадра передатчик выставляет 1 или 2 
  стоповых бита(логическая "1") для подтверждения стартового бита.
  
  Передача и прием управляются общим генератором baud rate. (Скорость приема и передачи будет одна)
  
  27.3.2 Transmitter(Страница 791)
  Передатчик может отправлять слова данных длиной 8 или 9 бит, в зависимости от состояния бита M.
  Когда бит разрешения передачи (TE) установлен, данные в сдвиговом регистре передачи выводятся на вывод TX,
  а соответствующие тактовые импульсы выводятся на вывод CK.
  
  Передача байта
  Во время передачи данные смещаются первым младшим значащим битом на выводе TX. В Этом режиме регистр USART_DR
  состоит из буфера (TDR) между внутренней шиной и сдвиговым регистром передачи.
  Каждому символу предшествует стартовый бит, который представляет собой низкий логический уровень в течение одного
  битового периода. Символ завершается конфигурируемым количеством стоповых битов.
  Стоповые биты 0.5, 1, 1.5, или 2.
  
  Внимание!
  Бит TE не должен сбрасываться во время передачи данных. Сброс бита TE во время передачи данных приведет к
  повреждению данных на выводе TX, поскольку счетчики скорости передачи зависнут.
  Текущие передаваемые данные будут потеряны.
  Пустой кадр будет отправлен после включения бита ТЕ.
  */
  
struct USART_name husart1; //Объявляем структуру по USART.(см. stm32f103xx_CMSIS.h)
struct USART_name husart2; //Объявляем структуру по USART.(см. stm32f103xx_CMSIS.h)


/**
 ******************************************************************************
 *  @breif Настройка USART1. Параметры 9600 8 N 1
 ******************************************************************************
 */

void CMSIS_USART1_Init(void) {
	
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Включение тактирование порта А
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN); //Включение альтернативных функций
	
	//Для конфигурирование ножек UART для Full Duplex нужно использовать Alternate function push-pull(См. п.п. 9.1.11 GPIO configurations for device peripherals стр.111 Reference Manual)
	//Tx - Alternative Function output Push-pull(Maximum output speed 50 MHz)
	MODIFY_REG(GPIOA->CRH, GPIO_CRH_CNF9_Msk, 0b10 << GPIO_CRH_CNF9_Pos); 
	MODIFY_REG(GPIOA->CRH, GPIO_CRH_MODE9_Msk, 0b11 << GPIO_CRH_MODE9_Pos);
	//Rx - Input floating
	MODIFY_REG(GPIOA->CRH, GPIO_CRH_CNF10_Msk, 0b1 << GPIO_CRH_CNF10_Pos);
	MODIFY_REG(GPIOA->CRH, GPIO_CRH_MODE10_Msk, 0b00 << GPIO_CRH_MODE10_Pos);
	
	//Запустим тактирование USART1
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
	
	/*Расчет Fractional baud rate generation
	есть формула:
	Tx/Rx baud = fCK/(16*USARTDIV)
	где fCK - Input clock to the peripheral (PCLK1 for USART2, 3, 4, 5 or PCLK2 for USART1)
	в нашем случае fCK = 72000000
	допустим нам нужна скорость 9600
	9600 = 72000000/(16*USARTDIV)
	
	Тогда USARTDIV = 72000000/9600*16 = 468.75
	DIV_Mantissa в данном случае будет 468, что есть 0x1D4
	DIV_Fraction будет, как 0.75*16 = 12, что есть 0xC
	
	Тогда весь регистр USART->BRR для скорости 9600 будет выглядеть, как 0x1D4C.
	
	для примера еще разберем скорость 115200:
	115200 = 72000000/(16*USARTDIV)
	Тогда USARTDIV = 72000000/115200*16 = 39.0625
	DIV_Mantissa в данном случае будет 39, что есть 0x27
	DIV_Fraction будет, как 0.0625*16 = 1, что есть 0x1
	
	Тогда весь регистр USART->BRR для скорости 115200 будет выглядеть, как 0x271.

	*/
	
	MODIFY_REG(USART1->BRR, USART_BRR_DIV_Mantissa_Msk, 0x1D4 << USART_BRR_DIV_Mantissa_Pos);
	MODIFY_REG(USART1->BRR, USART_BRR_DIV_Fraction_Msk, 0xC << USART_BRR_DIV_Fraction_Pos);
	
	//27.6.4 Control register 1(USART_CR1)(см. стр 821 Reference Manual)
	SET_BIT(USART1->CR1, USART_CR1_UE); //USART enable
	CLEAR_BIT(USART1->CR1, USART_CR1_M); //Word lenght 1 Start bit, 8 Data bits, n Stop bit
	CLEAR_BIT(USART1->CR1, USART_CR1_WAKE); //Wake up idle Line
	CLEAR_BIT(USART1->CR1, USART_CR1_PCE); //Partity control disabled
	//настройка прерываний
	CLEAR_BIT(USART1->CR1, USART_CR1_PEIE); //partity error interrupt disabled
	CLEAR_BIT(USART1->CR1, USART_CR1_TXEIE); //TXE interrupt is inhibited
	CLEAR_BIT(USART1->CR1, USART_CR1_TCIE); //Transmission complete interrupt disabled
	SET_BIT(USART1->CR1, USART_CR1_RXNEIE); //Прерывание по приему данных включено
	SET_BIT(USART1->CR1, USART_CR1_IDLEIE); //Прерывание по флагу IDLE включено
	SET_BIT(USART1->CR1, USART_CR1_TE); //Transmitter is enabled
	SET_BIT(USART1->CR1, USART_CR1_RE); //Receiver is enabled and begins searching for a start bit
	CLEAR_BIT(USART1->CR1, USART_CR1_RWU);
	CLEAR_BIT(USART1->CR1, USART_CR1_SBK);
	
	//Остальную настройку, не касающуюся стандартного USART, мы пока трогать не будем, но на всякий случай обнулим
	USART1->CR2 = 0;
	CLEAR_BIT(USART1->CR2, USART_CR2_STOP); //1 стоп бит.
	USART1->CR3 = 0;
	USART1->GTPR = 0;

	NVIC_EnableIRQ(USART1_IRQn); //Включим прерывания по USART1
}

/**
 ******************************************************************************
 *  @breif Настройка USART2. Параметры 9600 8 N 1
 ******************************************************************************
 */

void CMSIS_USART2_Init(void) {
	
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Включение тактирование порта А
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN); //Включение альтернативных функций
	
	//Для конфигурирование ножек UART для Full Duplex нужно использовать Alternate function push-pull(См. п.п. 9.1.11 GPIO configurations for device peripherals стр.111 Reference Manual)
	//Tx - Alternative Function output Push-pull(Maximum output speed 50 MHz)
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF2_Msk, 0b10 << GPIO_CRL_CNF2_Pos); 
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE2_Msk, 0b11 << GPIO_CRL_MODE2_Pos);
	//Rx - Input floating
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF3_Msk, 0b1 << GPIO_CRL_CNF3_Pos);
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE3_Msk, 0b00 << GPIO_CRL_MODE3_Pos);
	
	//Запустим тактирование USART2
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
	
	/*Расчет Fractional baud rate generation
	есть формула:
	Tx/Rx baud = fCK/(16*USARTDIV)
	где fCK - Input clock to the peripheral (PCLK1 for USART2, 3, 4, 5 or PCLK2 for USART1)
	в нашем случае fCK = 36000000
	допустим нам нужна скорость 9600
	9600 = 36000000/(16*USARTDIV)
	
	Тогда USARTDIV = 36000000/9600*16 = 234.375
	DIV_Mantissa в данном случае будет 234, что есть 0xEA
	DIV_Fraction будет, как 0.375*16 = 6, что есть 0x6
	
	Тогда весь регистр USART->BRR для скорости 9600 будет выглядеть, как 0xEA6.
	
	для примера еще разберем скорость 115200: (Неточность по скорости будет 0.15%. Не рекомендуется)
	115200 = 36000000/(16*USARTDIV)
	Тогда USARTDIV = 36000000/115200*16 = 19.53125
	DIV_Mantissa в данном случае будет 19, что есть 0x13
	DIV_Fraction будет, как 0.53125*16 = 8, что есть 0x8
	
	Тогда весь регистр USART->BRR для скорости 115200 будет выглядеть, как 0x138.

	*/
	
	MODIFY_REG(USART2->BRR, USART_BRR_DIV_Mantissa_Msk, 0xEA << USART_BRR_DIV_Mantissa_Pos);
	MODIFY_REG(USART2->BRR, USART_BRR_DIV_Fraction_Msk, 0x6 << USART_BRR_DIV_Fraction_Pos);
	
	//27.6.4 Control register 1(USART_CR1)(см. стр 821 Reference Manual)
	SET_BIT(USART2->CR1, USART_CR1_UE); //USART enable
	CLEAR_BIT(USART2->CR1, USART_CR1_M); //Word lenght 1 Start bit, 8 Data bits, n Stop bit
	CLEAR_BIT(USART2->CR1, USART_CR1_WAKE); //Wake up idle Line
	CLEAR_BIT(USART2->CR1, USART_CR1_PCE); //Partity control disabled
	//настройка прерываний
	CLEAR_BIT(USART2->CR1, USART_CR1_PEIE); //partity error interrupt disabled
	CLEAR_BIT(USART2->CR1, USART_CR1_TXEIE); //TXE interrupt is inhibited
	CLEAR_BIT(USART2->CR1, USART_CR1_TCIE); //Transmission complete interrupt disabled
	SET_BIT(USART2->CR1, USART_CR1_RXNEIE); //Прерывание по приему данных включено
	SET_BIT(USART2->CR1, USART_CR1_IDLEIE); //Прерывание по флагу IDLE включено
	SET_BIT(USART2->CR1, USART_CR1_TE); //Transmitter is enabled
	SET_BIT(USART2->CR1, USART_CR1_RE); //Receiver is enabled and begins searching for a start bit
	CLEAR_BIT(USART2->CR1, USART_CR1_RWU);
	CLEAR_BIT(USART2->CR1, USART_CR1_SBK);
	
	//Остальную настройку, не касающуюся стандартного USART, мы пока трогать не будем, но на всякий случай обнулим
	USART2->CR2 = 0;
	CLEAR_BIT(USART2->CR2, USART_CR2_STOP); //1 стоп бит.
	USART2->CR3 = 0;
	USART2->GTPR = 0;

	NVIC_EnableIRQ(USART2_IRQn); //Включим прерывания по USART2
}

/**
 ******************************************************************************
 *  @breif Прерывание по USART1
 ******************************************************************************
 */

__WEAK void USART1_IRQHandler(void) {
	if (READ_BIT(USART1->SR, USART_SR_RXNE)) {
		//Если пришли данные по USART
		husart1.rx_buffer[husart1.rx_counter] = USART1->DR; //Считаем данные в соответствующую ячейку в rx_buffer
		husart1.rx_counter++; //Увеличим счетчик принятых байт на 1
	}
	if (READ_BIT(USART1->SR, USART_SR_IDLE)) {
		//Если прилетел флаг IDLE
		USART1->DR; //Сбросим флаг IDLE
		husart1.rx_len = husart1.rx_counter; //Узнаем, сколько байт получили
		husart1.rx_counter = 0; //сбросим счетчик приходящих данных
	}
}


__WEAK void USART2_IRQHandler(void) {
	if (READ_BIT(USART2->SR, USART_SR_RXNE)) {
		//Если пришли данные по USART
		husart2.rx_buffer[husart2.rx_counter] = USART2->DR; //Считаем данные в соответствующую ячейку в rx_buffer
		husart2.rx_counter++; //Увеличим счетчик принятых байт на 1
	}
	if (READ_BIT(USART2->SR, USART_SR_IDLE)) {
		//Если прилетел флаг IDLE
		USART2->DR; //Сбросим флаг IDLE
		husart2.rx_len = husart2.rx_counter; //Узнаем, сколько байт получили
		husart2.rx_counter = 0; //сбросим счетчик приходящих данных
	}
}


/**
 ******************************************************************************
 *  @breif Функция отправки данных по USART
 *  @param  *USART - USART, с которого будут отправляться данные
 *  @param  *data - данные, которые будем отправлять 	         
 *  @param  Size - сколько байт требуется передать
 ******************************************************************************
 */

bool CMSIS_USART_Transmit(USART_TypeDef *USART, uint8_t *data, uint16_t Size, uint32_t Timeout_ms) {
	for (uint16_t i = 0; i < Size; i++) {
		Timeout_counter_ms = Timeout_ms;
		//Ждем, пока линия не освободится
		while (READ_BIT(USART->SR, USART_SR_TXE) == 0) {
			if (!Timeout_counter_ms) {
				return false;
			}
		}
		USART->DR = *data++; //Кидаем данные  
	}
	return true;
}



/*================================= НАСТРОЙКА I2C ============================================*/
	
/**
***************************************************************************************
*  @breif Inter-integrated circuit (I2C) interface
*  Reference Manual/см. п.26 Inter-integrated circuit (I2C) interface (стр. 752)
***************************************************************************************
*/

/*Функциональное описание*/
/*
 * Помимо приема и передачи данных, этот интерфейс преобразует из последовательного формата
 * в параллельный и наоборот. Прерывания включаются или отключаются программно.
 * Интерфейс имеет вывод данных (SDA) и вывод синхронизации (SCL)
 * Его можно подключить с стандартной скоростью до 100 кГц или FastMode (до 400 кГц).
 */

/*Выбор режима*/
/*
 Интерфейс может работать в одном из четырех режимов:
   - Slave transmitter
   - Slave receiver
   - Master transmitter
   - Master receiver

По-умолчанию он работает в slave режиме. Интерфейм автоматически переключается с slave на 
master после того, как он сгенерирует условие START, и с master на slave, если происходит потеря
арбитража или генерация STOP, что обеспечивает возможность работы в резиме мультимастера.
*/


/*Коммуникационный поток*/
/*
 * В режиме Master интерфейс I2C инициирует передачу данных и генерирует тактовый сигнал. 
 * A последовательная передача данных всегда начинается с условия "старт" и заканчивается условием "стоп". 
 * В режиме ведущего устройства условия старта и останова генерируются программно.
 * В режиме Slave интерфейс способен распознавать свои собственные адреса (7 или 10-битные), а также
 * адрес общего вызова. Распознавание адреса общего вызова может быть включено или отключено программным путем.
 * Данные и адреса передаются в виде 8-битных байтов, MSB первым. 
 * Первый байт(ы) содержат адрес (один в 7-битном режиме, два в 10-битном режиме). 
 * Адрес всегда передается в режиме ведущего устройства.
 * За 8 тактами передачи байта следует 9-й тактовый импульс, в течение которого приемник должен передать передатчику бит подтверждения
*/

/*Регистры(См. ReferenceManual стр. 772)*/

void CMSIS_I2C_Reset(void) {
	//Сброс настроек I2C
	//п.п. 26.6.1 I2C Control register 1 (I2C_CR1) (стр. 772)
	SET_BIT(I2C1->CR1, I2C_CR1_SWRST); //: I2C Peripheral not under reset
	while (READ_BIT(I2C1->CR1, I2C_CR1_SWRST) == 0) ;
	CLEAR_BIT(I2C1->CR1, I2C_CR1_SWRST); //: I2C Peripheral not under reset
	while (READ_BIT(I2C1->CR1, I2C_CR1_SWRST)) ;
	/* Примечание: Этот бит можно использовать для повторной инициализации 
	 * периферийного устройства после ошибки или заблокированного состояния.
	 * Например, если бит BUSY установлен и остается заблокированным из-за сбоя на шине,
	 * бит SWRST можно использовать для выхода из этого состояния.*/
}

/**
 *************************************************************************************
 *  @breif Функция инициализации шины I2C1. Sm.
 *************************************************************************************
 */

void CMSIS_I2C1_Init(void) {
	//Настройки тактирования
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN); //Запуск тактирование порта B
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN); //Запуск тактирования альтернативных функций
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN); //Запуск тактирования I2C1
	
	//Настройки ножек SDA и SCL
	//PB7 SDA (I2C Data I/O) Alternate function open drain
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF7_Msk, 0b11 << GPIO_CRL_CNF7_Pos); //Alternate function open drain
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_MODE7_Msk, 0b11 << GPIO_CRL_MODE7_Pos); //Maximum output speed 50 MHz
	//PB6 SCL (I2C clock) Alternate function open drain
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF6_Msk, 0b11 << GPIO_CRL_CNF6_Pos); //Alternate function open drain
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_MODE6_Msk, 0b11 << GPIO_CRL_MODE6_Pos); //Maximum output speed 50 MHz
	
	//26.6 I2C registers( См. Reference Manual стр. 772)
	
	//п.п. 26.6.1 I2C Control register 1 (I2C_CR1) (стр. 772)
	CMSIS_I2C_Reset();
	
	/*Это все для инита не нужно. После сброса итак будет в 0. */
	CLEAR_BIT(I2C1->CR1, I2C_CR1_ALERT); //Releases SMBA pin high.Alert Response Address Header followed by NACK
	CLEAR_BIT(I2C1->CR1, I2C_CR1_PEC); //No PEC transfer
	CLEAR_BIT(I2C1->CR1, I2C_CR1_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
	CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK); //No acknowledge returned
	CLEAR_BIT(I2C1->CR1, I2C_CR1_STOP); //No Stop generation
	CLEAR_BIT(I2C1->CR1, I2C_CR1_START); //No Start generation
	CLEAR_BIT(I2C1->CR1, I2C_CR1_NOSTRETCH); //Clock stretching enabled
	CLEAR_BIT(I2C1->CR1, I2C_CR1_ENGC); //General call disabled. Address 00h is NACKed.
	CLEAR_BIT(I2C1->CR1, I2C_CR1_ENPEC); //PEC calculation disabled
	CLEAR_BIT(I2C1->CR1, I2C_CR1_ENARP); //ARP disable
	CLEAR_BIT(I2C1->CR1, I2C_CR1_SMBTYPE); //SMBus Device
	CLEAR_BIT(I2C1->CR1, I2C_CR1_SMBUS); //I2C mode
	
	//п.п. 26.6.2 I2C Control register 2(I2C_CR2)(стр.774)
	CLEAR_BIT(I2C1->CR2, I2C_CR2_LAST); //Next DMA EOT is not the last transfer
	CLEAR_BIT(I2C1->CR2, I2C_CR2_DMAEN); //DMA requests disabled
	CLEAR_BIT(I2C1->CR2, I2C_CR2_ITBUFEN); //TxE = 1 or RxNE = 1 does not generate any interrupt.
	CLEAR_BIT(I2C1->CR2, I2C_CR2_ITEVTEN); //Event interrupt disabled
	CLEAR_BIT(I2C1->CR2, I2C_CR2_ITERREN); //Error interrupt disabled
	MODIFY_REG(I2C1->CR2, I2C_CR2_FREQ_Msk, 36 << I2C_CR2_FREQ_Pos); //f PCLK1 = 36 Мгц
		
	//п.п. 26.6.3 I2C Own address register 1(I2C_OAR1)(стр.776)
	I2C1->OAR1 = 0;
	//п.п. 26.6.4 I2C Own address register 1(I2C_OAR2)(стр.776)
	I2C1->OAR2 = 0;
	
	//п.п. 26.6.8 I2C Clock control register (I2C_CCR)(стр.781)
	CLEAR_BIT(I2C1->CCR, I2C_CCR_FS); //Standard mode I2C
	//SET_BIT(I2C1->CCR, I2C_CCR_FS); //Fast mode I2C

	CLEAR_BIT(I2C1->CCR, I2C_CCR_DUTY); //Fm mode tlow/thigh = 2
	//SET_BIT(I2C1->CCR, I2C_CCR_DUTY); //Fm mode tlow/thigh = 16/9 (see CCR)
	
	//Расчет CCR. Смотри примеры расчета
	//MODIFY_REG(I2C1->CCR, I2C_CCR_CCR_Msk, 180 << I2C_CCR_CCR_Pos); //для Sm mode
	MODIFY_REG(I2C1->CCR, I2C_CCR_CCR_Msk, 30 << I2C_CCR_CCR_Pos); //для Fm mode. DUTY 0.
	//MODIFY_REG(I2C1->CCR, I2C_CCR_CCR_Msk, 4 << I2C_CCR_CCR_Pos); //для Fm mode. DUTY 1.
	
	//п.п. 26.6.9 I2C TRISE register (I2C_TRISE)(стр. 782)
	MODIFY_REG(I2C1->TRISE, I2C_TRISE_TRISE_Msk, 37 << I2C_TRISE_TRISE_Pos); //для Sm mode
	//MODIFY_REG(I2C1->TRISE, I2C_TRISE_TRISE_Msk, 12 << I2C_TRISE_TRISE_Pos); //для Fm mode
	
	SET_BIT(I2C1->CR1, I2C_CR1_PE); //I2C1 enable
}


/**
 *************************************************************************************
 *  @breif Функция сканирования устройства по заданному 7-битному адресу
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства         
 *  @retval  Возвращает статус true - если устройство по заданному адресу отозвалось, 
 *           false - если устройство по заданному адресу не отвечает
 *************************************************************************************
 */
bool CMSIS_I2C_Adress_Device_Scan(I2C_TypeDef *I2C, uint8_t Adress_Device, uint32_t Timeout_ms) {
	
	/*-------------------Проверка занятости шины-------------------*/
	if (READ_BIT(I2C->SR2, I2C_SR2_BUSY)) {
		//Если шина занята
		
		if ((READ_BIT(GPIOB->IDR, GPIO_IDR_IDR6)) && (READ_BIT(GPIOB->IDR, GPIO_IDR_IDR7))) {
			//Если линия на самом деле свободна, а BUSY висит
			CMSIS_I2C_Reset(); //ресет
			CMSIS_I2C1_Init(); //повторная инициализация
		} 
		
		if (READ_BIT(I2C->SR2, I2C_SR2_MSL)) {
			//Если стоит статус, что мы в мастере
			SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправим сигнал STOP
		} 
		
		if (I2C->CR1 != 1) {
			//Если в CR1 что-то лишнее, то перезагрузим I2C
			CLEAR_BIT(I2C->CR1, I2C_CR1_PE);
			SET_BIT(I2C->CR1, I2C_CR1_PE);
		} 	
		
		return false;	
	}
	/*-------------------Проверка занятости шины-------------------*/
	
	CLEAR_BIT(I2C->CR1, I2C_CR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
	SET_BIT(I2C->CR1, I2C_CR1_START); //Отправляем сигнал START
	
	Timeout_counter_ms = Timeout_ms;
	while (READ_BIT(I2C->SR1, I2C_SR1_SB) == 0) {
		//Ожидаем до момента, пока не сработает Start condition generated
	
		if (!Timeout_counter_ms) {
			return false;
		}
		
	} 
	//ВНИМАНИЕ!
	/* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью данных в регистр DR или когда PE=0*/
	I2C->SR1;
	I2C->DR = (Adress_Device << 1); //Адрес + Write
	
	Timeout_counter_ms = Timeout_ms;
	while ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C->SR1, I2C_SR1_ADDR) == 0)) {
		//Ждем, пока адрес отзовется
		
		if (!Timeout_counter_ms) {
			return false;
		}
		
	}
	
	if (READ_BIT(I2C->SR1, I2C_SR1_ADDR)) {
		//Если устройство отозвалось
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправляем сигнал STOP
		/*Сброс бита ADDR производится чтением SR1, а потом SR2*/
		I2C->SR1;
		I2C->SR2;
		return true;
	} else {
		//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправляем сигнал STOP
		CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
		return false;
	}
}



/**
 **************************************************************************************************
 *  @breif Функция передачи данных по I2C
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства      
 *  @param  *data - Данные, которые будем отправлять
 *  @param  Size_data - Размер, сколько байт будем отправлять.
 *  @retval  Возвращает статус отправки данных. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_I2C_Data_Transmit(I2C_TypeDef *I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	
	/*-------------------Проверка занятости шины-------------------*/
	if (READ_BIT(I2C->SR2, I2C_SR2_BUSY)) {
		//Если шина занята
		
		if ((READ_BIT(GPIOB->IDR, GPIO_IDR_IDR6)) && (READ_BIT(GPIOB->IDR, GPIO_IDR_IDR7))) {
			//Если линия на самом деле свободна, а BUSY висит
			CMSIS_I2C_Reset(); //ресет
			CMSIS_I2C1_Init(); //повторная инициализация
		} 
		
		if (READ_BIT(I2C->SR2, I2C_SR2_MSL)) {
			//Если стоит статус, что мы в мастере
			SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправим сигнал STOP
		} 
		
		if (I2C->CR1 != 1) {
			//Если в CR1 что-то лишнее, то перезагрузим I2C
			CLEAR_BIT(I2C->CR1, I2C_CR1_PE);
			SET_BIT(I2C->CR1, I2C_CR1_PE);
		} 	
		
		return false;	
	}
	/*-------------------Проверка занятости шины-------------------*/
	
	CLEAR_BIT(I2C->CR1, I2C_CR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
	SET_BIT(I2C->CR1, I2C_CR1_START); //Стартуем.
	
	Timeout_counter_ms = Timeout_ms;
	while (READ_BIT(I2C->SR1, I2C_SR1_SB) == 0) {
		//Ожидаем до момента, пока не сработает Start condition generated
	
		if (!Timeout_counter_ms) {
			return false;
		}
		
	} 
	//ВНИМАНИЕ!
	/* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
	I2C->SR1;
	I2C->DR = (Adress_Device << 1); //Адрес + Write
	
	Timeout_counter_ms = Timeout_ms;
	while ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C->SR1, I2C_SR1_ADDR) == 0)) {
		//Ждем, пока адрес отзовется
		
		if (!Timeout_counter_ms) {
			return false;
		}
		
	}
	
	if (READ_BIT(I2C->SR1, I2C_SR1_ADDR)) {
		//Если устройство отозвалось, сбросим бит ADDR
		/*Сброс бита ADDR производится чтением SR1, а потом SR2*/
		I2C->SR1;
		I2C->SR2;
		
		/*Отправим данные*/
		for (uint16_t i = 0; i < Size_data; i++) {
			I2C->DR = *(data + i); //Запись байта
			while (READ_BIT(I2C->SR1, I2C_SR1_TXE) == 0) {
				//Ждем, пока данные загрузятся в регистр сдвига.
				
				if ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 1)) {
					//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
					SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
					CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
					return false;
				}
			} 
		}
		
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем

		return true;
	
	} else {
		//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
		CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
		
		return false;
	}
}


/**
 **************************************************************************************************
 *  @breif Функция приема данных по I2C
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства      
 *  @param  *data - Куда будем записывать принятые данные
 *  @param  Size_data - Размер, сколько байт будем принимать.
 *  @retval  Возвращает статус приема данных. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_I2C_Data_Receive(I2C_TypeDef *I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	
	/*-------------------Проверка занятости шины-------------------*/
	if (READ_BIT(I2C->SR2, I2C_SR2_BUSY)) {
		//Если шина занята
		
		if ((READ_BIT(GPIOB->IDR, GPIO_IDR_IDR6)) && (READ_BIT(GPIOB->IDR, GPIO_IDR_IDR7))) {
			//Если линия на самом деле свободна, а BUSY висит
			CMSIS_I2C_Reset(); //ресет
			CMSIS_I2C1_Init(); //повторная инициализация
		} 
		
		if (READ_BIT(I2C->SR2, I2C_SR2_MSL)) {
			//Если стоит статус, что мы в мастере
			SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправим сигнал STOP
		} 
		
		if (I2C->CR1 != 1) {
			//Если в CR1 что-то лишнее, то перезагрузим I2C
			CLEAR_BIT(I2C->CR1, I2C_CR1_PE);
			SET_BIT(I2C->CR1, I2C_CR1_PE);
		} 	
		
		return false;	
	}
	/*-------------------Проверка занятости шины-------------------*/
	
	CLEAR_BIT(I2C->CR1, I2C_CR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
	SET_BIT(I2C->CR1, I2C_CR1_START); //Стартуем.
	
	Timeout_counter_ms = Timeout_ms;
	while (READ_BIT(I2C->SR1, I2C_SR1_SB) == 0) {
		//Ожидаем до момента, пока не сработает Start condition generated
	
		if (!Timeout_counter_ms) {
			return false;
		}
		
	} 
	//ВНИМАНИЕ!
	/* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
	I2C->SR1;
	I2C->DR = (Adress_Device << 1 | 1); //Адрес + команда Read
		
	Timeout_counter_ms = Timeout_ms;
	while ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C->SR1, I2C_SR1_ADDR) == 0)) {
		//Ждем, пока адрес отзовется
		
		if (!Timeout_counter_ms) {
			return false;
		}
		
	}
		
	if (READ_BIT(I2C->SR1, I2C_SR1_ADDR)) {
		//Если устройство отозвалось, сбросим бит ADDR
		/*Сброс бита ADDR производится чтением SR1, а потом SR2*/
		I2C->SR1;
		I2C->SR2;
			
		/*Прочтем данные*/
		for (uint16_t i = 0; i < Size_data; i++) {
			if (i < Size_data - 1) {
				SET_BIT(I2C->CR1, I2C_CR1_ACK); //Если мы хотим принять следующий байт, то отправляем ACK
				
				Timeout_counter_ms = Timeout_ms;
				while (READ_BIT(I2C->SR1, I2C_SR1_RXNE) == 0) {
					//Ожидаем, пока в сдвиговом регистре появятся данные
					if (!Timeout_counter_ms) {
						return false;
					}
				}
				
				*(data + i) = I2C->DR; //Чтение байта
			} else {
				CLEAR_BIT(I2C->CR1, I2C_CR1_ACK); //Если мы знаем, что следующий принятый байт будет последним, то отправим NACK
		
				SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
				Timeout_counter_ms = Timeout_ms;
				while (READ_BIT(I2C->SR1, I2C_SR1_RXNE) == 0) {
					//Ожидаем, пока в сдвиговом регистре появятся данные
					if (!Timeout_counter_ms) {
						return false;
					}
				}
				*(data + i) = I2C->DR; //Чтение байта
			}
		} return true;
			
	} else {
		//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
		CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
		return false;
	}
		
} 


/**
 **************************************************************************************************
 *  @breif Функция записи в память по указанному адресу
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства      
 *  @param  Adress_data - Адрес в памяти, куда будем записывать данные
 *  @param  Size_adress - Размер адреса в байтах. Пример: 1 - 8 битный адрес. 2 - 16 битный адрес.
 *  @param  *data - Данные, которые будем записывать
 *  @param  Size_data - Размер, сколько байт будем записывать.
 *  @retval  Возвращает статус записи. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_I2C_MemWrite(I2C_TypeDef *I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	
	/*-------------------Проверка занятости шины-------------------*/
	if (READ_BIT(I2C->SR2, I2C_SR2_BUSY)) {
		//Если шина занята
		
		if ((READ_BIT(GPIOB->IDR, GPIO_IDR_IDR6)) && (READ_BIT(GPIOB->IDR, GPIO_IDR_IDR7))) {
			//Если линия на самом деле свободна, а BUSY висит
			CMSIS_I2C_Reset(); //ресет
			CMSIS_I2C1_Init(); //повторная инициализация
		} 
		
		if (READ_BIT(I2C->SR2, I2C_SR2_MSL)) {
			//Если стоит статус, что мы в мастере
			SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправим сигнал STOP
		} 
		
		if (I2C->CR1 != 1) {
			//Если в CR1 что-то лишнее, то перезагрузим I2C
			CLEAR_BIT(I2C->CR1, I2C_CR1_PE);
			SET_BIT(I2C->CR1, I2C_CR1_PE);
		} 	
		
		return false;	
	}
	/*-------------------Проверка занятости шины-------------------*/
	
	CLEAR_BIT(I2C->CR1, I2C_CR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
	SET_BIT(I2C->CR1, I2C_CR1_START); //Стартуем.
	
	Timeout_counter_ms = Timeout_ms;
	while (READ_BIT(I2C->SR1, I2C_SR1_SB) == 0) {
		//Ожидаем до момента, пока не сработает Start condition generated
			
		if (!Timeout_counter_ms) {
			return false;
		}
		
	} 
	//ВНИМАНИЕ!
	/* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
	I2C->SR1;
	I2C->DR = (Adress_Device << 1); //Адрес + Write
	
	Timeout_counter_ms = Timeout_ms;
	while ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C->SR1, I2C_SR1_ADDR) == 0)) {
		//Ждем, пока адрес отзовется
		
		if (!Timeout_counter_ms) {
			return false;
		}
		
	}
	if (READ_BIT(I2C->SR1, I2C_SR1_ADDR)) {
		//Если устройство отозвалось, сбросим бит ADDR
		/*Сброс бита ADDR производится чтением SR1, а потом SR2*/
		I2C->SR1;
		I2C->SR2;
		
		/*Отправим адрес памяти*/
		for (uint16_t i = 0; i < Size_adress; i++) {
			I2C->DR = *((uint8_t*)&Adress_data + (Size_adress - 1 - i)); //Запись байта
			while (READ_BIT(I2C->SR1, I2C_SR1_TXE) == 0) { 
				//Ждем, пока данные загрузятся в регистр сдвига.
				
				if ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 1)) {
					//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
					SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
					CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
					return false;
				}	
			}
		}
	
		/*Будем записывать данные в ячейку памяти, начиная с указанного адреса*/
		for (uint16_t i = 0; i < Size_data; i++) {
			I2C->DR = *(data + i); //Запись байта
			while (READ_BIT(I2C->SR1, I2C_SR1_TXE) == 0) {
				//Ждем, пока данные загрузятся в регистр сдвига.
				
				if ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 1)) {
					//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
					SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
					CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
					return false;
				}
			} 
		}
		
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем

		return true;
	
	} else {
		//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
		CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
		
		return false;
	}
}


/**
 **************************************************************************************************
 *  @breif Функция чтения из памяти по указанному адресу
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства      
 *  @param  Adress_data - Адрес в памяти, откуда будем считывать данные
 *  @param  Size_adress - Размер адреса в байтах. Пример: 1 - 8 битный адрес. 2 - 16 битный адрес.
 *  @param  *data - Данные, в которые будем записывать считанную информацию.
 *  @param  Size_data - Размер, сколько байт будем считывать.
 *  @retval  Возвращает статус считывания. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_I2C_MemRead(I2C_TypeDef *I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	
	/*-------------------Проверка занятости шины-------------------*/
	if (READ_BIT(I2C->SR2, I2C_SR2_BUSY)) {
		//Если шина занята
		
		if ((READ_BIT(GPIOB->IDR, GPIO_IDR_IDR6)) && (READ_BIT(GPIOB->IDR, GPIO_IDR_IDR7))) {
			//Если линия на самом деле свободна, а BUSY висит
			CMSIS_I2C_Reset(); //ресет
			CMSIS_I2C1_Init(); //повторная инициализация
		} 
		
		if (READ_BIT(I2C->SR2, I2C_SR2_MSL)) {
			//Если стоит статус, что мы в мастере
			SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправим сигнал STOP
		} 
		
		if (I2C->CR1 != 1) {
			//Если в CR1 что-то лишнее, то перезагрузим I2C
			CLEAR_BIT(I2C->CR1, I2C_CR1_PE);
			SET_BIT(I2C->CR1, I2C_CR1_PE);
		} 	
		
		return false;	
	}
	/*-------------------Проверка занятости шины-------------------*/
	
	CLEAR_BIT(I2C->CR1, I2C_CR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
	SET_BIT(I2C->CR1, I2C_CR1_START); //Стартуем.
	
	Timeout_counter_ms = Timeout_ms;
	while (READ_BIT(I2C->SR1, I2C_SR1_SB) == 0) {
		//Ожидаем до момента, пока не сработает Start condition generated
			
		if (!Timeout_counter_ms) {
			return false;
		}
		
	} 
	//ВНИМАНИЕ!
	/* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
	I2C->SR1;
	I2C->DR = (Adress_Device << 1); //Адрес + команда Write
	
	Timeout_counter_ms = Timeout_ms;
	while ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C->SR1, I2C_SR1_ADDR) == 0)) {
		//Ждем, пока адрес отзовется
		
		if (!Timeout_counter_ms) {
			return false;
		}
		
	}
	
	if (READ_BIT(I2C->SR1, I2C_SR1_ADDR)) {
		//Если устройство отозвалось, сбросим бит ADDR
		/*Сброс бита ADDR производится чтением SR1, а потом SR2*/
		I2C->SR1;
		I2C->SR2;
		
		/*Отправим адрес памяти*/
		for (uint16_t i = 0; i < Size_adress; i++) {
			I2C->DR = *((uint8_t*)&Adress_data + (Size_adress - 1 - i)); //Запись байта
			while (READ_BIT(I2C->SR1, I2C_SR1_TXE) == 0) { 
				//Ждем, пока данные загрузятся в регистр сдвига.
				
				if ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 1)) {
					//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
					SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
					CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
					return false;
				}	
			}
		}
		
		//Повторный старт
		SET_BIT(I2C->CR1, I2C_CR1_START); //Стартуем.
		
		Timeout_counter_ms = Timeout_ms;
		while (READ_BIT(I2C->SR1, I2C_SR1_SB) == 0) {
			//Ожидаем до момента, пока не сработает Start condition generated
			
			if (!Timeout_counter_ms) {
				return false;
			}
		
		} 
		//ВНИМАНИЕ!
		/* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
		I2C->SR1;
		I2C->DR = (Adress_Device << 1 | 1); //Адрес + команда Read
		
		Timeout_counter_ms = Timeout_ms;
		while ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C->SR1, I2C_SR1_ADDR) == 0)) {
			//Ждем, пока адрес отзовется
		
			if (!Timeout_counter_ms) {
				return false;
			}
		
		}
		
		if (READ_BIT(I2C->SR1, I2C_SR1_ADDR)) {
			//Если устройство отозвалось, сбросим бит ADDR
			/*Сброс бита ADDR производится чтением SR1, а потом SR2*/
			I2C->SR1;
			I2C->SR2;
			
			/*Прочтем данные, начиная с указанного адреса*/
			for (uint16_t i = 0; i < Size_data; i++) {
				if (i < Size_data - 1) {
					SET_BIT(I2C->CR1, I2C_CR1_ACK); //Если мы хотим принять следующий байт, то отправляем ACK 
					while (READ_BIT(I2C->SR1, I2C_SR1_RXNE) == 0) ;
					*(data + i) = I2C->DR; //Чтение байта
				} else {
					CLEAR_BIT(I2C->CR1, I2C_CR1_ACK); //Если мы знаем, что следующий принятый байт будет последним, то отправим NACK
		
					SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
					while (READ_BIT(I2C->SR1, I2C_SR1_RXNE) == 0) ; //Подождем, пока сдвиговый регистр пополнится новым байтом данных
					*(data + i) = I2C->DR; //Чтение байта
				}
			} return true;
			
		} else {
			//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
			SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
			CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
			return false;
		}
		
	} else {
		//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
		CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
		return false;
	}
}


/*================================= НАСТРОЙКА SPI ============================================*/
	
/**
***************************************************************************************
*  @breif Serial peripheral interface (SPI)
*  Reference Manual/см. п.25 Serial peripheral interface (SPI) (стр. 699)
***************************************************************************************
*/

/*Функциональное описание*/
/*
 * Обеспечивает полудуплексную, синхронную, последовательную связь с внешними устройствами.
 * Интерфейс может быть сконфигурирован, как ведущий, и в этом случае он обеспечивает 
 * синхронизацию связи SCK с внешним ведомым устройством. Интерфейс также может работать
 * в конфигурации с несколькими мастерами.
 */

/*Функции:*/
/*
 * - Полный дюплекс и синхронная передача по трем ляниям
 * - Симплексная синхронная передача по двум линиям с двунаправленной линией данных или без нее
 * - Выбор формата 8 или 16 битного кадра передачи
 * - Работа в режиме Master и Slave
 * - Возможность работы в режиме Multimaster
 * - 8 предварительных делителей скорости передачи данных в ведущем режиме (fPCLK/2 max)
 * - Частота слейв мода (fPCLK/2 max)
 * - Более быстрая связь, как для мастера, так и для слейва(что?)
 * - Управление NSS с помощью аппаратного или программного обеспечения, как для ведущего,
 *   так и для ведомого: динамическая смена операция ведущего/ведомого
 * - Программируемая полярность и фаза (Polarity and phase)
 * - Программируемый порядок данных со свигом в сторону старших или младших разрядов
 * - Специальные флаги передачи и приема с возможностью прерывания.
 * - Флаг состояния занятости шины SPI
 * - Аппаратный CRC:
 *   - значение CRC может быть передано, как последний байт в режиме Tx
 *   - автоматическая проверка ошибок CRC для последнего полученного байта
 * - Флаги неисправности, переполнения и ошибок CRC в ведущем режиме, с возможностью прерывания.
 * - 1 байтовый буфер передачи и приема с возможностью DMA: запросы Tx и Rx.
*/

/*SPI interrupts (см. п.п. 25.3.11 стр.722)*/
/*---------------------SPI interrupt requests-------------------------*/
/*______________________________________________________________________________
  |______Interrupt event________|_______Event flag______|___Enable Control bit__|
  |Transmit buffer empty flag   |          TXE          |          TXEIE        |
  |Receive buffer not empty flag|          RXNE         |__________RXNEIE_______|
  |Master Mode fault event      |          MODF         |                       |
  |Overrun error                |          OVR          |          ERRIE        |
  |CRC error flag_______________|__________CRCERR_______|_______________________|
  
  */

/*----SPI and I2S registers(см п.п. 25.5  стр 742)-------*/

void CMSIS_SPI1_init(void) {
	/*Настройка GPIO*/
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN); //Включение альтернативных функций
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN); //Включение тактирования SPI1
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Включение тактирования порта А
	/*Какие ножки:*/
	//PA4 - NSS
	//PA5 - SCK
	//PA6 - MISO
	//PA7 - MOSI
	//Мы будем настраивать SPI в режим Master
	/*
	 * SPIx_SCK  Master - Alternate function push-pull
	 * SPIx_MOSI:
	 *             Full duplex / master - Alternate function push-pull
	 *             Simplex bidirectional data wire / master - Alternate function push-pull
	 * SPIx_MISO:  
	 *             Full duplex / master - Input floating / Input pull-up
	 * 
	 * SPIx_NSS:  
	 *             Hardware master /slave - Input floating/ Input pull-up / Input pull-down
	 *             Hardware master/ NSS output enabled - Alternate function push-pull
	 *             Software - Not used. Can be used as a GPIO
	 */
    //Настроим сами ножки уже после инициализации SPI, чтоб при старте не было лишних ногодерганий. 	
   
    /*SPI control register 1 (SPI_CR1) (not used in I2S mode)(см. п.п. 25.5.1 стр 742)*/
	MODIFY_REG(SPI1->CR1, SPI_CR1_BR, 0b001 << SPI_CR1_BR_Pos); //fPCLK/4. 72000000/4 = 18 MBits/s
	CLEAR_BIT(SPI1->CR1, SPI_CR1_CPOL); //0: CK to 0 when idle
	CLEAR_BIT(SPI1->CR1, SPI_CR1_CPHA); //0: The first clock transition is the first data capture edge
	CLEAR_BIT(SPI1->CR1, SPI_CR1_DFF); //0: 8-bit data frame format is selected for transmission/reception
	CLEAR_BIT(SPI1->CR1, SPI_CR1_LSBFIRST); //0: MSB transmitted first
	SET_BIT(SPI1->CR1, SPI_CR1_SSM); //1: Software slave management enabled
	SET_BIT(SPI1->CR1, SPI_CR1_SSI); //1: Software slave management enabled
	SET_BIT(SPI1->CR1, SPI_CR1_MSTR); //1: Master configuration
	SET_BIT(SPI1->CR1, SPI_CR1_SPE); //Включим SPI
    
	CLEAR_BIT(SPI1->CR1, SPI_CR1_RXONLY); //0: Full duplex (Transmit and receive)
    
	CLEAR_BIT(SPI1->CR1, SPI_CR1_CRCEN); //0: CRC calculation disabled
	CLEAR_BIT(SPI1->CR1, SPI_CR1_CRCNEXT); // 0: Data phase (no CRC phase) 
	CLEAR_BIT(SPI1->CR1, SPI_CR1_BIDIMODE); //0: 2-line unidirectional data mode selected
    
    
	/*SPI control register 2 (SPI_CR2) (см. п.п. 25.5.2 стр 744)*/
	CLEAR_BIT(SPI1->CR2, SPI_CR2_RXDMAEN); //0: Rx buffer DMA disabled
	CLEAR_BIT(SPI1->CR2, SPI_CR2_TXDMAEN); //0: Tx buffer DMA disabled
	CLEAR_BIT(SPI1->CR2, SPI_CR2_SSOE); //0: SS output is disabled in master mode and the cell can work in multimaster configuration
	CLEAR_BIT(SPI1->CR2, SPI_CR2_ERRIE); //0: Error interrupt is masked
	CLEAR_BIT(SPI1->CR2, SPI_CR2_RXNEIE); //0: RXNE interrupt masked 
	CLEAR_BIT(SPI1->CR2, SPI_CR2_TXEIE); //0: TXE interrupt masked 
    
	/*SPI_I2S configuration register (SPI_I2SCFGR) (см. п.п. 25.5.8 стр 748)*/
	CLEAR_BIT(SPI1->I2SCFGR, SPI_I2SCFGR_I2SMOD); //т.к. на F103C6T6 нет I2S, его вырезали, а регистр оставили, нужно просто обнулить данный регистр. Тем самым включим режим SPI mode.
    
    
	//SCK - PA5:
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE5, 0b11 << GPIO_CRL_MODE5_Pos); //Maximum output speed 50 MHz
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF5, 0b10 << GPIO_CRL_CNF5_Pos); //Alternate Function output Push-pull
	//MISO - PA6:
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE6, 0b00 << GPIO_CRL_MODE6_Pos); //Reserved
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF6, 0b1 << GPIO_CRL_CNF6_Pos); //Input pull-up
	SET_BIT(GPIOA->ODR, GPIO_ODR_ODR6); //Pull-Up
	//MOSI - PA7:
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE7, 0b11 << GPIO_CRL_MODE7_Pos); //Maximum output speed 50 MHz
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF7, 0b10 << GPIO_CRL_CNF7_Pos); //Alternate Function output Push-pull 
}

/**
 **************************************************************************************************
 *  @breif Функция передачи данных по шине SPI
 *  @param  *SPI - шина SPI
 *  @param  *data - Данные, которые будем передавать.
 *  @param  Size_data - Размер, сколько байт будем передавать.
 *  @retval  Возвращает статус передачи. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_SPI_Data_Transmit(SPI_TypeDef *SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	if (!READ_BIT(SPI->SR, SPI_SR_BSY)) {
		//Если шина MOSI свободна, то отправляем данные
		for (uint16_t i = 0; i < Size_data; i++) {
			Timeout_counter_ms = Timeout_ms;
			while (!READ_BIT(SPI->SR, SPI_SR_TXE)) {
				//Ждем, пока буфер на передачу не освободится
				if (!Timeout_counter_ms) {
					return false;
				}
			}
			SPI->DR = *(data + i);
		}
		Timeout_counter_ms = Timeout_ms;
		while (READ_BIT(SPI->SR, SPI_SR_BSY)) {
			//Ждем, пока мы освободим шину
			if (!Timeout_counter_ms) {
				return false;
			}
		}
		return true;
	} else {
		return false;
	}
}

/**
 **************************************************************************************************
 *  @breif Функция приема данных по шине SPI
 *  @param  *SPI - шина SPI
 *  @param  *data - Данные, куда будем записывать принятые данные.
 *  @param  Size_data - Размер, сколько байт хотим принять.
 *  @retval  Возвращает статус приема. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_SPI_Data_Receive(SPI_TypeDef *SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	if (!READ_BIT(SPI->SR, SPI_SR_BSY)) {
		//Если шина MOSI свободна, то принимаем данные
		for (uint16_t i = 0; i < Size_data; i++) {
			SPI->DR = 0;//Запустим тактирование
			Timeout_counter_ms = Timeout_ms;
			while (!READ_BIT(SPI->SR, SPI_SR_RXNE)) {
				//Ждем, пока буфер на прием не заполнится
				if (!Timeout_counter_ms) {
					return false;
				}
			}
			*(data + i) = SPI->DR;
		}
		Timeout_counter_ms = Timeout_ms;
		while (READ_BIT(SPI->SR, SPI_SR_BSY)) {
			//Ждем, пока мы освободим шину
			if (!Timeout_counter_ms) {
				return false;
			}
		}
		return true;
	}
	else {
		return false;
	}
}

/**
 **************************************************************************************************
 *  @breif Функция передачи данных по шине SPI. 
 *  Немного быстрее стандартной функции. CS(NSS) уже включен в функцию
 *  @param  *SPI - шина SPI
 *  @param  *data - Данные, которые будем передавать.
 *  @param  Size_data - Размер, сколько байт будем передавать.
 *  @retval  Возвращает статус передачи. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_SPI_Data_Transmit_fast(SPI_TypeDef *SPI, GPIO_TypeDef *GPIO, uint8_t NSS_pin, bool NSS_logic, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	if (!READ_BIT(SPI->SR, SPI_SR_BSY)) {
		//Если шина MOSI свободна, то отправляем данные
		if (NSS_logic) {
			//CS on
			GPIO->BSRR = (0x1UL << (NSS_pin + 16U)); //NSS_ACTIVE_LOW
		}
		else {
			GPIO->BSRR = (0x1UL << NSS_pin); //NSS_ACTIVE_HIGH
		}      
		for (uint16_t i = 0; i < Size_data; i++) {
			Timeout_counter_ms = Timeout_ms;
			while (!READ_BIT(SPI->SR, SPI_SR_TXE)) {
				//Ждем, пока буфер на передачу не освободится
				if (!Timeout_counter_ms) {
					return false;
				}
			}
			SPI->DR = *(data + i);
		}
		Timeout_counter_ms = Timeout_ms;
		while (READ_BIT(SPI->SR, SPI_SR_BSY)) {
			//Ждем, пока мы освободим шину
			if (!Timeout_counter_ms) {
				return false;
			}
		}
		if (NSS_logic) {
			//CS off
			GPIO->BSRR = (0x1UL << NSS_pin); //NSS_ACTIVE_LOW
		}
		else {
			GPIO->BSRR = (0x1UL << (NSS_pin + 16U)); //NSS_ACTIVE_HIGH
		}
		return true;
	}
	else {
		return false;
	}
}

/**
 **************************************************************************************************
 *  @breif Функция приема данных по шине SPI
 *  Немного быстрее стандартной функции. CS(NSS) уже включен в функцию
 *  @param  *SPI - шина SPI
 *  @param  *data - Данные, куда будем записывать принятые данные.
 *  @param  Size_data - Размер, сколько байт хотим принять.
 *  @retval  Возвращает статус приема. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_SPI_Data_Receive_fast(SPI_TypeDef *SPI, GPIO_TypeDef *GPIO, uint8_t NSS_pin, bool NSS_logic, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	if (!READ_BIT(SPI->SR, SPI_SR_BSY)) {
		//Если шина MOSI свободна, то отправляем данные
		if (NSS_logic) {
			//CS on
			GPIO->BSRR = (0x1UL << (NSS_pin + 16U)); //NSS_ACTIVE_LOW
		}
		else {
			GPIO->BSRR = (0x1UL << NSS_pin); //NSS_ACTIVE_HIGH
		}      
		for (uint16_t i = 0; i < Size_data; i++) {
			SPI->DR = 0; //Запустим тактирование
			Timeout_counter_ms = Timeout_ms;
			while (!READ_BIT(SPI->SR, SPI_SR_RXNE)) {
				//Ждем, пока буфер на прием не заполнится
				if (!Timeout_counter_ms) {
					return false;
				}
			}
			*(data + i) = SPI->DR;
		}
		Timeout_counter_ms = Timeout_ms;
		while (READ_BIT(SPI->SR, SPI_SR_BSY)) {
			//Ждем, пока мы освободим шину
			if (!Timeout_counter_ms) {
				return false;
			}
		}
		if (NSS_logic) {
			//CS off
			GPIO->BSRR = (0x1UL << NSS_pin); //NSS_ACTIVE_LOW
		}
		else {
			GPIO->BSRR = (0x1UL << (NSS_pin + 16U)); //NSS_ACTIVE_HIGH
		}
		return true;
	}
	else {
		return false;
	}
}




