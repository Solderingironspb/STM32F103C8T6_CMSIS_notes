/**
 ******************************************************************************
 *  @file STM32F103C8T6_CMSIS_lib.c
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

#include "STM32F103C8T6_CMSIS_lib.h"

/**
 ***************************************************************************************
 *  @breif Настройка МК STM32F103C8T6 на частоту 72MHz от внешнего кварцевого резонатора
 *  Внешний кварцевый резонатор на 8 MHz
 *  USB настроен на 48MHz
 *  MCO подключен к PLL и тактируется от 36MHz
 *  Reference Manual/см. п.7.3 RCC registers (стр. 99)  
 *  В настройке также необходимо настроить FLASH на работу, совместимую с 72MHz:
 *  Reference Manual/см. п.3.2 Memory organization (стр. 49)
 ***************************************************************************************
 */

void RCC_SystemClock_72MHz(void) {

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
	
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE); //Prefetch is enabled(В Cube MX включено и я включил...) 

/**
*  Bit 5 PRFTBS: Prefetch buffer status
*  This bit provides the status of the prefetch buffer.
*  0: Prefetch buffer is disabled
*  1: Prefetch buffer is enabled
*/
	
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBS); //Prefetch buffer is enabled(В Cube MX включено и я включил...) 

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
	
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0) ; //Дожидемся поднятия флага включения PLL
	
	//В итоге должно получится:
	//RCC->CR == 0x030B5A83
	//RCC->CFGR == 0x071D840A
	//К сожалению, нельзя просто так взять и сразу применить значения регистров и настроить все в 2 строчки кода, т.к. порядок выполнения команд играет очень большую роль.
}
