/**
 ******************************************************************************
 *  @file stm32f103xx_CMSIS.h
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

#ifndef __STM32F103XX_CMSIS_H
#define __STM32F103XX_CMSIS_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include <main.h>
#include <stdbool.h>
        void CMSIS_Debug_init(void); //Настройка Debug (Serial Wire)
	void CMSIS_RCC_SystemClock_72MHz(void); //Настрока тактирования микроконтроллера на частоту 72MHz
	void CMSIS_SysTick_Timer_init(void); //Инициализация системного таймера
	void Delay_ms(uint32_t Milliseconds); //Функция задержки
	void SysTick_Handler(void); //Прерывания от системного таймера
	void CMSIS_PC13_OUTPUT_Push_Pull_init(void); //Пример настройки ножки PC13 в режим Push-Pull 50 MHz
	void CMSIS_PA8_MCO_init(void); //Пример настройки ножки PA8 в выход тактирующего сигнала c MCO
	void CMSIS_RCC_AFIO_enable(void);//Включить тактирование для альтернативных функций
	void CMSIS_AFIO_EXTICR1_B0_select(void);//Пример выбора ножки PB0 для работы с EXTI0
	void CMSIS_PB0_INPUT_Pull_Down_init(void);//Настройка ножки PB0 на вход. Подтяжка к земле.
	void CMSIS_EXTI_0_init(void);//Инициализации EXTI0 для PB0 в режиме Rishing edge trigger
	void EXTI0_IRQHandler(void);//Прерывания от EXTI0
	
#ifdef __cplusplus
}
#endif

#endif /* __STM32F103XX_CMSIS_H */
