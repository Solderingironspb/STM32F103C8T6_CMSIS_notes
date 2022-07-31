#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx.h"

void CMSIS_RCC_SystemClock_72MHz(void); //Настрока тактирования микроконтроллера на частоту 72MHz
void CMSIS_SysTick_Timer_init(void); //Инициализация системного таймера
void Delay_ms(uint32_t Milliseconds); //Функция задержки
void SysTick_Handler(void); //Прерывания от системного таймера
void CMSIS_PC13_OUTPUT_Push_Pull_init(void); //Пример настройки ножки PC13 в режим Push-Pull 50 MHz
void CMSIS_PA8_MCO_init(void); //Пример настройки ножки PA8 в выход тактирующего сигнала c MCO

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
