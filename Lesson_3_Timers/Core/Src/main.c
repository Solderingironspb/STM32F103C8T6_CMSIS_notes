#include "main.h"

volatile uint32_t Counter = 0;

void TIM3_IRQHandler(void) {
	if (READ_BIT(TIM3->SR, TIM_SR_UIF)) {
		CLEAR_BIT(TIM3->SR, TIM_SR_UIF);  //Сбросим флаг прерывания
	}
	Counter++;
}


int main(void) {
	CMSIS_Debug_init();
	CMSIS_RCC_SystemClock_72MHz();
	CMSIS_SysTick_Timer_init();
	
	/*Включим тактирование таймера (страница 48)*/
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);  //Запуск тактирования таймера 3
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);  //Запуск тактирования альтернативных функций
	
	/*Настройка таймера 3 (Страница 404)*/
	//15.4.1 TIMx control register 1 (TIMx_CR1)
	
	//SET_BIT(TIM3->CR1, TIM_CR1_CEN);  //Запуск таймера
	CLEAR_BIT(TIM3->CR1, TIM_CR1_UDIS);  //Генерировать событие Update
	CLEAR_BIT(TIM3->CR1, TIM_CR1_URS);  //Генерировать прерывание
	CLEAR_BIT(TIM3->CR1, TIM_CR1_OPM);  //One pulse mode off(Счетчик не останавливается при обновлении)
	CLEAR_BIT(TIM3->CR1, TIM_CR1_DIR);  //Считаем вверх
	MODIFY_REG(TIM3->CR1, TIM_CR1_CMS_Msk, 0b00 << TIM_CR1_CMS_Pos);  //Выравнивание по краю
	SET_BIT(TIM3->CR1, TIM_CR1_ARPE); //Auto-reload preload enable
	MODIFY_REG(TIM3->CR1, TIM_CR1_CKD_Msk, 0b00 << TIM_CR1_CKD_Pos); //Предделение выключено
	
	/*Настройка прерываний (Страница 409)*/
	//15.4.4 TIMx DMA/Interrupt enable register (TIMx_DIER)
	SET_BIT(TIM3->DIER, TIM_DIER_UIE); //Update interrupt enable
	
	//15.4.5 TIMx status register (TIMx_SR) - Статусные регистры
	
	TIM3->PSC = 7200 - 1;
	TIM3->ARR = 10 - 1;
	
	NVIC_EnableIRQ(TIM3_IRQn); //Разрешить прерывания по таймеру 3
	SET_BIT(TIM3->CR1, TIM_CR1_CEN);  //Запуск таймера

	
	/*Настройка ножки PA6 под ШИМ*/
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);  //Включим тактирование порта А
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF6_Msk, 0b10 << GPIO_CRL_CNF6_Pos);
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE6_Msk, 0b11 << GPIO_CRL_MODE6_Pos);
	
	/*Настройка ножки PA7 под ШИМ*/
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF7_Msk, 0b10 << GPIO_CRL_CNF7_Pos);
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE7_Msk, 0b11 << GPIO_CRL_MODE7_Pos);
	
	
	/*Настройка шим(Канал 1)*/
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_CC1S_Msk, 0b00 << TIM_CCMR1_CC1S_Pos);  //CC1 channel is configured as output
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1FE); //Fast mode disable
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1PE);  //Preload enable
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_OC1M_Msk, 0b110 << TIM_CCMR1_OC1M_Pos); //PWM MODE 1
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1CE); //OC1Ref is not affected by the ETRF input
	
	/*Настройка шим(Канал 2)*/
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_CC2S_Msk, 0b00 << TIM_CCMR1_CC2S_Pos); //CC1 channel is configured as output
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC2FE); //Fast mode disable
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC2PE); //Preload enable
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_OC2M_Msk, 0b110 << TIM_CCMR1_OC2M_Pos); //PWM MODE 1
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC2CE); //OC1Ref is not affected by the ETRF input
	
	/*Запуск ШИМ*/
	//15.4.9 TIMx capture/compare enable register (TIMx_CCER)
	SET_BIT(TIM3->CCER, TIM_CCER_CC1E); //On - OC1 signal is output on the corresponding output pin. 
	SET_BIT(TIM3->CCER, TIM_CCER_CC1P); //OC1 active high.
	
	SET_BIT(TIM3->CCER, TIM_CCER_CC2E); //On - OC1 signal is output on the corresponding output pin. 
	CLEAR_BIT(TIM3->CCER, TIM_CCER_CC2P); //OC1 active high.
	
	TIM3->CCR1 = 5;
	TIM3->CCR2 = 5;
	
	while (1) {
		
	}
}