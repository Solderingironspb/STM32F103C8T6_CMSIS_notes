#include "main.h"
volatile uint32_t spi_rx_counter = 0;
volatile uint32_t spi_rx_len = 0;
uint8_t spi_tx_buffer[6] = { 4, 8, 15, 16, 23, 42 };
uint8_t spi_rx_buffer[20] = { 0, };

void SPI1_IRQHandler(void) {
	spi_rx_buffer[spi_rx_counter] = SPI1->DR;
	spi_rx_counter++;
}

void EXTI4_IRQHandler(void) {
	if (READ_BIT(EXTI->PR, EXTI_PR_PR4)) {
		//Если прерывание пришло от EXTI4
			//NSS вернулся в состояние 1
			spi_rx_len = spi_rx_counter;
			spi_rx_counter = 0;
		SET_BIT(EXTI->PR, EXTI_PR_PR4); //Сбросим флаг
	}
}

int main(void) {
	CMSIS_Debug_init();
	CMSIS_RCC_SystemClock_72MHz();
	CMSIS_SysTick_Timer_init();
	CMSIS_SPI1_init();
	/*Настроим реакцию на CS*/
	//PA4 - NSS
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Запуск тактирования порта A
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE4, 0b00 << GPIO_CRL_MODE4_Pos); //Настройка GPIOA Pin 4 на вход
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF4, 0b01 << GPIO_CRL_CNF4_Pos); //Настройка GPIOA Pin 4 Input floating
	/*Настроим EXTI для PA4*/
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN); //Включить тактирование для альтернативных функций
	MODIFY_REG(AFIO->EXTICR[1], AFIO_EXTICR2_EXTI4, AFIO_EXTICR2_EXTI4_PA); //AFIO_EXTICR2, EXTI4, выбран порт A.
	SET_BIT(EXTI->IMR, EXTI_IMR_MR4); //Вкл. прерывание для 4 ножки.
	SET_BIT(EXTI->RTSR, EXTI_RTSR_RT4); //Прерывание будет срабатывать по переходу сигнала из 0 в 1
	NVIC_EnableIRQ(EXTI4_IRQn); //Включим прерывание по EXTI4
	
	while (1) {
		
    	
	}
}