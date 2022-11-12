#include "main.h"
#include "stm32f103xx_CMSIS.h"

#include <stdio.h>
#include <string.h>

uint8_t A[6] = { 4, 8, 15, 16, 23, 42 };

struct USART_name {
	uint8_t tx_buffer[64]; //Буфер под выходящие данные
	uint8_t rx_buffer[64]; //Буфер под входящие данные
	uint16_t rx_counter; //Счетчик приходящих данных типа uint8_t по USART
	uint16_t rx_len; //Количество принятых байт после сработки флага IDLE
};
struct USART_name husart1;




void CMSIS_USART_Transmit(USART_TypeDef *USART, uint8_t *data, uint16_t Size) {
	for (uint16_t i = 0; i < Size; i++) {
		while (READ_BIT(USART->SR, USART_SR_TXE) == 0) ; //Ждем, пока линия не освободится
		USART->DR = *data++; //Кидаем данные  
	}	
}



int main(void) {
	CMSIS_Debug_init();
	CMSIS_RCC_SystemClock_72MHz();
	CMSIS_SysTick_Timer_init();
	CMSIS_PC13_OUTPUT_Push_Pull_init();
	
	
	
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
	
	MODIFY_REG(USART1->BRR, USART_BRR_DIV_Mantissa_Msk, 0x27 << USART_BRR_DIV_Mantissa_Pos);
	MODIFY_REG(USART1->BRR, USART_BRR_DIV_Fraction_Msk, 0x1 << USART_BRR_DIV_Fraction_Pos);
	
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
	USART1->CR3 = 0;
	USART1->GTPR = 0;

	NVIC_EnableIRQ(USART1_IRQn); //Включим прерывания по USART
	
	
	
	while (1) {
		
		/*----------Простая передача одного байта----------*/
		/*while (READ_BIT(USART1->SR, USART_CR1_TXEIE) == 0) ;
		USART1->DR = 0x55;
		Delay_ms(100);*/
		/*----------Простая передача одного байта----------*/
		
		
		//CMSIS_USART_Transmit(USART1, A , sizeof A);
		sprintf((char*)husart1.tx_buffer, "Hello world\r\n");
		CMSIS_USART_Transmit(USART1, husart1.tx_buffer, strlen((char*)husart1.tx_buffer));
		Delay_ms(1000);
	}
}



void USART1_IRQHandler(void) {
	
	if (READ_BIT(USART1->SR, USART_SR_RXNE)) {
		//Если пришли данные по USART
		husart1.rx_buffer[husart1.rx_counter] = USART1->DR; //Считаем данные в соответствующую ячейку в rx_buffer
		husart1.rx_counter++; //Увеличим счетчик принятых байт на 1
	}
	if (READ_BIT(USART1->SR, USART_SR_IDLE)) {
		//Если прилетел флаг IDLE
		USART1->DR; //Сбросим флаг IDLE
		husart1.rx_len = husart1.rx_counter; //Узнаем, сколько байт получили
		CMSIS_USART_Transmit(USART1, husart1.rx_buffer, husart1.rx_counter); //Отправим в порт то, что прилетело для проверки.
		husart1.rx_counter = 0; //сбросим счетчик приходящих данных
	}
}

