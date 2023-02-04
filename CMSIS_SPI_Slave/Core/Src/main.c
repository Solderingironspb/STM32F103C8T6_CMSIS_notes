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
		//���� ���������� ������ �� EXTI4
			//NSS �������� � ��������� 1
			spi_rx_len = spi_rx_counter;
			spi_rx_counter = 0;
		SET_BIT(EXTI->PR, EXTI_PR_PR4); //������� ����
	}
}

int main(void) {
	CMSIS_Debug_init();
	CMSIS_RCC_SystemClock_72MHz();
	CMSIS_SysTick_Timer_init();
	CMSIS_SPI1_init();
	/*�������� ������� �� CS*/
	//PA4 - NSS
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //������ ������������ ����� A
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE4, 0b00 << GPIO_CRL_MODE4_Pos); //��������� GPIOA Pin 4 �� ����
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF4, 0b01 << GPIO_CRL_CNF4_Pos); //��������� GPIOA Pin 4 Input floating
	/*�������� EXTI ��� PA4*/
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN); //�������� ������������ ��� �������������� �������
	MODIFY_REG(AFIO->EXTICR[1], AFIO_EXTICR2_EXTI4, AFIO_EXTICR2_EXTI4_PA); //AFIO_EXTICR2, EXTI4, ������ ���� A.
	SET_BIT(EXTI->IMR, EXTI_IMR_MR4); //���. ���������� ��� 4 �����.
	SET_BIT(EXTI->RTSR, EXTI_RTSR_RT4); //���������� ����� ����������� �� �������� ������� �� 0 � 1
	NVIC_EnableIRQ(EXTI4_IRQn); //������� ���������� �� EXTI4
	
	while (1) {
		
    	
	}
}