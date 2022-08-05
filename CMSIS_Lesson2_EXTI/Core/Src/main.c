#include "main.h"
#include <stdbool.h>

bool PA1_Pin_state;
bool PB0_Pin_state;

volatile uint32_t counter_0 = 0;
volatile uint32_t counter_1 = 0;


void EXTI0_IRQHandler(void) {
	SET_BIT(EXTI->PR, EXTI_PR_PR0);
	counter_0++;
}

void EXTI1_IRQHandler(void) {
	SET_BIT(EXTI->PR, EXTI_PR_PR1);
	counter_1++;
}

int main(void) {
	CMSIS_Debug_init(); //Serial wire
	CMSIS_RCC_SystemClock_72MHz(); //��������� ������� �� 72MHz
	CMSIS_SysTick_Timer_init();  //��������� ���������� �������

	/*�������� ����� A1 � B0 �� ����*/
	
	/*��������� PA1 �� ���� - floating*/
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //��������� ������������ ����� A
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF1_Msk, 0b01 << GPIO_CRL_CNF1_Pos); //01: Floating input (reset state)
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE1_Msk, 0b00 << GPIO_CRL_MODE1_Pos); //00: Input mode (reset state)
	
	/*��������� PB0 �� ���� - Input with pull-up*/
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN); //��������� ������������ ����� B
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF0_Msk, 0b10 << GPIO_CRL_CNF0_Pos);  //10: Input with pull-up / pull-down
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_MODE0_Msk, 0b00 << GPIO_CRL_MODE0_Pos); //00: Input mode (reset state)
	GPIOB->BSRR = GPIO_BSRR_BS0; //Input with pull-up
	
	//��������� EXTI ��� ����� PA1 � PB0
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN); //�������� ������������ �������������� �������
	//������ � PB0:
	MODIFY_REG(AFIO->EXTICR[0], AFIO_EXTICR1_EXTI0_Msk, 0b0001 << AFIO_EXTICR1_EXTI0_Pos); //EXTI0, ���� B
	//PA1:
	MODIFY_REG(AFIO->EXTICR[0], AFIO_EXTICR1_EXTI1_Msk, 0b0000 << AFIO_EXTICR1_EXTI1_Pos); //EXTI1, ���� A
	
	SET_BIT(EXTI->IMR, EXTI_IMR_MR0); //��������� ���������� � EXTI0
	SET_BIT(EXTI->IMR, EXTI_IMR_MR1); //��������� ���������� � EXTI1
	
	/*��� ������� �������� ���������� ��� PB0 �� ������, � PA1 �� ������ � ����� �������*/
	SET_BIT(EXTI->RTSR, EXTI_RTSR_TR0); //EXTI0 PB0 Rising on
	SET_BIT(EXTI->RTSR, EXTI_RTSR_TR1); //EXTI1 PA1 Rising on
	CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_TR0); //EXTI0 PB0 Falling off
	SET_BIT(EXTI->FTSR, EXTI_FTSR_TR1); //EXTI1 PA1 Falling on
	
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	
	
	while (1) {	
		
		//PA1_Pin_state = GPIOA->IDR & GPIO_IDR_IDR1;
		PA1_Pin_state = READ_BIT(GPIOA->IDR, GPIO_IDR_IDR1);
		//PB0_Pin_state = GPIOB->IDR & GPIO_IDR_IDR0;
		PB0_Pin_state = READ_BIT(GPIOB->IDR, GPIO_IDR_IDR0);
		
		Delay_ms(100);
				
	}

}