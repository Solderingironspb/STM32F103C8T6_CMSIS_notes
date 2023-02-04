#include "main.h"

#define NSS_ON  GPIOA->BSRR = GPIO_BSRR_BR4;
#define NSS_OFF GPIOA->BSRR = GPIO_BSRR_BS4;

uint8_t spi_tx_buffer[6] = { 4, 8, 15, 16, 23, 42 };
uint8_t spi_rx_buffer[20] = {0, };



int main(void) {
    CMSIS_Debug_init();
    CMSIS_RCC_SystemClock_72MHz();
    CMSIS_SysTick_Timer_init();
    CMSIS_SPI1_init();
    
    //PA4 - NSS
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Запуск тактирования порта A
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE4, 0b10 << GPIO_CRL_MODE4_Pos); //Настройка GPIOA Pin 4 на выход со максимальной скоростью в 50 MHz
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF4, 0b00 << GPIO_CRL_CNF4_Pos); //Настройка GPIOA Pin 4 на выход в режиме Push-Pull
	NSS_OFF;
	
	while (1) {
    	
		NSS_ON;
		CMSIS_SPI_Data_Transmit_8BIT(SPI1, spi_tx_buffer, 6, 100);
		NSS_OFF;
		Delay_ms(100);
		NSS_ON;
		CMSIS_SPI_Data_Receive_8BIT(SPI1, spi_rx_buffer, 6, 100);
		NSS_OFF;
		Delay_ms(100);
	}
}