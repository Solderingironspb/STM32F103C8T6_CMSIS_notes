#include "main.h"


/*----------Макросы----------*/
#define NSS_PORT GPIOA  //Порт ножки CS
#define NSS_PIN  4      //Пин ножки CS

//NSS_ACTIVE_LOW
#define NSS_ON  NSS_PORT->BSRR = (1 << (NSS_PIN + 16)) //CS вкл. 
#define NSS_OFF NSS_PORT->BSRR = (1 << NSS_PIN); //CS выкл.
/*----------Макросы----------*/

uint8_t spi_tx_buffer[6] = { 4, 8, 15, 16, 23, 42 };
uint8_t spi_rx_buffer[6] = {0,};

int main(void) {
    CMSIS_Debug_init();
    CMSIS_RCC_SystemClock_72MHz();
    CMSIS_SysTick_Timer_init();
    CMSIS_SPI1_init();
    
    //PA4 - NSS
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Запуск тактирования порта A
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE4, 0b10 << GPIO_CRL_MODE4_Pos); //Настройка GPIOA Pin 4 на выход со максимальной скоростью в 50 MHz
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF4, 0b00 << GPIO_CRL_CNF4_Pos); //Настройка GPIOA Pin 4 на выход в режиме Push-Pull
    
	while (1) {
    	
    	NSS_ON;
    	CMSIS_SPI_Data_Transmit_8BIT(SPI1, spi_tx_buffer, 6, 100);
    	NSS_OFF;
    	
		for (int i = 0; i < 6; i++) {
			spi_tx_buffer[i]++;
		}
		
		Delay_ms(1000);
		
    	
    	
	}
}