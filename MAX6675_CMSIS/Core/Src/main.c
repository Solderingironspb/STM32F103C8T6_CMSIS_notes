#include "main.h"
#include "MAX6675.h"

extern float MAX6675_Temperature; //Измеренная температура
extern float MAX6675_Correction_additive; //Калибровка смещения
extern float MAX6675_Correction_multiplicative; //Калибровка наклона

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

        MAX6675_Temperature = MAX6675_GetTemperature(SPI1);  
        Delay_ms(220);
    	
    }
}