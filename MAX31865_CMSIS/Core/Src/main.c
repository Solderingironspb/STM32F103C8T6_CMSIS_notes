#include "main.h"
#include "MAX31865.h"

/*-----------------------------------------Глобальные переменные---------------------------------------------*/
extern float MAX31865_PT100_R; //Глобальная переменная, определяющая сопротивление датчика PT100
extern float MAX31865_PT100_T; //Глобальная переменная, определяющая температуру датчика PT100
extern float MAX31865_Correction_additive; //Калибровка смещения
extern float MAX31865_Correction_multiplicative; //Калибровка наклона
extern bool MAX31865_Sensor_Error; //Глобальная переменная, определяющая неисправность датчика PT100
/*-----------------------------------------Глобальные переменные---------------------------------------------*/


int main(void) {
    CMSIS_Debug_init();
    CMSIS_RCC_SystemClock_72MHz();
    CMSIS_SysTick_Timer_init();
    CMSIS_SPI1_init();
    
    //PA4 - NSS
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Запуск тактирования порта A
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE4, 0b10 << GPIO_CRL_MODE4_Pos); //Настройка GPIOA Pin 4 на выход со максимальной скоростью в 50 MHz
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF4, 0b00 << GPIO_CRL_CNF4_Pos); //Настройка GPIOA Pin 4 на выход в режиме Push-Pull
    
    MAX31865_Init(SPI1, 3); //3 проводное подключение
    
	while (1) {
    	
    	MAX31865_PT100_R = (MAX31865_Get_Resistance(SPI1) * MAX31865_Correction_multiplicative) + MAX31865_Correction_additive; //Значение сопротивления датчика PT100
    	MAX31865_PT100_T = MAX31865_Get_Temperature(MAX31865_PT100_R); //Рассчет температуры датчика PT100
    	Delay_ms(100);
	}
}