#include "main.h"
#include "stm32f103xx_CMSIS.h"
volatile uint32_t Counter_ADC = 0;
volatile uint32_t Counter_DMA = 0;
volatile uint16_t ADC_Data[2] = { 0, };

void ADC1_2_IRQHandler(void) {
	/*This bit is set by hardware at the end of a group channel conversion (regular or injected). It is
    * cleared by software or by reading the ADC_DR.
    * 0: Conversion is not complete
    * 1: Conversion complete*/
	Counter_ADC++;
	if (READ_BIT(ADC1->SR, ADC_SR_EOC)) {
		ADC1->DR; //Читаем канал, чтоб сбросить флаг
	}
	
}
void DMA1_Channel1_IRQHandler(void) {
	if (READ_BIT(DMA1->ISR, DMA_ISR_TCIF1)) {
		SET_BIT(DMA1->IFCR, DMA_IFCR_CGIF1); //Сбросим глобальный флаг.
		Counter_DMA++;
	} else if (READ_BIT(DMA1->ISR, DMA_ISR_TEIF1)) {
		/*Здесь можно сделать какой-то обработчик ошибок*/
		SET_BIT(DMA1->IFCR, DMA_IFCR_CGIF1); //Сбросим глобальный флаг.
	}
}

int main(void) {
	CMSIS_Debug_init();
	CMSIS_RCC_SystemClock_72MHz();
	CMSIS_SysTick_Timer_init();
	CMSIS_PC13_OUTPUT_Push_Pull_init();
	
	/* Настройка DMA
	*  Внимание:
	*  Порядок настройки DMA хорошо описан на странице 278 "Channel configuration procedure"*/
	 
	SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN); //Включение тактирования DMA1
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR); //Задаем адрес периферийного устройства
	DMA1_Channel1->CMAR = (uint32_t)ADC_Data; //Задаем адрес в памяти, куда будем кидать данные.
	DMA1_Channel1->CNDTR = 2; //Настроим количество данных для передачи. После каждого периферийного события это значение будет уменьшаться.
	MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_PL_Msk, 0b00 << DMA_CCR_PL_Pos); //Зададим приоритет канала на высокий
	CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_DIR); //Чтение будем осуществлять с периферии
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_CIRC); //Настроим DMA в Circular mode
	MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_PSIZE_Msk, 0b01 << DMA_CCR_PSIZE_Pos); //Размер данных периферийного устройства 16 бит
	MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_MSIZE_Msk, 0b01 << DMA_CCR_MSIZE_Pos); //Размер данных в памяти 16 бит
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_TCIE); //Включим прерывание по полной передаче
	CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_HTIE); //Отключим прерывание по половинной передаче
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_TEIE); //Включим прерывание по ошибке передачи.
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_MINC); //Включим инкрементирование памяти
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_EN); //DMA ON
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN); //Включение тактирования ADC1.
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Включение тактирования порта А.
	
	/*Настройка ножек PA0 и PA1 на аналоговый вход*/
	/*Pin PA0 - Analog*/
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF0_Msk, 0b00 << GPIO_CRL_CNF0_Pos);
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE0_Msk, 0b00 << GPIO_CRL_MODE0_Pos);
	
	/*Pin PA1 - Analog*/
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF1_Msk, 0b00 << GPIO_CRL_CNF1_Pos);
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE1_Msk, 0b00 << GPIO_CRL_MODE1_Pos);
	
	
	/*11.12 ADC registers(страница 237)*/
	//11.12.2 ADC control register 1 (ADC_CR1)(страница 238)
	
	CLEAR_BIT(ADC1->CR1, ADC_CR1_EOCIE); //EOC interrupt enabled/disabled. An interrupt is generated when the EOC bit is set
	CLEAR_BIT(ADC1->CR1, ADC_CR1_AWDIE); //Analog watchdog interrupt disabled
	CLEAR_BIT(ADC1->CR1, ADC_CR1_JEOCIE); //JEOC interrupt disabled
	SET_BIT(ADC1->CR1, ADC_CR1_SCAN); //Scan mode enabled
	
	/* Примечание:
	* Прерывание EOC или JEOC генерируется только в конце преобразования последнего канала, 
	* если установлен соответствующий бит EOCIE или JEOCIE.*/
	
	CLEAR_BIT(ADC1->CR1, ADC_CR1_AWDSGL); //Analog watchdog enabled on all channels
	CLEAR_BIT(ADC1->CR1, ADC_CR1_JAUTO); //Automatic injected group conversion disabled
	CLEAR_BIT(ADC1->CR1, ADC_CR1_DISCEN); //Discontinuous mode on regular channels disabled
	CLEAR_BIT(ADC1->CR1, ADC_CR1_JDISCEN); //Discontinuous mode on injected channels disabled
	MODIFY_REG(ADC1->CR1, ADC_CR1_DUALMOD_Msk, 0b0110 << ADC_CR1_DUALMOD_Pos); //0110: Regular simultaneous mode only
	CLEAR_BIT(ADC1->CR1, ADC_CR1_JAWDEN); //Analog watchdog disabled on injected channels
	CLEAR_BIT(ADC1->CR1, ADC_CR1_AWDEN); //Analog watchdog disabled on regular channels
	
	/*11.12.3 ADC control register 2 (ADC_CR2)(страница 240)*/
	
	SET_BIT(ADC1->CR2, ADC_CR2_ADON); //Запустить АЦП
	
	/* Примечание:
	* Если в этот же момент изменяется какой-либо другой бит в этом регистре, 
	* кроме ADON, то конверсия не запускается. 
	* Это сделано для предотвращения ошибочного преобразования.*/
	
	SET_BIT(ADC1->CR2, ADC_CR2_CONT); //Continuous conversion mode(непрерывные преобразования) 
	SET_BIT(ADC1->CR2, ADC_CR2_CAL); //Enable calibration
    
	/*Примечание:
     * Этот бит устанавливается программой для запуска калибровки. 
     * Он сбрасывается аппаратно после завершения калибровки.*/
	
	while (READ_BIT(ADC1->CR2, ADC_CR2_CAL)) ;//Подождем окончания калибровки
	Delay_ms(1); //Задержка для GD32F103CBT6. На STM32F103CBT6 работает и так. 
	
	SET_BIT(ADC1->CR2, ADC_CR2_DMA); //DMA включен
	CLEAR_BIT(ADC1->CR2, ADC_CR2_ALIGN); //Выравнивание по правому краю
	MODIFY_REG(ADC1->CR2, ADC_CR2_EXTSEL_Msk, 0b111 << ADC_CR2_EXTSEL_Pos); //Запускать преобразование программно
	CLEAR_BIT(ADC1->CR2, ADC_CR2_EXTTRIG); //Conversion on external event disabled
	//SET_BIT(ADC1->CR2, ADC_CR2_SWSTART); //Начать преобразование
	SET_BIT(ADC1->CR2, ADC_CR2_TSVREFE); //Temperature sensor and VREFINT channel enabled


	/*Note: 
	 * ADC1 analog Channel16 and Channel 17 are internally connected to the temperature
     * sensor and to VREFINT, respectively.
     * ADC2 analog input Channel16 and Channel17 are internally connected to VSS.
     * ADC3 analog inputs Channel14, Channel15, Channel16 and Channel17 are connected to VSS.*/
	
	// 11.12.5 ADC sample time register 2 (ADC_SMPR2)(страница 245)
	MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP0_Msk, 0b111 << ADC_SMPR2_SMP0_Pos); //239.5 cycles 
	MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP1_Msk, 0b111 << ADC_SMPR2_SMP1_Pos); //239.5 cycles 
	MODIFY_REG(ADC1->SMPR1, ADC_SMPR1_SMP17_Msk, 0b111 << ADC_SMPR1_SMP17_Pos); //239.5 cycles 
	
	// 11.12.9 ADC regular sequence register 1 (ADC_SQR1)(страница 247)
	MODIFY_REG(ADC1->SQR1, ADC_SQR1_L_Msk, 0b0001 << ADC_SQR1_L_Pos); //2 преобразования
	
	// 11.12.11 ADC regular sequence register 3 (ADC_SQR3)(страница 249)
	MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ1_Msk, 0 << ADC_SQR3_SQ1_Pos);
	MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ2_Msk, 17 << ADC_SQR3_SQ2_Pos);
	//NVIC_EnableIRQ(ADC1_IRQn); //Разрешить прерывания по АЦП
	
	
	//SET_BIT(ADC1->CR2, ADC_CR2_SWSTART); //Начать преобразование. Не нужно запускать, если circular mode.
	
	while (1) {
		GPIOC->BSRR = GPIO_BSRR_BR13;
		Delay_ms(100);
		GPIOC->BSRR = GPIO_BSRR_BS13;
		Delay_ms(100);
	}
} 
