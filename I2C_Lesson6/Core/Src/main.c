#include "main.h"
#include "stm32f103xx_CMSIS.h"

extern volatile uint32_t Timeout_counter_ms;

bool flag_I2C_Adress_Device_ACK;
bool flag_I2C_Data_Transmit;
bool flag_I2C_Data_Receive;
uint8_t i2c_tx_buffer[32] = { 0, 0, 4, 8, 15, 16, 23, 42 };
uint8_t i2c_rx_buffer[32] = { 0, };

uint8_t i2c_tx_buffer_2[32] = { 1, 2, 3, 4, 5, 6 };

void CMSIS_I2C_Reset(void);
void CMSIS_I2C1_Init(void);
bool CMSIS_I2C_Adress_Device_Scan(I2C_TypeDef *I2C, uint8_t Adress_Device, uint32_t Timeout_ms);
bool CMSIS_I2C_Data_Transmit(I2C_TypeDef *I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool CMSIS_I2C_Data_Receive(I2C_TypeDef *I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool CMSIS_I2C_MemWrite(I2C_TypeDef *I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool CMSIS_I2C_MemRead(I2C_TypeDef *I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);

int main(void) {
	CMSIS_Debug_init();
	CMSIS_RCC_SystemClock_72MHz();
	CMSIS_SysTick_Timer_init();
	CMSIS_I2C1_Init();	
	Delay_ms(1000);
	
	//flag_I2C_Adress_Device_ACK = CMSIS_I2C_Adress_Device_Scan(I2C1, 0x27, 100);
	//Delay_ms(10);
	//flag_I2C_Data_Transmit = CMSIS_I2C_Data_Transmit(I2C1, 0x50, i2c_tx_buffer, 8, 100);
	//flag_I2C_Data_Transmit = CMSIS_I2C_Data_Transmit(I2C1, 0x50, i2c_tx_buffer, 2, 100);
	//Delay_ms(5);
	//flag_I2C_Data_Receive = CMSIS_I2C_Data_Receive(I2C1, 0x50, i2c_rx_buffer, 6, 100);
	//flag_I2C_Data_Transmit = CMSIS_I2C_MemWrite(I2C1, 0x50, 6, 2, i2c_tx_buffer_2, 6, 100);
	flag_I2C_Data_Receive = CMSIS_I2C_MemRead(I2C1, 0x50, 6, 2, i2c_rx_buffer, 6, 100);
	
	while (1) {	
	}
}


void CMSIS_I2C_Reset(void) {
	//п.п. 26.6.1 I2C Control register 1 (I2C_CR1) (стр. 772)
	SET_BIT(I2C1->CR1, I2C_CR1_SWRST); //: I2C Peripheral not under reset
	while (READ_BIT(I2C1->CR1, I2C_CR1_SWRST) == 0) ;
	CLEAR_BIT(I2C1->CR1, I2C_CR1_SWRST); //: I2C Peripheral not under reset
	while (READ_BIT(I2C1->CR1, I2C_CR1_SWRST)) ;
	/* Примечание: Этот бит можно использовать для повторной инициализации 
	 * периферийного устройства после ошибки или заблокированного состояния.
	 * Например, если бит BUSY установлен и остается заблокированным из-за сбоя на шине,
	 * бит SWRST можно использовать для выхода из этого состояния.*/
}



/**
 *************************************************************************************
 *  @breif Функция инициализации шины I2C1. Sm.
 *************************************************************************************
 */

void CMSIS_I2C1_Init(void) {
	//Настройки тактирования
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN); //Запуск тактирование порта B
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN); //Запуск тактирования альтернативных функций
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN); //Запуск тактирования I2C1
	
	//Настройки ножек SDA и SCL
	//PB7 SDA (I2C Data I/O) Alternate function open drain
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF7_Msk, 0b11 << GPIO_CRL_CNF7_Pos); //Alternate function open drain
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_MODE7_Msk, 0b11 << GPIO_CRL_MODE7_Pos); //Maximum output speed 50 MHz
	//PB6 SCL (I2C clock) Alternate function open drain
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF6_Msk, 0b11 << GPIO_CRL_CNF6_Pos); //Alternate function open drain
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_MODE6_Msk, 0b11 << GPIO_CRL_MODE6_Pos); //Maximum output speed 50 MHz
	
	//26.6 I2C registers( См. Reference Manual стр. 772)
	
	//п.п. 26.6.1 I2C Control register 1 (I2C_CR1) (стр. 772)
	CMSIS_I2C_Reset();
	
	/*Это все для инита не нужно. После сброса итак будет в 0. */
	CLEAR_BIT(I2C1->CR1, I2C_CR1_ALERT); //Releases SMBA pin high.Alert Response Address Header followed by NACK
	CLEAR_BIT(I2C1->CR1, I2C_CR1_PEC); //No PEC transfer
	CLEAR_BIT(I2C1->CR1, I2C_CR1_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
	CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK); //No acknowledge returned
	CLEAR_BIT(I2C1->CR1, I2C_CR1_STOP); //No Stop generation
	CLEAR_BIT(I2C1->CR1, I2C_CR1_START); //No Start generation
	CLEAR_BIT(I2C1->CR1, I2C_CR1_NOSTRETCH); //Clock stretching enabled
	CLEAR_BIT(I2C1->CR1, I2C_CR1_ENGC); //General call disabled. Address 00h is NACKed.
	CLEAR_BIT(I2C1->CR1, I2C_CR1_ENPEC); //PEC calculation disabled
	CLEAR_BIT(I2C1->CR1, I2C_CR1_ENARP); //ARP disable
	CLEAR_BIT(I2C1->CR1, I2C_CR1_SMBTYPE); //SMBus Device
	CLEAR_BIT(I2C1->CR1, I2C_CR1_SMBUS); //I2C mode
	
	//п.п. 26.6.2 I2C Control register 2(I2C_CR2)(стр.774)
	CLEAR_BIT(I2C1->CR2, I2C_CR2_LAST); //Next DMA EOT is not the last transfer
	CLEAR_BIT(I2C1->CR2, I2C_CR2_DMAEN); //DMA requests disabled
	CLEAR_BIT(I2C1->CR2, I2C_CR2_ITBUFEN); //TxE = 1 or RxNE = 1 does not generate any interrupt.
	CLEAR_BIT(I2C1->CR2, I2C_CR2_ITEVTEN); //Event interrupt disabled
	CLEAR_BIT(I2C1->CR2, I2C_CR2_ITERREN); //Error interrupt disabled
	MODIFY_REG(I2C1->CR2, I2C_CR2_FREQ_Msk, 36 << I2C_CR2_FREQ_Pos); //f PCLK1 = 36 Мгц
		
	//п.п. 26.6.3 I2C Own address register 1(I2C_OAR1)(стр.776)
	I2C1->OAR1 = 0;
	//п.п. 26.6.4 I2C Own address register 1(I2C_OAR2)(стр.776)
	I2C1->OAR2 = 0;
	
	//п.п. 26.6.8 I2C Clock control register (I2C_CCR)(стр.781)
	//CLEAR_BIT(I2C1->CCR, I2C_CCR_FS); //Standard mode I2C
	SET_BIT(I2C1->CCR, I2C_CCR_FS); //Fast mode I2C

	CLEAR_BIT(I2C1->CCR, I2C_CCR_DUTY); //Fm mode tlow/thigh = 2
	//SET_BIT(I2C1->CCR, I2C_CCR_DUTY); //Fm mode tlow/thigh = 16/9 (see CCR)
	
	//Расчет CCR. Смотри примеры расчета
	//MODIFY_REG(I2C1->CCR, I2C_CCR_CCR_Msk, 180 << I2C_CCR_CCR_Pos); //для Sm mode
	MODIFY_REG(I2C1->CCR, I2C_CCR_CCR_Msk, 30 << I2C_CCR_CCR_Pos); //для Fm mode. DUTY 0.
	//MODIFY_REG(I2C1->CCR, I2C_CCR_CCR_Msk, 4 << I2C_CCR_CCR_Pos); //для Fm mode. DUTY 1.
	
	//п.п. 26.6.9 I2C TRISE register (I2C_TRISE)(стр. 782)
	//MODIFY_REG(I2C1->TRISE, I2C_TRISE_TRISE_Msk, 37 << I2C_TRISE_TRISE_Pos); //для Sm mode
	MODIFY_REG(I2C1->TRISE, I2C_TRISE_TRISE_Msk, 12 << I2C_TRISE_TRISE_Pos); //для Fm mode
	
	SET_BIT(I2C1->CR1, I2C_CR1_PE); //I2C1 enable
}


/**
 *************************************************************************************
 *  @breif Функция сканирования устройства по заданному 7-битному адресу
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства         
 *  @retval  Возвращает статус true - если устройство по заданному адресу отозвалось, 
 *           false - если устройство по заданному адресу не отвечает
 *************************************************************************************
 */
bool CMSIS_I2C_Adress_Device_Scan(I2C_TypeDef *I2C, uint8_t Adress_Device, uint32_t Timeout_ms) {
	
	/*-------------------Проверка занятости шины-------------------*/
	if (READ_BIT(I2C->SR2, I2C_SR2_BUSY)) {
		//Если шина занята
		
		if ((READ_BIT(GPIOB->IDR, GPIO_IDR_IDR6)) && (READ_BIT(GPIOB->IDR, GPIO_IDR_IDR7))) {
			//Если линия на самом деле свободна, а BUSY висит
			CMSIS_I2C_Reset(); //ресет
			CMSIS_I2C1_Init(); //повторная инициализация
		} 
		
		if (READ_BIT(I2C->SR2, I2C_SR2_MSL)) {
			//Если стоит статус, что мы в мастере
			SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправим сигнал STOP
		} 
		
		if (I2C->CR1 != 1) {
			//Если в CR1 что-то лишнее, то перезагрузим I2C
			CLEAR_BIT(I2C->CR1, I2C_CR1_PE);
			SET_BIT(I2C->CR1, I2C_CR1_PE);
		} 	
		
		return false;	
	}
	/*-------------------Проверка занятости шины-------------------*/
	
	CLEAR_BIT(I2C->CR1, I2C_CR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
	SET_BIT(I2C->CR1, I2C_CR1_START); //Отправляем сигнал START
	
	Timeout_counter_ms = Timeout_ms;
	while (READ_BIT(I2C->SR1, I2C_SR1_SB) == 0) {
		//Ожидаем до момента, пока не сработает Start condition generated
	
		if (!Timeout_counter_ms) {
			return false;
		}
		
	} 
	//ВНИМАНИЕ!
	/* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью данных в регистр DR или когда PE=0*/
	I2C->SR1;
	I2C->DR = (Adress_Device << 1); //Адрес + Write
	
	Timeout_counter_ms = Timeout_ms;
	while ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C->SR1, I2C_SR1_ADDR) == 0)) {
		//Ждем, пока адрес отзовется
		
		if (!Timeout_counter_ms) {
			return false;
		}
		
	}
	
	if (READ_BIT(I2C->SR1, I2C_SR1_ADDR)) {
		//Если устройство отозвалось
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправляем сигнал STOP
		/*Сброс бита ADDR производится чтением SR1, а потом SR2*/
		I2C->SR1;
		I2C->SR2;
		return true;
	} else {
		//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправляем сигнал STOP
		CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
		return false;
	}
}



/**
 **************************************************************************************************
 *  @breif Функция передачи данных по I2C
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства      
 *  @param  *data - Данные, которые будем отправлять
 *  @param  Size_data - Размер, сколько байт будем отправлять.
 *  @retval  Возвращает статус отправки данных. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_I2C_Data_Transmit(I2C_TypeDef *I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	
	/*-------------------Проверка занятости шины-------------------*/
	if (READ_BIT(I2C->SR2, I2C_SR2_BUSY)) {
		//Если шина занята
		
		if ((READ_BIT(GPIOB->IDR, GPIO_IDR_IDR6)) && (READ_BIT(GPIOB->IDR, GPIO_IDR_IDR7))) {
			//Если линия на самом деле свободна, а BUSY висит
			CMSIS_I2C_Reset(); //ресет
			CMSIS_I2C1_Init(); //повторная инициализация
		} 
		
		if (READ_BIT(I2C->SR2, I2C_SR2_MSL)) {
			//Если стоит статус, что мы в мастере
			SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправим сигнал STOP
		} 
		
		if (I2C->CR1 != 1) {
			//Если в CR1 что-то лишнее, то перезагрузим I2C
			CLEAR_BIT(I2C->CR1, I2C_CR1_PE);
			SET_BIT(I2C->CR1, I2C_CR1_PE);
		} 	
		
		return false;	
	}
	/*-------------------Проверка занятости шины-------------------*/
	
	CLEAR_BIT(I2C->CR1, I2C_CR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
	SET_BIT(I2C->CR1, I2C_CR1_START); //Стартуем.
	
	Timeout_counter_ms = Timeout_ms;
	while (READ_BIT(I2C->SR1, I2C_SR1_SB) == 0) {
		//Ожидаем до момента, пока не сработает Start condition generated
	
		if (!Timeout_counter_ms) {
			return false;
		}
		
	} 
	//ВНИМАНИЕ!
	/* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
	I2C->SR1;
	I2C->DR = (Adress_Device << 1); //Адрес + Write
	
	Timeout_counter_ms = Timeout_ms;
	while ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C->SR1, I2C_SR1_ADDR) == 0)) {
		//Ждем, пока адрес отзовется
		
		if (!Timeout_counter_ms) {
			return false;
		}
		
	}
	
	if (READ_BIT(I2C->SR1, I2C_SR1_ADDR)) {
		//Если устройство отозвалось, сбросим бит ADDR
		/*Сброс бита ADDR производится чтением SR1, а потом SR2*/
		I2C->SR1;
		I2C->SR2;
		
		/*Отправим данные*/
		for (uint16_t i = 0; i < Size_data; i++) {
			I2C->DR = *(data + i); //Запись байта
			while (READ_BIT(I2C->SR1, I2C_SR1_TXE) == 0) {
				//Ждем, пока данные загрузятся в регистр сдвига.
				
				if ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 1)) {
					//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
					SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
					CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
					return false;
				}
			} 
		}
		
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем

		return true;
	
	} else {
		//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
		CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
		
		return false;
	}
}


/**
 **************************************************************************************************
 *  @breif Функция приема данных по I2C
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства      
 *  @param  *data - Куда будем записывать принятые данные
 *  @param  Size_data - Размер, сколько байт будем принимать.
 *  @retval  Возвращает статус приема данных. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_I2C_Data_Receive(I2C_TypeDef *I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	
	/*-------------------Проверка занятости шины-------------------*/
	if (READ_BIT(I2C->SR2, I2C_SR2_BUSY)) {
		//Если шина занята
		
		if ((READ_BIT(GPIOB->IDR, GPIO_IDR_IDR6)) && (READ_BIT(GPIOB->IDR, GPIO_IDR_IDR7))) {
			//Если линия на самом деле свободна, а BUSY висит
			CMSIS_I2C_Reset(); //ресет
			CMSIS_I2C1_Init(); //повторная инициализация
		} 
		
		if (READ_BIT(I2C->SR2, I2C_SR2_MSL)) {
			//Если стоит статус, что мы в мастере
			SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправим сигнал STOP
		} 
		
		if (I2C->CR1 != 1) {
			//Если в CR1 что-то лишнее, то перезагрузим I2C
			CLEAR_BIT(I2C->CR1, I2C_CR1_PE);
			SET_BIT(I2C->CR1, I2C_CR1_PE);
		} 	
		
		return false;	
	}
	/*-------------------Проверка занятости шины-------------------*/
	
	CLEAR_BIT(I2C->CR1, I2C_CR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
	SET_BIT(I2C->CR1, I2C_CR1_START); //Стартуем.
	
	Timeout_counter_ms = Timeout_ms;
	while (READ_BIT(I2C->SR1, I2C_SR1_SB) == 0) {
		//Ожидаем до момента, пока не сработает Start condition generated
	
		if (!Timeout_counter_ms) {
			return false;
		}
		
	} 
	//ВНИМАНИЕ!
	/* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
	I2C->SR1;
	I2C->DR = (Adress_Device << 1 | 1); //Адрес + команда Read
		
	Timeout_counter_ms = Timeout_ms;
	while ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C->SR1, I2C_SR1_ADDR) == 0)) {
		//Ждем, пока адрес отзовется
		
		if (!Timeout_counter_ms) {
			return false;
		}
		
	}
		
	if (READ_BIT(I2C->SR1, I2C_SR1_ADDR)) {
		//Если устройство отозвалось, сбросим бит ADDR
		/*Сброс бита ADDR производится чтением SR1, а потом SR2*/
		I2C->SR1;
		I2C->SR2;
			
		/*Прочтем данные*/
		for (uint16_t i = 0; i < Size_data; i++) {
			if (i < Size_data - 1) {
				SET_BIT(I2C->CR1, I2C_CR1_ACK); //Если мы хотим принять следующий байт, то отправляем ACK
				
				Timeout_counter_ms = Timeout_ms;
				while (READ_BIT(I2C->SR1, I2C_SR1_RXNE) == 0) {
					//Ожидаем, пока в сдвиговом регистре появятся данные
					if (!Timeout_counter_ms) {
						return false;
					}
				}
				
				*(data + i) = I2C->DR; //Чтение байта
			} else {
				CLEAR_BIT(I2C->CR1, I2C_CR1_ACK); //Если мы знаем, что следующий принятый байт будет последним, то отправим NACK
		
				SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
				Timeout_counter_ms = Timeout_ms;
				while (READ_BIT(I2C->SR1, I2C_SR1_RXNE) == 0) {
					//Ожидаем, пока в сдвиговом регистре появятся данные
					if (!Timeout_counter_ms) {
						return false;
					}
				}
				*(data + i) = I2C->DR; //Чтение байта
			}
		} return true;
			
	} else {
		//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
		CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
		return false;
	}
		
} 


/**
 **************************************************************************************************
 *  @breif Функция записи в память по указанному адресу
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства      
 *  @param  Adress_data - Адрес в памяти, куда будем записывать данные
 *  @param  Size_adress - Размер адреса в байтах. Пример: 1 - 8 битный адрес. 2 - 16 битный адрес.
 *  @param  *data - Данные, которые будем записывать
 *  @param  Size_data - Размер, сколько байт будем записывать.
 *  @retval  Возвращает статус записи. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_I2C_MemWrite(I2C_TypeDef *I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	
	/*-------------------Проверка занятости шины-------------------*/
	if (READ_BIT(I2C->SR2, I2C_SR2_BUSY)) {
		//Если шина занята
		
		if ((READ_BIT(GPIOB->IDR, GPIO_IDR_IDR6)) && (READ_BIT(GPIOB->IDR, GPIO_IDR_IDR7))) {
			//Если линия на самом деле свободна, а BUSY висит
			CMSIS_I2C_Reset(); //ресет
			CMSIS_I2C1_Init(); //повторная инициализация
		} 
		
		if (READ_BIT(I2C->SR2, I2C_SR2_MSL)) {
			//Если стоит статус, что мы в мастере
			SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправим сигнал STOP
		} 
		
		if (I2C->CR1 != 1) {
			//Если в CR1 что-то лишнее, то перезагрузим I2C
			CLEAR_BIT(I2C->CR1, I2C_CR1_PE);
			SET_BIT(I2C->CR1, I2C_CR1_PE);
		} 	
		
		return false;	
	}
	/*-------------------Проверка занятости шины-------------------*/
	
	CLEAR_BIT(I2C->CR1, I2C_CR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
	SET_BIT(I2C->CR1, I2C_CR1_START); //Стартуем.
	
	Timeout_counter_ms = Timeout_ms;
	while (READ_BIT(I2C->SR1, I2C_SR1_SB) == 0) {
		//Ожидаем до момента, пока не сработает Start condition generated
			
		if (!Timeout_counter_ms) {
			return false;
		}
		
	} 
	//ВНИМАНИЕ!
	/* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
	I2C->SR1;
	I2C->DR = (Adress_Device << 1); //Адрес + Write
	
	Timeout_counter_ms = Timeout_ms;
	while ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C->SR1, I2C_SR1_ADDR) == 0)) {
		//Ждем, пока адрес отзовется
		
		if (!Timeout_counter_ms) {
			return false;
		}
		
	}
	if (READ_BIT(I2C->SR1, I2C_SR1_ADDR)) {
		//Если устройство отозвалось, сбросим бит ADDR
		/*Сброс бита ADDR производится чтением SR1, а потом SR2*/
		I2C->SR1;
		I2C->SR2;
		
		/*Отправим адрес памяти*/
		for (uint16_t i = 0; i < Size_adress; i++) {
			I2C->DR = *((uint8_t*)&Adress_data + (Size_adress - 1 - i)); //Запись байта
			while (READ_BIT(I2C->SR1, I2C_SR1_TXE) == 0) { 
				//Ждем, пока данные загрузятся в регистр сдвига.
				
				if ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 1)) {
					//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
					SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
					CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
					return false;
				}	
			}
		}
	
		/*Будем записывать данные в ячейку памяти, начиная с указанного адреса*/
		for (uint16_t i = 0; i < Size_data; i++) {
			I2C->DR = *(data + i); //Запись байта
			while (READ_BIT(I2C->SR1, I2C_SR1_TXE) == 0) {
				//Ждем, пока данные загрузятся в регистр сдвига.
				
				if ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 1)) {
					//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
					SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
					CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
					return false;
				}
			} 
		}
		
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем

		return true;
	
	} else {
		//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
		CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
		
		return false;
	}
}


/**
 **************************************************************************************************
 *  @breif Функция чтения из памяти по указанному адресу
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства      
 *  @param  Adress_data - Адрес в памяти, откуда будем считывать данные
 *  @param  Size_adress - Размер адреса в байтах. Пример: 1 - 8 битный адрес. 2 - 16 битный адрес.
 *  @param  *data - Данные, в которые будем записывать считанную информацию.
 *  @param  Size_data - Размер, сколько байт будем считывать.
 *  @retval  Возвращает статус считывания. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool CMSIS_I2C_MemRead(I2C_TypeDef *I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
	
	/*-------------------Проверка занятости шины-------------------*/
	if (READ_BIT(I2C->SR2, I2C_SR2_BUSY)) {
		//Если шина занята
		
		if ((READ_BIT(GPIOB->IDR, GPIO_IDR_IDR6)) && (READ_BIT(GPIOB->IDR, GPIO_IDR_IDR7))) {
			//Если линия на самом деле свободна, а BUSY висит
			CMSIS_I2C_Reset(); //ресет
			CMSIS_I2C1_Init(); //повторная инициализация
		} 
		
		if (READ_BIT(I2C->SR2, I2C_SR2_MSL)) {
			//Если стоит статус, что мы в мастере
			SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправим сигнал STOP
		} 
		
		if (I2C->CR1 != 1) {
			//Если в CR1 что-то лишнее, то перезагрузим I2C
			CLEAR_BIT(I2C->CR1, I2C_CR1_PE);
			SET_BIT(I2C->CR1, I2C_CR1_PE);
		} 	
		
		return false;	
	}
	/*-------------------Проверка занятости шины-------------------*/
	
	CLEAR_BIT(I2C->CR1, I2C_CR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
	SET_BIT(I2C->CR1, I2C_CR1_START); //Стартуем.
	
	Timeout_counter_ms = Timeout_ms;
	while (READ_BIT(I2C->SR1, I2C_SR1_SB) == 0) {
		//Ожидаем до момента, пока не сработает Start condition generated
			
		if (!Timeout_counter_ms) {
			return false;
		}
		
	} 
	//ВНИМАНИЕ!
	/* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
	I2C->SR1;
	I2C->DR = (Adress_Device << 1); //Адрес + команда Write
	
	Timeout_counter_ms = Timeout_ms;
	while ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C->SR1, I2C_SR1_ADDR) == 0)) {
		//Ждем, пока адрес отзовется
		
		if (!Timeout_counter_ms) {
			return false;
		}
		
	}
	
	if (READ_BIT(I2C->SR1, I2C_SR1_ADDR)) {
		//Если устройство отозвалось, сбросим бит ADDR
        /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
		I2C->SR1;
		I2C->SR2;
		
		/*Отправим адрес памяти*/
		for (uint16_t i = 0; i < Size_adress; i++) {
			I2C->DR = *((uint8_t*)&Adress_data + (Size_adress - 1 - i)); //Запись байта
			while (READ_BIT(I2C->SR1, I2C_SR1_TXE) == 0) { 
				//Ждем, пока данные загрузятся в регистр сдвига.
				
				if ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 1)) {
					//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
					SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
					CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
					return false;
				}	
			}
		}
		
		//Повторный старт
		SET_BIT(I2C->CR1, I2C_CR1_START); //Стартуем.
		
		Timeout_counter_ms = Timeout_ms;
		while (READ_BIT(I2C->SR1, I2C_SR1_SB) == 0) {
			//Ожидаем до момента, пока не сработает Start condition generated
			
			if (!Timeout_counter_ms) {
				return false;
			}
		
		} 
		//ВНИМАНИЕ!
		/* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
		I2C->SR1;
		I2C->DR = (Adress_Device << 1 | 1); //Адрес + команда Read
		
		Timeout_counter_ms = Timeout_ms;
		while ((READ_BIT(I2C->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C->SR1, I2C_SR1_ADDR) == 0)) {
			//Ждем, пока адрес отзовется
		
			if (!Timeout_counter_ms) {
				return false;
			}
		
		}
		
		if (READ_BIT(I2C->SR1, I2C_SR1_ADDR)) {
			//Если устройство отозвалось, сбросим бит ADDR
			/*Сброс бита ADDR производится чтением SR1, а потом SR2*/
			I2C->SR1;
			I2C->SR2;
			
			/*Прочтем данные, начиная с указанного адреса*/
			for (uint16_t i = 0; i < Size_data; i++) {
				if (i < Size_data - 1) {
					SET_BIT(I2C->CR1, I2C_CR1_ACK); //Если мы хотим принять следующий байт, то отправляем ACK 
					while (READ_BIT(I2C->SR1, I2C_SR1_RXNE) == 0) ;
					*(data + i) = I2C->DR; //Чтение байта
				} else {
					CLEAR_BIT(I2C->CR1, I2C_CR1_ACK); //Если мы знаем, что следующий принятый байт будет последним, то отправим NACK
		
					SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
					while (READ_BIT(I2C->SR1, I2C_SR1_RXNE) == 0) ; //Подождем, пока сдвиговый регистр пополнится новым байтом данных
					*(data + i) = I2C->DR; //Чтение байта
				}
			} return true;
			
		} else {
			//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
			SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
			CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
			return false;
		}
		
	} else {
		//Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF 
		SET_BIT(I2C->CR1, I2C_CR1_STOP); //Останавливаем
		CLEAR_BIT(I2C->SR1, I2C_SR1_AF); //Сбрасываем бит AF
		return false;
	}
}
