/**
 ******************************************************************************
 *  @file stm32f103xx_CMSIS.h
 *  @brief CMSIS на примере МК STM32F103C8T6
 *  @author Волков Олег
 *  @date 17.07.2022
 *
 ******************************************************************************
 * @attention
 *
 *  Библиотека помогает разобраться с библиотекой CMSIS на примере
 *  МК STM32F103C8T6
 *
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub:
 *https://github.com/Solderingironspb/Lessons-Stm32/blob/master/README.md Группа
 *ВК: https://vk.com/solderingiron.stm32 Работал по Reference Manual:
 *https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 *
 ******************************************************************************
 */

#ifndef __STM32F103XX_CMSIS_H
#define __STM32F103XX_CMSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <main.h>
#include <stdbool.h>
#include <stm32f103xb.h>

    //Структура по USART
    struct USART_name {
        uint8_t tx_buffer[20]; //Буфер под выходящие данные
        uint8_t rx_buffer[20]; //Буфер под входящие данные
        uint16_t rx_counter; //Счетчик приходящих данных типа uint8_t по USART
        uint16_t rx_len; //Количество принятых байт после сработки флага IDLE
    };
	
	//GPIO Configuration mode
	enum { 
		GPIO_GENERAL_PURPOSE_OUTPUT,
		GPIO_ALTERNATIVE_FUNCTION_OUTPUT,
		GPIO_INPUT
	};
	//GPIO Input/OUTPUT type
	enum { 
		GPIO_OUTPUT_PUSH_PULL,
		GPIO_OUTPUT_OPEN_DRAIN,
		GPIO_INPUT_ANALOG,
		GPIO_INPUT_FLOATING,
		GPIO_INPUT_PULL_DOWN,
		GPIO_INPUT_PULL_UP
	};
	
	//GPIO Maximum output speed
	enum {
		GPIO_SPEED_RESERVED,
		GPIO_SPEED_10_MHZ,
		GPIO_SPEED_2_MHZ,
		GPIO_SPEED_50_MHZ 
	};

    void CMSIS_Debug_init(void); //Настройка Debug (Serial Wire)
    void CMSIS_RCC_SystemClock_72MHz(void); //Настрока тактирования микроконтроллера на частоту 72MHz
    void CMSIS_SysTick_Timer_init(void); //Инициализация системного таймера
    void Delay_ms(uint32_t Milliseconds); //Функция задержки
    void SysTick_Handler(void); //Прерывания от системного таймера
    void CMSIS_PC13_OUTPUT_Push_Pull_init(void); //Пример настройки ножки PC13 в режим Push-Pull 50 MHz
	void CMSIS_GPIO_init(GPIO_TypeDef *GPIO, uint8_t GPIO_Pin, uint8_t Configuration_mode, uint8_t Type, uint8_t Speed); //Конфигурация GPIO
    void CMSIS_Blink_PC13(uint32_t ms); //Обычный blink
    void CMSIS_PA8_MCO_init(void); //Пример настройки ножки PA8 в выход тактирующего сигнала c MCO
    void CMSIS_RCC_AFIO_enable(void); //Включить тактирование для альтернативных функций
    void CMSIS_AFIO_EXTICR1_B0_select(void); //Пример выбора ножки PB0 для работы с EXTI0
    void CMSIS_PB0_INPUT_Pull_Down_init(void); //Настройка ножки PB0 на вход. Подтяжка к земле.
    void CMSIS_EXTI_0_init(void); //Инициализации EXTI0 для PB0 в режиме Rishing edge trigger
    void EXTI0_IRQHandler(void); //Прерывания от EXTI0
    void CMSIS_TIM3_init(void); //Инициализация таймера 3. Включение прерывания по переполнению
    void CMSIS_TIM3_PWM_CHANNEL1_init(void); //Запуск шим канала 1 (PA6 - Pin)
    void CMSIS_TIM3_PWM_CHANNEL2_init(void); //Запуск шим канала 2 (PA7 - Pin)
    void TIM3_IRQHandler(void); //Прерывание по таймеру 3.
    void CMSIS_ADC_DMA_init(void); //Пример настройки АЦП + DMA на 2 канала. PA0 и Vrefint
    void ADC1_2_IRQHandler(void);        //Прерывание по АЦП
    void DMA1_Channel1_IRQHandler(void); //Прерывание по DMA(АЦП)
    void CMSIS_USART1_Init(void);        //Инициализация USART1
    void CMSIS_USART2_Init(void);        //Инициализация USART2
    bool CMSIS_USART_Transmit(USART_TypeDef* USART, uint8_t* data, uint16_t Size, uint32_t Timeout_ms); //Отправка данных по USART
    void USART1_IRQHandler(void); //Прерывание по USART1
    void CMSIS_I2C_Reset(void);   //Сброс настроек I2C
    void CMSIS_I2C1_Init(void); //Функция инициализации шины I2C1. Sm.
    bool CMSIS_I2C_Adress_Device_Scan(I2C_TypeDef* I2C, uint8_t Adress_Device, uint32_t Timeout_ms); //Функция сканирования устройства по заданному 7-битному адресу
    bool CMSIS_I2C_Data_Transmit(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция передачи данных по I2C
    bool CMSIS_I2C_Data_Receive(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция приема данных по I2C
    bool CMSIS_I2C_MemWrite(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция записи в память по указанному адресу
    bool CMSIS_I2C_MemRead(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция чтения из памяти по указанному адресу
    void CMSIS_SPI1_init(void); //Инициализация SPI1
    bool CMSIS_SPI_Data_Transmit_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция отправки данных по SPI
    bool CMSIS_SPI_Data_Transmit_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция отправки данных по SPI
    bool CMSIS_SPI_Data_Receive_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);//Функция приема данных по SPI
    bool CMSIS_SPI_Data_Receive_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция приема данных по SPI
    bool CMSIS_SPI_Data_Transmit_fast(SPI_TypeDef* SPI, GPIO_TypeDef* GPIO, uint8_t NSS_pin, bool NSS_logic, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция передачи данных по SPI(быстрая. CS уже включен в нее)
    bool CMSIS_SPI_Data_Receive_fast(SPI_TypeDef* SPI, GPIO_TypeDef* GPIO, uint8_t NSS_pin, bool NSS_logic, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms); //Функция приема данных по SPI(быстрая. CS уже включен в нее)
    void FLASH_Unlock(void); //Разблокировка FLASH
    void FLASH_Lock(void); //Блокировка FLASH
    void FLASH_Page_erase(uint16_t Adress); //Стирание страницы во FLASH
    void FLASH_Page_write(uint32_t Adress, uint8_t *Data, uint16_t Size); //Запись страницы во FLASH
    void FLASH_Read_data(uint32_t Adress, uint8_t *Data, uint16_t Size); //Считывание данных с FLASH
    
    void xPortSysTickHandler( void );

#ifdef __cplusplus
}
#endif

#endif /* __STM32F103XX_CMSIS_H */
