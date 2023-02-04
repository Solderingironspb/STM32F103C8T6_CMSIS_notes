/**
 ******************************************************************************
 *  @file MAX31865.h
 *  @brief Библиотека для работы с MAX31865 (Преобразователь температуры для термосопротивлений PT100/PT1000)
 *  @author Волков Олег
 *  @date 31.01.2023
 *
 ******************************************************************************
 * @attention
 * 
 *  В данном примере библиотеки я работал с датчиком PT100.
 *  MAX31865 выполняет компенсацию подключения датчика в режимах 3 или 4 проводное.
 *  Поддерживает и 2 проводное подключение.
 *  Микросхема отлично держит температуру окружающей среды и стабильно измеряет сопротивление датчика,
 *  что очень хорошо для точных измерений.
 *
 *  SPI: Максимальная скорость SCK =  5 MHz
 *       CS(NSS) в активном состоянии подтянут к земле
 *       CPOL = 0 или 1
 *       CPHA = 1
 *       Работа с данными осуществляется в 8-битном формате.
 *
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub: https://github.com/Solderingironspb/Lessons-Stm32/blob/master/README.md Группа
 *  ВК: https://vk.com/solderingiron.stm32
 *  Документация: 
 *  
 *
 ******************************************************************************
 */

#ifndef INC_MAX31865_
#define INC_MAX31865_

#include <math.h>
#include <stdbool.h>

 /*----------Макросы----------*/
#define NSS_PORT GPIOA  //Порт ножки CS
#define NSS_PIN  4      //Пин ножки CS

//NSS_ACTIVE_LOW
#define NSS_ON  NSS_PORT->BSRR = (1 << (NSS_PIN + 16)) //CS вкл. 
#define NSS_OFF NSS_PORT->BSRR = (1 << NSS_PIN); //CS выкл.
/*----------Макросы----------*/

 /*----------Выбор библиотеки----------*/
#define USE_CMSIS   //Работать на CMSIS
//#define USE_HAL   //Работать на HAL
/*----------Выбор библиотеки----------*/

#if defined (USE_CMSIS)
#include "stm32f103xx_CMSIS.h"
#elif defined (USE_HAL)
#include "stm32f1xx_hal.h"
#endif

#if defined (USE_CMSIS)
void MAX31865_Init(SPI_TypeDef* SPI, uint8_t num_wires);
uint8_t MAX31865_Configuration_info(SPI_TypeDef* SPI);
double MAX31865_Get_Resistance(SPI_TypeDef* SPI);
#elif defined (USE_HAL)
void MAX31865_Init(SPI_HandleTypeDef * hspi, uint8_t num_wires);
uint8_t MAX31865_Configuration_info(SPI_HandleTypeDef * hspi);
double MAX31865_Get_Resistance(SPI_HandleTypeDef * hspi);
#endif

double MAX31865_Get_Temperature(double PT100_Resistance);

#endif /* INC_MAX31865_ */
