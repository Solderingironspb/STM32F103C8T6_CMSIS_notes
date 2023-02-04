/**
 ******************************************************************************
 *  @file MAX6675.c
 *  @brief Библиотека для работы с MAX6675(Преобразователь температуры для термопар K-типа)
 *  @author Волков Олег
 *  @date 23.01.2023
 *
 ******************************************************************************
 * @attention
 *
 *  MAX6675 выполняет компенсацию холодного спая и оцифровывает сигнал термопары K-типа.
 *  Данные выводятся в 12-битном разрешении, совместимом с SPI, формат только для чтения.
 *  Разрешение температуры кратно 0.25 °С.
 *  Используется в промышленности, бытовой технике, системах отопления, вентиляции и кондиционировании.
 *  Горячий спай термопары способен измерять температуру от 0°C до +1023,75°C, при этом
 *  холодный конец (температура окружающей среды платы, на которой установлен MAX6675),
 *  может находиться в диапазоне от -20°C до +85°C.
 *  MAX6675 корректирует измерения по температуре холодного спая.
 *
 *  SPI: Максимальная скорость SCK =  4.3 MHz
 *       CS(NSS) в активном состоянии подтянут к земле
 *       CPOL = 0
 *       CPHA = 0
 *       Работа с данными осуществляется в 16-битном формате.
 *
 *  Частота преобразования: 0.17 - 0.22 секунды (не нужно опрашивать чаще, чем это значение)
 *
 *
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub: https://github.com/Solderingironspb/Lessons-Stm32/blob/master/README.md 
 *  Группа ВК: https://vk.com/solderingiron.stm32
 *  Документация: https://github.com/Solderingironspb/MAX6675_lib/blob/main/MAX6675.pdf
 *
 ******************************************************************************
 */
#include <MAX6675.h>

float MAX6675_Temperature = 0.0f;
float MAX6675_Correction_additive = -1.75f; //Калибровка смещения
float MAX6675_Correction_multiplicative = 1.0f; //Калибровка наклона

#if defined (USE_CMSIS)
float MAX6675_GetTemperature(SPI_TypeDef *SPI) {
    uint16_t MAX6675_rx_data = 0;
    NSS_ON;
    CMSIS_SPI_Data_Receive_16BIT(SPI, (uint16_t*)&MAX6675_rx_data, 1, 100);
    NSS_OFF;
    return ((MAX6675_rx_data >> 3) * 0.25f * MAX6675_Correction_multiplicative) + MAX6675_Correction_additive;
}
#elif defined (USE_HAL)
extern SPI_HandleTypeDef hspi1; //Шина SPI, которую будем подключать.

float MAX6675_GetTemperature(SPI_HandleTypeDef *hspi) {
	uint16_t MAX6675_rx_data = 0;
	NSS_ON;
	HAL_SPI_Receive(&hspi1, (uint8_t*)&MAX6675_rx_data, 1, 100);
	NSS_OFF;
	return ((MAX6675_rx_data >> 3) * 0.25f * MAX6675_Correction_multiplicative) + MAX6675_Correction_additive;
}
#endif

