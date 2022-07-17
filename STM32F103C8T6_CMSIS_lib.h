/**
 ******************************************************************************
 *  @file STM32F103C8T6_CMSIS_lib.h
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
 *  GitHub: https://github.com/Solderingironspb/Lessons-Stm32/blob/master/README.md
 *  Группа ВК: https://vk.com/solderingiron.stm32
 *  Работал по Reference Manual: https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 *  
 ******************************************************************************
 */

#ifndef __STM32F103C8T6_CMSIS_LIB_H
#define __STM32F103C8T6_CMSIS_LIB_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include <main.h>

	void RCC_SystemClock_72MHz(void);
	
#ifdef __cplusplus
}
#endif

#endif /* __STM32F103C8T6_CMSIS_LIB_H */
