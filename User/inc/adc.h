/**
  ******************************************************************************
  * @file    adc.h
  * @author  Leo
  * @version V1.00
  * @date    20121015 - 
  * @brief   This file contains definitions for STM32F103xx Leds, push-buttons
  *          and switch ports hardware resources.
  ******************************************************************************
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
	 

#ifdef   ADC_GLOBALS
#define  ADC_EXT
#else
#define  ADC_EXT  extern
#endif
	 
	 	 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

	 
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void ADC1_IN8_Init(void);



#ifdef __cplusplus
}
#endif

#endif /* __ADC_H */


/* End of File ****************************************************************/
