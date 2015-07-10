/**
  ******************************************************************************
  * @file    tim.h
  * @author  Leo
  * @version V1.00
  * @date    20121015 - 
  * @brief   This file contains definitions for STM32F103's TIM.
  ******************************************************************************
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H
#define __TIM_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

	 
#ifdef   TIM_GLOBALS
#define  TIM_EXT
#else
#define  TIM_EXT  extern
#endif
	 

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define TIM2_Pre_0p5us		(2000000)		/* 2MHz is 0.5us */
#define TIM2_ShortWait		(8)					/* 8 is 8/2=4us */ 
#define TIM2_LongWait		(24)				/* 24 is 24/2=12us */

	 
/* Exported variables --------------------------------------------------------*/
TIM_EXT __IO uint16_t TIM2CCR1_Val;	 
TIM_EXT __IO uint16_t TIM2CCR2_Val;
TIM_EXT __IO uint16_t TIM2CCR3_Val;
TIM_EXT __IO uint16_t TIM2CCR4_Val; 
	 
TIM_EXT __IO uint16_t TIM3CCR1_Val;  
TIM_EXT __IO uint16_t TIM3CCR2_Val;
TIM_EXT __IO uint16_t TIM3CCR3_Val;
TIM_EXT __IO uint16_t TIM3CCR4_Val; 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void TIM2_Init(void);
void TIM3_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* __TIM_H */


/* End of File ****************************************************************/
