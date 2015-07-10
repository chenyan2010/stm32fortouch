/**
  ******************************************************************************
  * @file    sys.h
  * @author  Leo
  * @version V1.00
  * @date    2012.09.17
  * @brief   This file contains all the functions prototypes for the base.
  ******************************************************************************
  */ 
	
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYS_H
#define __SYS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
   

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void MCO_Output(void);
uint32_t OS_SysTick_Config(uint32_t ticks);

  
#ifdef __cplusplus
}
#endif

#endif /* __SYS_H */
 

/* End of File ****************************************************************/
