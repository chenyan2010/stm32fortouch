/**
  ******************************************************************************
  * @file    param.h
  * @author  Leo
  * @version V1.00
  * @date    20121015 - 
  * @brief   This file contains definitions for STM32F103xx Leds, push-buttons
  *          and switch ports hardware resources.
  ******************************************************************************
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAM_H
#define __PARAM_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
	 
	 
#ifdef   PARAM_GLOBALS
#define  PARAM_EXT
#else
#define  PARAM_EXT  extern
#endif
	 
	 
/* Exported variables --------------------------------------------------------*/
//#define X_POINT_SIZE			128
//#define Y_POINT_SIZE			64
	 
PARAM_EXT uint8_t		X_Clumn_Size;
PARAM_EXT uint8_t		Y_Clumn_Size;
	 
/**
 * @brief Resistor set
 */
#define R_2048		0x0001
#define R_1024		0x0002
#define R_512		0x0004
#define R_256		0x0008
#define R_128		0x0010
#define R_64		0x0020
#define R_32		0x0040
#define R_16		0x0100
#define R_8			0x0200
#define R_4			0x0400
#define R_1			0x0800	 
   

/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  Point Clumn and Row Structure definition  
  */  
typedef struct
{
	uint8_t		Trans_Clumn;
	uint8_t		Trans_Row;
	uint8_t		Rec_Clumn;
	uint8_t		Rec_Row;
} PointClumnRow_TypeDef;

/** 
  * @brief  Turn_OFF and Turn_ON enumeration  
  */
typedef enum
{
	Turn_OFF = 0,
	Turn_ON  = !Turn_OFF
} TurnAction;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
uint8_t Get_Clumn_Numb(uint8_t point);



#ifdef __cplusplus
}
#endif

#endif /* __PARAM_H */


/* End of File ****************************************************************/
