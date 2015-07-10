/**
  ******************************************************************************
  * @file    gpio.c
  * @author  Leo
  * @version V1.00
  * @date    20121015 - 
  * @brief   This file provides firmware functions to the GPIO
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "ucos_ii.h"
#include "task.h"

#include <stdarg.h>
#include <stdio.h>

#include "gpio.h"
#define PARAM_GLOBALS
#include "param.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/																			 																			 
/**
  * @brief  Get the Clumn number of X or Y.
  * @param  point: The Point number.
  * @retval The Clumn number. It's the multiple of 8.
  */
uint8_t Get_Clumn_Numb(uint8_t point)
{
	uint16_t		clumn;
	
	
	clumn = point / 8;
	
	if(point % 8)
	{
		clumn ++;
	}

	return(clumn);
}


/**
  * @brief  Get Point X clumn and row.
  * @param  pPcr: pointer to a PointClumnRow_TypeDef. To get the value.
  * @param  x: The point in X. Leftest is 0.
  * @retval SUCCESS or ERROR.
  */
ErrorStatus Get_X_ClumnRow(PointClumnRow_TypeDef *pPcr, uint16_t x)
{
	if(x >= X_Clumn_Size)
		return(ERROR);
	
	pPcr->Trans_Clumn = x/8;
	pPcr->Trans_Row = x%8;
	
	pPcr->Rec_Clumn = Y_Clumn_Size + (x/8);
	pPcr->Rec_Row = x%8;
	
	return(SUCCESS);
}


/**
  * @brief  Get Point Y clumn and row.
  * @param  pPcr: pointer to a PointClumnRow_TypeDef. To get the value.
  * @param  y: The point in Y. Topest is 0.
  * @retval SUCCESS or ERROR.
  */
ErrorStatus Get_Y_ClumnRow(PointClumnRow_TypeDef *pPcr, uint16_t y)
{
	if(y >= Y_Clumn_Size)
		return(ERROR);
	
	pPcr->Trans_Clumn = X_Clumn_Size + Y_Clumn_Size - (y/8) - 1;
	pPcr->Trans_Row = 8 - (y%8) - 1;
	
	pPcr->Rec_Clumn = Y_Clumn_Size - (y/8) - 1;
	pPcr->Rec_Row = 8 - (y%8) - 1;
	
	return(SUCCESS);
}


/**
  * @brief  Trans point turn.
  * @param  Pcr: PointClumnRow_TypeDef, include clumn and row value.
  * @param  TurnVal: specifies the value to be written to the selected bit.
  *   This parameter can be one of the BitAction enum values:
  *     @arg Turn_OFF: to trun off the port pin
  *     @arg Turn_ON: to trun on the port pin
  * @retval None
  */
void Trans_Point_Turn(PointClumnRow_TypeDef Pcr, TurnAction TurnVal)
{
	int		i;
	

	/* Control select Clumn in Pcr */
	for(i=0; i<(X_Clumn_Size+Y_Clumn_Size); i++)
	{
		if((i==Pcr.Trans_Clumn) && (TurnVal==Turn_ON))
		{
			Trans_Clumn_SendBit(Bit_SET);
		}
		else
		{
			Trans_Clumn_SendBit(Bit_RESET);
		}
	}
	
	
}



   
/* End of File ****************************************************************/
