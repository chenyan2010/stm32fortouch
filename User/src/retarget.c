/**
  ******************************************************************************
  * @file    retarget.c 
  * @author  Leo
  * @version V1.00
  * @date    2012.09.17
  * @brief   This file provides firmware functions to the retarget.
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <rt_misc.h>
#include "uart.h"


#pragma import(__use_no_semihosting_swi)

//#pragma import(__use_no_semihosting_swi)

extern int SendChar(int ch); // 
extern int GetKey(void);


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct __FILE
{ 
  int handle;			/* Add whatever you need here */ 
};

FILE __stdout;
FILE __stdin;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  ch
  * @param  Pointer to f
  * @retval None
  */
 
  
int fputc(int ch, FILE *f)
{
  return (SendChar(ch));
}

/**
  * @brief  Retargets the C library scanf function to the USART.
  * @param  Pointer to f
  * @retval None
  */

int fgetc(FILE *f)
{	
	return (SendChar(GetKey()));
}

/**
  * @brief  Retargets the C library scanf function to the USART.
  * @param  ch
  * @retval None
  */
void _ttywrch_noint(int ch)
{
    SendChar(ch);
}

/**
  * @brief  Retargets the C library ferror.
  * @param  Pointer to f
  * @retval None
  */
int ferror(FILE *f)
{
  /* Your implementation of ferror */
  return EOF;
}


/**
  * @brief  Retargets the C library scanf function to the USART.
  * @param  return_code
  * @retval None
  */
void _sys_exit(int return_code)
{
label:  goto label;  		/* endless loop */
}

#if 0

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  ch
  * @param  Pointer to f
  * @retval None
  */


int fputc_noint(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
	
	/* Loop until the end of transmission */
	while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET);
	
  /* e.g. write a character to the USART */
  USART_SendData(EVAL_COM1, (uint8_t) ch);

  return ch;
}

/**
  * @brief  Retargets the C library scanf function to the USART.
  * @param  Pointer to f
  * @retval None
  */
int fgetc_noint(FILE *f)
{
	int  ch;
	
  while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_RXNE) == RESET);
  ch = USART_ReceiveData(EVAL_COM1);
	
	while(USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET);
	USART_SendData(EVAL_COM1, (uint8_t) ch);
		
	return ch;
}



#endif






/* End of File ****************************************************************/
