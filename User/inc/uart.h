/**
  ******************************************************************************
  * @file    uart.h
  * @author  Leo
  * @version V1.00
  * @date    2012.3.1 - 2012.7.31
  * @brief   This file contains definitions for COM ports hardware resources.
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
   
/* Exported types ------------------------------------------------------------*/
typedef enum 
{
  COM1 = 0,
  COM2 = 1
} COM_TypeDef;	 
	 
/* Exported constants --------------------------------------------------------*/	 
#define COMn                             2

/**
 * @brief Definition for COM port1, connected to USART1
 */ 
#define EVAL_COM1                        USART1
#define EVAL_COM1_CLK                    RCC_APB2Periph_USART1
#define EVAL_COM1_TX_PIN                 GPIO_Pin_9
#define EVAL_COM1_TX_GPIO_PORT           GPIOA
#define EVAL_COM1_TX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM1_RX_PIN                 GPIO_Pin_10
#define EVAL_COM1_RX_GPIO_PORT           GPIOA
#define EVAL_COM1_RX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM1_IRQn                   USART1_IRQn
#define EVAL_COM1_IRQHandler             USART1_IRQHandler

/**
 * @brief Definition for COM port2, connected to UART2
 */ 
#define EVAL_COM2                        USART2
#define EVAL_COM2_CLK                    RCC_APB1Periph_USART2
#define EVAL_COM2_TX_PIN                 GPIO_Pin_2
#define EVAL_COM2_TX_GPIO_PORT           GPIOA
#define EVAL_COM2_TX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM2_RX_PIN                 GPIO_Pin_3
#define EVAL_COM2_RX_GPIO_PORT           GPIOA
#define EVAL_COM2_RX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define EVAL_COM2_IRQn                   USART2_IRQn
#define EVAL_COM2_IRQHandler             USART2_IRQHandler


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void COM_Init(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct);
void COM_Full_Init(void);
void USART_Configuration(void);

int uart1_sendchar(int ch);
//int com1_sendchar(int ch);
//int SendChar (int ch);

int uart1_getchar(void);
//int GetKey (void);

//int com1_getchar(void);
void uart1_sendstring(char *pString);
void com1_sendstring(char *pString);
void uart1_printf(char *fmt, ...);
void com1_printf(char *fmt, ...);
void uart1_scanf(char *fmt, ...);
void com1_scanf(char *fmt, ...);

int uart2_sendchar(int ch);
int com2_sendchar(int ch);
int uart2_getchar(void);
int com2_getchar(void);
void uart2_sendstring(char *pString);
void com2_sendstring(char *pString);
void uart2_printf(char *fmt, ...);
void com2_printf(char *fmt, ...);
void com2_rx_clear(void);

  
#ifdef __cplusplus
}
#endif

#endif /* __UART_H */
  

/* End of File ****************************************************************/
