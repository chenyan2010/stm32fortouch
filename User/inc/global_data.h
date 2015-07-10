/**
  ******************************************************************************
  * @file    global_data.h
  * @author  haleliu
  * @version V1.00
  * @date    20130123 - 
  * @brief   
  ******************************************************************************
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_DATA_H__
#define __GLOBAL_DATA_H__



#ifdef __cplusplus
 extern "C" {
#endif
	 
/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"

/* global macro ------------------------------------------------------------*/

#define 	SETBIT(x,y) 	x |= ( 1<<y ) 	
#define 	CLRBIT(x,y) 	x &= ~(1<<y)
#define		ISONE(x,y)		((x >> y) & 1)
#define		ISZER0(x,y) 	(x & (1 << y))

	#define USE_ZW_BOARD

//#define USE_BOOTLOADER
#define USE_UART_CONTROL_POINT_SEND

#ifdef  USE_UART_CONTROL_POINT_SEND
//#define USE_UART2_FUNCTION
#define USE_UART1_FUNCTION
#endif

#define MCU_COM2_UART	(USART2)
#define ContrlUART2_IRQn		USART2_IRQn
#define ContrlUART2_IRQHander 	 USART2_IRQHandler
#ifdef USE_UART2_FUNCTION
#else
#define MCU_COM_UART	(USART1)
#define ContrlUART_IRQn		USART1_IRQn
#define ContrlUART_IRQHander 	 USART1_IRQHandler
#endif


#define MAX_POINT_CNT			(6)     //chenyan 20150617 modified for multipoint touch.
#define TOUCH_DATA_COUNT       	(MAX_POINT_CNT)
#define	POINT_MARIX_MAX_SIZE	1//(MAX_POINT_CNT * MAX_POINT_CNT)
#define	POINT_BUFFER_SIZE		(MAX_POINT_CNT * 2)

#define USB_SEND_POINT_CNT		(6)

#define X_POINT_SIZE			(20)
#ifndef USE_ZW_BOARD
#define Y_POINT_SIZE			(20)
#else
#define Y_POINT_SIZE			(20)
#endif

#define SINGLE_MODE_REPORT_ID		(1)
#define SINGLE_MODE_REPORT_COUNT	(8)

#define MULTI_MODE_REPORT_ID		(2)
#define MULTI_MODE_REPORT_COUNT		(38) //(1 + 6*MAX_POINT_CNT + 1)//(38)	//(1+6+6+6 ... +1) 

//#define REPORT_COUNT		 	(38)//(1 + 6*MAX_POINT_CNT + 1)//(38)	//(1+6+6+6 ... +1) 
//#define REPORT_COUNT		 	(1 + (6*POINT_BUFFER_SIZE) + 1)//(38)	//(1+6+6+6 ... +1) 

#define USB_EP_TXBUF 			MULTI_MODE_REPORT_COUNT
#define USB_EP_RXBUF 			MULTI_MODE_REPORT_COUNT


/* global types ------------------------------------------------------------*/
//  X & Y Direction enumeration  
typedef enum
{
	X_DIRECTION = 0x10,
	Y_DIRECTION = 0x20,
	YX_DIRECTION = 0x30,//rev-send
	XY_DIRECTION = 0x40,
} DIRECTION_TypeDef;

typedef enum 
{
	CMD_CLOSE_USB_DATA = 0xb0,
	CMD_OPEN_USB_DATA = 0xb1,
	CMD_CLOSE_UART_DATA = 0xb2,
	CMD_OPEN_UART_DATA = 0xb3,

	CMD_CONTROL_MAX,
}ControlCmdType;

/* global variables --------------------------------------------------------*/
/* global functions --------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* __PARAM_H */


/* End of File ****************************************************************/
