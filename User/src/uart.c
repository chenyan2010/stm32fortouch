/**
  ******************************************************************************
  * @file    uart.c
  * @author  Leo
  * @version V1.00
  * @date    2012.3.1 - 2012.7.31
  * @brief   This file provides firmware functions to the UART.
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <stdio.h>

#include "stm32f10x.h"
#include "uart.h"
#include "global_data.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define COM1_RXBUF_SIZE       64
 #define COM2_RXBUF_SIZE       64


/* Private macro -------------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USART_TypeDef* COM_USART[COMn] = {EVAL_COM1, EVAL_COM2};
GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT, EVAL_COM2_TX_GPIO_PORT};
GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT, EVAL_COM2_RX_GPIO_PORT};
const uint32_t COM_USART_CLK[COMn] = {EVAL_COM1_CLK, EVAL_COM2_CLK};
const uint32_t COM_TX_PORT_CLK[COMn] = {EVAL_COM1_TX_GPIO_CLK, EVAL_COM2_TX_GPIO_CLK};
const uint32_t COM_RX_PORT_CLK[COMn] = {EVAL_COM1_RX_GPIO_CLK, EVAL_COM2_RX_GPIO_CLK};
const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN, EVAL_COM2_TX_PIN};
const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN, EVAL_COM2_RX_PIN}; 


uint8_t Com1_RxBuf[COM1_RXBUF_SIZE];
__IO uint16_t Com1_RxBuf_Ip = 0;
__IO uint16_t Com1_RxBuf_Op = 0;


uint8_t RecvPtr = 0;
uint8_t ComDMARecvBuff[10]; 
#define _UART_CMD_LENGTH_ 43
bool g_bUartCmdDetect=FALSE;
typedef struct _UartCommadType
{
    uint8_t Buffer[_UART_CMD_LENGTH_];  // command buffer
                                     // [Length+Command] - [Byte 2] - [Byte 3] - [Byte 4] - [Byte 5]
    uint8_t Index;                     // buffer index

}UartCommadType;
UartCommadType g_UartCommand;
UartCommadType g_UartCommand1;

#define UART_TOUCHSCREEN_H					(g_UartCommand.Buffer[0]==0x1F)?TRUE:FALSE
#define UART_TOUCHSCREEN_L					(g_UartCommand.Buffer[1]==0xF7)?TRUE:FALSE
#define UART_TOUCHSCREEN	((g_UartCommand.Buffer[0]<<8|g_UartCommand.Buffer[1])==0x1FF7)?TRUE:FALSE//vino 20150407 modified for mantis 281

#define UART_CUSTOMERDEF_CTRL
#ifdef  UART_CUSTOMERDEF_CTRL
#define UART_CUSTOMERDEF_CMD_H					(g_UartCommand1.Buffer[0]==0x99)?TRUE:FALSE
#define UART_CUSTOMERDEF_CMD_L					(g_UartCommand1.Buffer[1]==0x23)?TRUE:FALSE
#define UART_CUSTOMERDEF_CMD	((g_UartCommand1.Buffer[0]<<8|g_UartCommand1.Buffer[1])==0x9923)?TRUE:FALSE//vino 20150407 modified for mantis 281
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/ 

void USART_RCC_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
#ifdef USE_UART2_FUNCTION
#endif
}

void USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStruct;

	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
	USART_Init(USART1, &USART_InitStruct);
	USART_Cmd(USART1, ENABLE);
	USART_Init(USART2, &USART_InitStruct);
	USART_Cmd(USART2, ENABLE);

#ifdef USE_UART2_FUNCTION
#endif
}

void USART_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

GPIO_Init(GPIOA, &GPIO_InitStruct);

GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	
GPIO_Init(GPIOA, &GPIO_InitStruct);
#ifdef USE_UART2_FUNCTION
#else
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
		
	GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif	
}



void UART_NVIC_Configuration(void) 
{ 
#ifdef  USE_UART_CONTROL_POINT_SEND
	
	NVIC_InitTypeDef NVIC_InitStructure; 

	//设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 

	//串口接收中断打开
	NVIC_InitStructure.NVIC_IRQChannel = ContrlUART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(MCU_COM_UART, USART_IT_RXNE , ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = ContrlUART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(MCU_COM2_UART, USART_IT_RXNE , ENABLE);
	
#endif 
} 




void COM_Full_Init( void )
{
	//DMA_Configuration();
	
	USART_RCC_Configuration();
	USART_GPIO_Configuration();
	USART_Configuration();
	UART_NVIC_Configuration();
	
	//USART_DMACmd(MCU_COM_UART, USART_DMAReq_Rx, ENABLE);
	//USART_DMACmd(MCU_COM_UART, USART_DMAReq_Tx,ENABLE); 
	//DMA_NVIC_Configuration();
	//DMA_Cmd(DMA1_Channel1,ENABLE);
}

#ifdef  USE_UART_CONTROL_POINT_SEND
extern bool g_bUsbSendOpen,g_bUartSendOpen;
extern uint8_t  g_u8UartCmdErrCode;
void USART2_printf (char *fmt, ...) ;
extern void testUART(void);
extern volatile uint8_t  g_u8UsbSendBuffer[MULTI_MODE_REPORT_COUNT];
uint8_t u8TempCurrentPoint=0;
extern uint8_t g_u8for818send_data;//chenyan 20150629 add for customer uart command.
uint8_t isUartRecving = 0;//chenyan 20150630 add for uart and i2c operation g_u8for818send_data synchronized.

int SendChar (int ch)
{
	while (!(USART1->SR & USART_FLAG_TXE));

	USART1->DR = (ch & 0x1FF);

	return (ch);
}

void ContrlUART_IRQHander(void)
{
	if((MCU_COM_UART->SR & USART_FLAG_RXNE) != 0)
	{
	    isUartRecving = 1;//uart is receiving.
		MCU_COM_UART->SR = (uint16_t)~USART_FLAG_RXNE;
        if (g_UartCommand1.Index >= _UART_CMD_LENGTH_)
		{
			g_UartCommand1.Index = 0;
		}
        g_UartCommand1.Buffer[g_UartCommand1.Index] = (MCU_COM_UART->DR & (uint16_t)0x1FF);
		//putchar(MCU_COM_UART->DR & (uint16_t)0x1FF);
		//putchar('C');
		if(UART_CUSTOMERDEF_CMD_H)
        {
            if(g_UartCommand1.Index < 2)
            {
                g_UartCommand1.Index ++;
            }
            else if(UART_CUSTOMERDEF_CMD)
            {
                if(g_UartCommand1.Index < 6)
                {
                    g_UartCommand1.Index ++;
                }
                if(g_UartCommand1.Index >= 6)//99 23 00 01 FF AA -->key home.
                {
                    g_UartCommand1.Index = 0;
                    g_u8for818send_data = g_UartCommand1.Buffer[2];
                    isUartRecving = 0;//release g_u8for818send_data.
                }
            
            }
            else
			{
				g_UartCommand1.Index = 0; 
			}
        
        }
	}  
}

void ContrlUART2_IRQHander(void)
{
	if((MCU_COM2_UART->SR & USART_FLAG_RXNE) != 0)
	{   
		MCU_COM2_UART->SR = (uint16_t)~USART_FLAG_RXNE;
		if (g_UartCommand.Index >= _UART_CMD_LENGTH_)
		{
			g_UartCommand.Index = 0;
		}
		g_UartCommand.Buffer[g_UartCommand.Index] = (MCU_COM2_UART->DR & (uint16_t)0x1FF);
		//putchar(g_UartCommand.Buffer[g_UartCommand.Index] );
		if(UART_TOUCHSCREEN_H)
		{
			if (g_UartCommand.Index <2) // check 1st data
			{
				g_UartCommand.Index++; // for get UART_CMD_EXT_LENGTH
				return;
			}
			else if(UART_TOUCHSCREEN)
			{
				if (g_UartCommand.Index < 43) // still read command
				{
					g_UartCommand.Index++; // next index of command buffer
				}
				if (g_UartCommand.Index >= 43) // read command ok
				{		
					g_u8UsbSendBuffer[0]=02;

					for(u8TempCurrentPoint=1; u8TempCurrentPoint<32;u8TempCurrentPoint++)
					{
						if(g_UartCommand.Buffer[u8TempCurrentPoint+4]==2)
							g_u8UsbSendBuffer[u8TempCurrentPoint]=4;//index--->1:5
						else if(g_UartCommand.Buffer[u8TempCurrentPoint+4]==3)
							g_u8UsbSendBuffer[u8TempCurrentPoint]=7;
						else 
							g_u8UsbSendBuffer[u8TempCurrentPoint]=0;//state
							
						u8TempCurrentPoint++;
						g_u8UsbSendBuffer[u8TempCurrentPoint]=g_UartCommand.Buffer[u8TempCurrentPoint+4];//index--->2:6
						u8TempCurrentPoint++;
						g_u8UsbSendBuffer[u8TempCurrentPoint]=g_UartCommand.Buffer[u8TempCurrentPoint+4];//index--->3:7
						u8TempCurrentPoint++;
						g_u8UsbSendBuffer[u8TempCurrentPoint]=g_UartCommand.Buffer[u8TempCurrentPoint+4];//index--->4:8
						u8TempCurrentPoint++;
						g_u8UsbSendBuffer[u8TempCurrentPoint]=g_UartCommand.Buffer[u8TempCurrentPoint+4];//index--->5:9
						u8TempCurrentPoint++;
						g_u8UsbSendBuffer[u8TempCurrentPoint]=g_UartCommand.Buffer[u8TempCurrentPoint+4];//index--->6:10
					}
					g_u8UsbSendBuffer[u8TempCurrentPoint]=g_UartCommand.Buffer[u8TempCurrentPoint+4];
					g_UartCommand.Index = 0; // reset index of command buffer
					g_bUartCmdDetect=TRUE;
				}
			}
			else
			{
				g_UartCommand.Index = 0; 
			}
		}
	}  
}
#endif


int GetKey (void) 
{
	while (!(USART1->SR & USART_FLAG_RXNE));
	return ((int)(USART1->DR & 0x1FF));
}

void USART2_printf (char *fmt, ...) 
{ 
char buffer[COM2_RXBUF_SIZE+1];  // CMD_BUFFER_LEN长度自己定义吧 
u8 i = 0; 

va_list arg_ptr; 
va_start(arg_ptr, fmt);   
vsnprintf(buffer, COM2_RXBUF_SIZE+1, fmt, arg_ptr); 
while ((i < COM2_RXBUF_SIZE) && buffer[i]) 
{ 
        USART_SendData(USART2, (u8) buffer[i++]); 
while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);  
} 
va_end(arg_ptr); 
} 


