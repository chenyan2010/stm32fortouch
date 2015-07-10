
/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Joystick Mouse demo main file
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#define TASK_GLOBALS

#include <stdio.h>

#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "global_data.h"
#include "sys.h"
#include "gpio.h"
#include "uart.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "tim.h"
#include "adc.h"
#include "tr.h"
#include "user_config.h"
#include "Gpio_operate.h"
#include "I2c_slave.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main.
* Description    : main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
extern void testUART(void);
extern bool g_bUartCmdDetect;
extern uint8_t i2c_regaddr;
extern bitfield8 tagI2C;

E_UI_INPUT_SOURCE source_num = UI_INPUT_SOURCE_HDMI;//chaojun.liu 20150609 add for source save num for smart signal switch.
uint8_t channel_index = 0;//chaojun.liu 20150608 add for smart signal switch func.
#define COUNTER_DATA            300//chaojun.liu 20150609 add for smart signal switch.
uint16_t u16monitorSOC182_count = COUNTER_DATA;//chaojun.liu 20150609 add for monitor soc818 no signal go to  sleep counter.
#define RELOAD_COUNTER()    do{u16monitorSOC182_count = COUNTER_DATA;}while(0)
#define START_COUNTER()     do{    \
                             if(u16monitorSOC182_count != 0)   \
                             u16monitorSOC182_count --;        \
                             //MSG_PRINT(printf("counter = %d\n",u16monitorSOC182_count)); \
                              }while(0)
#define IS_COUNTER_ZERO()   (u16monitorSOC182_count == 0)
CHANNEL_NUM_TYPE ch_num = CHANNEL_YPBPR;
CHANNEL_NUM_TYPE ch_hassignal = CHANNEL_NONE;//SOC which channel has signal.
CHANNEL_NUM_TYPE ch_nosignal = CHANNEL_NONE;//SOC current channel has no signal.
uint16_t u16switched_channel = 0;//switched channels need not switch again.
uint16_t u16send_data = 0;//save two bytes data.//chaojun.liu 20150609 add for smart signal switch func save send data.
uint16_t u16recv_data = 0;
extern uint8_t recv_datArray[];//i2c receive three bytes data.


uint8_t  SEND_CTRLTOUCHArray[][9]={
   {0x1f,0xf7,0xfc,0x30,0x08,0x01,0x00,0x01,0x4c},//open touch screen.
   {0x1f,0xf7,0xfc,0x30,0x08,0x01,0x00,0x00,0x4b} //close touch screen.
};

uint8_t SEND_SHORTCUTArray[]={0x1f,0xf7,0xfc,0xad,0x01,0x02,0x00,0x00,0x00,0x00};//shortcut last three bytes.

///////////////////////////////////////////////////////////////////////////////
//<FuncName>: I2C_Parse_Recdata.
//<Description>: parse the receive data.
//<Param[IN]>: uint8_t *recv_buf.
//<Returns>: void.
//<Author>: ChenYan.
//<Date>: 2014/10/16.
///////////////////////////////////////////////////////////////////////////////
extern void UART_SendChar(USART_TypeDef * USARTx, uint8_t * buff, uint8_t lenght);
void I2C_Parse_Recdata(uint8_t *recv_buf)
{
    uint8_t i = 0;
	switch(i2c_regaddr)
	{			
		case REGADDR_OPEN_TOUCH://open touchscreen.
		     //printf("open_touch!\n");
             UART_SendChar(MCU_COM2_UART, SEND_CTRLTOUCHArray[0], 9);
			 break;

        case REGADDR_CLOSE_TOUCH://close touchscreen.
             //printf("close_touch!\n");
             UART_SendChar(MCU_COM2_UART, SEND_CTRLTOUCHArray[1], 9);
			 break;
             
        case REGADDR_SEND_SHORTCUT://send shortcut to touchscreen.
             for(i = 0;i < RECV_DATA_SIZE;i ++)
             {
                SEND_SHORTCUTArray[7+i] = recv_buf[i];
             }
             UART_SendChar(MCU_COM2_UART, SEND_SHORTCUTArray, 10);
             break;
			
		default:
			 break;
	}

}

void NVIC_I2c_Int_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 

	//设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 

	//I2C EVENT IRQ ENABLE.
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    //I2C EVENT IRQ ENABLE.
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    I2C_ITConfig(I2C1, (I2C_IT_BUF | I2C_IT_EVT | I2C_IT_ERR), ENABLE);//enable i2c interrupt.
}

void I2C1_Init(void)
{
    I2C_InitTypeDef I2C_InitStruct;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);//enable afio clock.
	Init_I2c_Gpio();//i2c init;
	I2C_Cmd(I2C1, DISABLE);
	I2C_StructInit(&I2C_InitStruct);
	I2C_Init(I2C1,&I2C_InitStruct);
    NVIC_I2c_Int_Configuration();
    I2C_Cmd(I2C1, ENABLE);
}

int main(void)
{
	//FLASH_ReadOutProtection(DISABLE);
    
	Set_System();
	
	USB_Interrupts_Config();
	
	Set_USBClock();  
	
	USB_Init();
	
	COM_Full_Init();
    
	Init_AVCheck_GPIO();//check all signal port.

    I2C1_Init();//i2c1 init.

    printf("system init end!\n");
    
	while(1)
	{
		if(g_bUartCmdDetect)
		{
			testUART();
			g_bUartCmdDetect=FALSE;
		}

		if(1 == tagI2C.Bit0)//SOC send data to STM32 flag I2C address matched.
        {
            tagI2C.Bit0 = 0;//clear the flag.
        }

        //SOC write data to STM,handle the event.
		if(1 == tagI2C.Bit1)//SOC write data to STM32 flag.
		{
			I2C_Parse_Recdata(recv_datArray);
			tagI2C.Bit1 = 0;
		}

#if  0 /* chenyan marked */
        if(1 == tagI2C.Bit2)//SOC818 read data from STM32.
        {
            tagI2C.Bit2 = 0;
            RELOAD_COUNTER();//reload counter to default value.
        }
#endif /* chenyan marked */
///////chaojun.liu 20150608 add for smart signal switch func end ---.
	}
	

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
