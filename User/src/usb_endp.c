/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#ifdef STM32L1XX_MD
#include "stm32l1xx.h"
#else
#include "stm32f10x.h"
#endif /* STM32L1XX_MD */

#include "platform_config.h"
#include "stm32f10x.h"
#include "usb_lib.h"
#include "usb_istr.h"
//#include "stm32_eval.h"
#include "global_data.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#if 0
__IO uint8_t  MsgCmd;
uint8_t  USB_ReceiveFlg = FALSE;
uint8_t  Receive_Buffer[MULTI_MODE_REPORT_COUNT];
#endif


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : EP1_OUT_Callback.
* Description    : EP1 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/

void EP1_OUT_Callback(void)
{
    #if 0
    USB_ReceiveFlg = TRUE;
	USB_SIL_Read(EP1_OUT, Receive_Buffer);
    // PMAToUserBufferCopy(Receive_Buffer, ENDP1_RXADDR,MULTI_MODE_REPORT_COUNT);
    MsgCmd = Receive_Buffer[MULTI_MODE_REPORT_COUNT-1];
    SetEPRxStatus(ENDP1, EP_RX_VALID);
    #endif
}
//extern uint8_t getreportcnt;

void EP1_IN_Callback(void)
{
  /* Set the transfer complete token to inform upper layer that the current 
  transfer has been complete */
  //PrevXferComplete = 1; 
  //getreportcnt ++;
  	//uint8_t i;
     //	for (i = 0; i < MULTI_MODE_REPORT_COUNT; i++) 
		//Transi_Buffer[i] = 0x00;		
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

