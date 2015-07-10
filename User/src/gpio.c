/**
  ******************************************************************************
  * @file    gpio.c
  * @author  rui.zhao
  * @version 2.00
  * @date    20130703 
  * @brief   This file provides firmware functions to the GPIO
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "global_data.h"
#include "gpio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_TypeDef*  LED_PORT[LED_GROUP] = {LED1_PORT, LED2_PORT};
const uint16_t LED_PIN[LED_GROUP] = {LED1_PIN, LED2_PIN};
const uint32_t LED_CLK[LED_GROUP] = {LED1_CLK, LED2_CLK};

//extern uint8_t 	g_u8GpioDebug;


GPIO_TypeDef* TR_164CLK_PORT[CTR_GROUP_NMB] = 
{
	TRohm_164CP_PORT, 
	TransClumn_164CP_PORT, 
	RevClumn_164CP_PORT
};

const uint16_t TR_164CLK_PIN[CTR_GROUP_NMB] = 
{
	TRohm_164CP_PIN, 
	TransClumn_164CP_PIN, 
	RevClumn_164CP_PIN
};

const uint32_t TR_164CLK_CLK[CTR_GROUP_NMB] = 
{
	TRohm_164CP_CLK, 
	TransClumn_164CP_CLK, 
	RevClumn_164CP_CLK
};


GPIO_TypeDef* TR_164DATA_PORT[CTR_GROUP_NMB] = 
{
	TRohm_164D_PORT, 
	TransClumn_164D_PORT, 
	RevClumn_164D_PORT
};


const uint16_t TR_164DATA_PIN[CTR_GROUP_NMB] = 
{
	TRohm_164D_PIN, 
	TransClumn_164D_PIN, 
	RevClumn_164D_PIN
};

const uint32_t TR_164DATA_CLK[CTR_GROUP_NMB] = 
{
	TRohm_164D_CLK, 
	TransClumn_164D_CLK, 
	RevClumn_164D_CLK
};


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  * @retval None
  */
void LED_Init(LED_TypeDef LedGroup)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  RCC_APB2PeriphClockCmd(LED_CLK[LedGroup], ENABLE);
  GPIO_InitStructure.GPIO_Pin = LED_PIN[LedGroup];
  GPIO_Init(LED_PORT[LedGroup], &GPIO_InitStructure);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED1  
  * @retval None
  */
void LED_On(LED_TypeDef LedGroup)
{ 
	LED_PORT[LedGroup]->BRR = LED_PIN[LedGroup];
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED1 
  * @retval None
  */
void LED_Off(LED_TypeDef LedGroup)
{
	LED_PORT[LedGroup]->BSRR = LED_PIN[LedGroup];  
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED1  
  * @retval None
  */
void LED_Toggle(LED_TypeDef LedGroup)
{
	LED_PORT[LedGroup]->ODR ^= LED_PIN[LedGroup];
}


/**
  * @brief  Configures the IrDA Operate control GPIOs.
  * @param  IrOpe: Specifies witch HC164, HC138 and CD4051 to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg R_GROUP
  *     @arg TRANS_GROUP
  *     @arg REV_GROUP
  * @retval None
  */
void TR_GPIO_Init(IrOPE_TypeDef ctrGroup)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_Speed_50MHz;

	/* Configure the 164 CLK GPIO  */
	RCC_APB2PeriphClockCmd(TR_164CLK_CLK[ctrGroup], ENABLE);
	GPIO_InitStructure.GPIO_Pin = TR_164CLK_PIN[ctrGroup];
	GPIO_Init(TR_164CLK_PORT[ctrGroup], &GPIO_InitStructure);
	/* Set the 164 CLK to 0 */
	GPIO_WriteBit(TR_164CLK_PORT[ctrGroup], TR_164CLK_PIN[ctrGroup], Bit_RESET);

	/* Configure the 164 DATA GPIO  */
	RCC_APB2PeriphClockCmd(TR_164DATA_CLK[ctrGroup], ENABLE);
	GPIO_InitStructure.GPIO_Pin = TR_164DATA_PIN[ctrGroup];
	GPIO_Init(TR_164DATA_PORT[ctrGroup], &GPIO_InitStructure);
	
	if(ctrGroup == TRANS_GROUP)
	{
		/* Configure GPIO 138CE */
		RCC_APB2PeriphClockCmd(TransRow_138CE_CLK, ENABLE);
		GPIO_InitStructure.GPIO_Pin = TransRow_138CE_PIN;
		GPIO_Init(TransRow_138CE_PORT, &GPIO_InitStructure);
		/* Set the GPIO 138CE  to 1 */
		GPIO_WriteBit(TransRow_138CE_PORT, TransRow_138CE_PIN, Bit_SET);

		/* Configure GPIO 138A 138B 138C */
		RCC_APB2PeriphClockCmd(TransRow_138_CLK, ENABLE);
		GPIO_InitStructure.GPIO_Pin = TransRow_138A_PIN | TransRow_138B_PIN | TransRow_138C_PIN;
		GPIO_Init(TransRow_138_PORT, &GPIO_InitStructure);
	}
	else if(ctrGroup == REV_GROUP)
	{
		/* Configure GPIO 4051A 4051B 4051C */
		RCC_APB2PeriphClockCmd(RevRow_4051_CLK, ENABLE);
		GPIO_InitStructure.GPIO_Pin = RevRow_4051A_PIN | RevRow_4051B_PIN | RevRow_4051C_PIN;
		GPIO_Init(RevRow_4051_PORT, &GPIO_InitStructure);
	}
}


/**
  * @brief  IrOpe(HC164) send 1 bit.
  * @param  IrOpe: Specifies the HC164 to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg R_GROUP
  *     @arg TRANS_GROUP: t_bit high level is open
  *     @arg REV_GROUP: t_bit low level is open
  * @param  BitVal: specifies the value to be written to the selected bit.
  *   This parameter can be one of the BitAction enum values:
  *     @arg Bit_RESET: to clear the port pin
  *     @arg Bit_SET: to set the port pin
  * @retval None
  */
void TR_Send164_1Bit(IrOPE_TypeDef IrOpe, BitAction BitVal)
{
	GPIO_WriteBit(TR_164DATA_PORT[IrOpe], TR_164DATA_PIN[IrOpe], BitVal);

	__ASM volatile ("nop");
	__ASM volatile ("nop");
	GPIO_WriteBit(TR_164CLK_PORT[IrOpe], TR_164CLK_PIN[IrOpe], Bit_SET);
	__ASM volatile ("nop");
	__ASM volatile ("nop");
	GPIO_WriteBit(TR_164CLK_PORT[IrOpe], TR_164CLK_PIN[IrOpe], Bit_RESET);
}


/**
  * @brief  Configures the R, Trans and Recevie IOs.
  * @param  None
  * @retval None
  */
void TR_Init(void)
{
	int			i;

	/* Resistor Init */
	TR_GPIO_Init(R_GROUP);
	Resistor_Set_12bit(Ecurrent_None);
	
	/* Trans Clumn and Row Init */
	TR_GPIO_Init(TRANS_GROUP);
	for(i = X_CLUMN_SIZE+Y_CLUMN_SIZE-1; i >= 0; i --)
	{
		/* Turn Off all trans clumn */
		TR_Send164_1Bit(TRANS_GROUP, Bit_RESET);
	}

	/* Receive Clumn and Row Init */
	TR_GPIO_Init(REV_GROUP);
	for(i = X_CLUMN_SIZE+Y_CLUMN_SIZE-1; i >= 0; i --)
	{
		/* Turn Off all receive clumn */
		TR_Send164_1Bit(REV_GROUP, Bit_SET);
	} 
}


/**
  * @brief  Trans Turn Off.
  * @param  None
  * @retval None
  */
void Trans_TurnOff(void)
{
	TransRow_138CE_PORT->BSRR = TransRow_138CE_PIN;
}


/**
  * @brief  Trans Turn On.
  * @param  None
  * @retval None
  */
void Trans_TurnOn(void)
{
	TransRow_138CE_PORT->BRR = TransRow_138CE_PIN;
}


/**
  * @brief  Trans Point X & Y set.
  * @param  Direction: DIRECTION_TypeDef, X or Y direction.
  *   This parameter can be one of the DIRECTION_TypeDef enum values:
  *     @arg X_DIRECTION: X direction
  *     @arg Y_DIRECTION: Y direction
  * @param  Point: Point number.
  * @retval None
  */

void Init_Trans_Point_Set(DIRECTION_TypeDef Direction, uint16_t Point)
{
	int			i;	
	uint8_t		Clumn, Row;
	//uint8_t		inShift, outShift, insertP;
	static uint8_t		Current_Clumn = X_CLUMN_SIZE + Y_CLUMN_SIZE + 1;// Y_CLUMN_SIZE: 17,  X_CLUMN_SIZE: 30 //47 + 1 = 48
	static uint8_t		Current_Row = 8;
	volatile uint16_t	portVal;
		
	if(Direction == X_DIRECTION)
	{
		Row = Point & 0x07;
		Clumn = Point >> 3;
	}
	else
	{
		Point = (Y_POINT_SIZE - 1) - Point;
		
		Row = Point & 0x07;
		Clumn = Point >> 3;
		
		Clumn = X_CLUMN_SIZE + Clumn;
	}

	if(Clumn != Current_Clumn)
	{
		for(i=(X_CLUMN_SIZE+Y_CLUMN_SIZE-1); i>=0; i--)
		{
			if(i == Clumn)
			{
				TransClumn_164D_PORT->BSRR = TransClumn_164D_PIN;
			}
			else
			{
				TransClumn_164D_PORT->BRR = TransClumn_164D_PIN;
			}
			TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			TransClumn_164CP_PORT->BRR = TransClumn_164CP_PIN;
		}
		
		Current_Clumn = Clumn;
	}

	if(Row != Current_Row)
	{
		portVal = TransRow_138_PORT->ODR;
			
		switch(Row)
		{
			case 0:// 000
				portVal &= ~(TransRow_138C_PIN | TransRow_138B_PIN | TransRow_138A_PIN);
				break;
				
			case 1:// 001
				portVal &= ~(TransRow_138C_PIN | TransRow_138B_PIN);
				portVal |= (TransRow_138A_PIN);
				break;
				
			case 2:// 010
				portVal &= ~(TransRow_138C_PIN | TransRow_138A_PIN);
				portVal |= (TransRow_138B_PIN);
				break;
				
			case 3:// 011
				portVal &= ~(TransRow_138C_PIN);
				portVal |= (TransRow_138B_PIN | TransRow_138A_PIN);
				break;
				
			case 4:// 100
				portVal |= (TransRow_138C_PIN);
				portVal &= ~(TransRow_138B_PIN | TransRow_138A_PIN);
				break;
				
			case 5:// 101
				portVal |= (TransRow_138C_PIN | TransRow_138A_PIN);
				portVal &= ~(TransRow_138B_PIN);
				break;
				
			case 6:// 110
				portVal |= (TransRow_138C_PIN | TransRow_138B_PIN);
				portVal &= ~(TransRow_138A_PIN);
				break;
				
			case 7:// 111 
				portVal |= (TransRow_138C_PIN | TransRow_138B_PIN | TransRow_138A_PIN);
				break;
				
			default:
				break;
		}

		TransRow_138_PORT->ODR = portVal;
		Current_Row = Row;
	}
}

void Trans_Point_Set(DIRECTION_TypeDef Direction, uint16_t Point)
{
	int			i;	
	uint8_t		Clumn, Row;
	//uint8_t		inShift, outShift, insertP;
	static uint8_t		Current_Clumn = X_CLUMN_SIZE + Y_CLUMN_SIZE + 1;// Y_CLUMN_SIZE: 17,  X_CLUMN_SIZE: 30 //47 + 1 = 48
	static uint8_t		Current_Row = 8;
	volatile uint16_t	portVal;
		
	if(Direction == X_DIRECTION)
	{
		Row = Point & 0x07;
		Clumn = Point >> 3;
	}
	else
	{
		Point = (Y_POINT_SIZE - 1) - Point;
		
		Row = Point & 0x07;
		Clumn = Point >> 3;
		
		Clumn = X_CLUMN_SIZE + Clumn;
	}

#if 0	
	if(Clumn != Current_Clumn)
	{
		for(i=(X_CLUMN_SIZE+Y_CLUMN_SIZE-1); i>=0; i--)
		{
			if(i == Clumn)
			{
				TransClumn_164D_PORT->BSRR = TransClumn_164D_PIN;
			}
			else
			{
				TransClumn_164D_PORT->BRR = TransClumn_164D_PIN;
			}
			TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			TransClumn_164CP_PORT->BRR = TransClumn_164CP_PIN;
		}
		
		Current_Clumn = Clumn;
	}
	
#else
	if(Current_Clumn == X_CLUMN_SIZE + Y_CLUMN_SIZE + 1)
	{
		//if(g_u8GpioDebug == 1)
		//printf("\r\n 0 T: %d -> %d \n",Current_Clumn,Clumn);
		
		for(i=(X_CLUMN_SIZE+Y_CLUMN_SIZE-1); i>=0; i--)
		{
			if(i == Clumn)
				TransClumn_164D_PORT->BSRR = TransClumn_164D_PIN;
			else
				TransClumn_164D_PORT->BRR = TransClumn_164D_PIN;

			TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			TransClumn_164CP_PORT->BRR = TransClumn_164CP_PIN;
		}
		
		Current_Clumn = Clumn;
	}
	else if(Clumn > Current_Clumn)
	{
		//if(g_u8GpioDebug == 1)
		//printf("\r\n 1 T: %d -> %d \n",Current_Clumn,Clumn);
		
		//push 0
		TransClumn_164D_PORT->BRR = TransClumn_164D_PIN;

		for(i = Current_Clumn; i < Clumn; i ++) 
		{
			TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			TransClumn_164CP_PORT->BRR = TransClumn_164CP_PIN;
		}
		
		Current_Clumn = Clumn;
	}
	else if(Clumn < Current_Clumn)
	{
		//if(g_u8GpioDebug == 1)
		//printf("\r\n 2 T: %d -> %d \n",Current_Clumn,Clumn);

		//push 0
		TransClumn_164D_PORT->BRR = TransClumn_164D_PIN;

		for(i = Current_Clumn; i < X_CLUMN_SIZE+Y_CLUMN_SIZE; i ++)
		{
			TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			TransClumn_164CP_PORT->BRR = TransClumn_164CP_PIN;
		}

		//push 1
		TransClumn_164D_PORT->BSRR = TransClumn_164D_PIN;

		TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
		__ASM volatile ("nop");
		__ASM volatile ("nop");
		TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
		__ASM volatile ("nop");
		__ASM volatile ("nop");
		TransClumn_164CP_PORT->BRR = TransClumn_164CP_PIN;

		//push 0
		TransClumn_164D_PORT->BRR = TransClumn_164D_PIN;
		for(i = 0; i < Clumn; i ++)
		{
			TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			TransClumn_164CP_PORT->BSRR = TransClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			TransClumn_164CP_PORT->BRR = TransClumn_164CP_PIN;
		}			
		
		Current_Clumn = Clumn;
	}
#endif

	if(Row != Current_Row)
	{
		portVal = TransRow_138_PORT->ODR;
			
		switch(Row)
		{
			case 0:// 000
				portVal &= ~(TransRow_138C_PIN | TransRow_138B_PIN | TransRow_138A_PIN);
				break;
				
			case 1:// 001
				portVal &= ~(TransRow_138C_PIN | TransRow_138B_PIN);
				portVal |= (TransRow_138A_PIN);
				break;
				
			case 2:// 010
				portVal &= ~(TransRow_138C_PIN | TransRow_138A_PIN);
				portVal |= (TransRow_138B_PIN);
				break;
				
			case 3:// 011
				portVal &= ~(TransRow_138C_PIN);
				portVal |= (TransRow_138B_PIN | TransRow_138A_PIN);
				break;
				
			case 4:// 100
				portVal |= (TransRow_138C_PIN);
				portVal &= ~(TransRow_138B_PIN | TransRow_138A_PIN);
				break;
				
			case 5:// 101
				portVal |= (TransRow_138C_PIN | TransRow_138A_PIN);
				portVal &= ~(TransRow_138B_PIN);
				break;
				
			case 6:// 110
				portVal |= (TransRow_138C_PIN | TransRow_138B_PIN);
				portVal &= ~(TransRow_138A_PIN);
				break;
				
			case 7:// 111 
				portVal |= (TransRow_138C_PIN | TransRow_138B_PIN | TransRow_138A_PIN);
				break;
				
			default:
				break;
		}

		TransRow_138_PORT->ODR = portVal;
		Current_Row = Row;
	}
}


/**
  * @brief  Receive Point X & Y set.
  * @param  Direction: DIRECTION_TypeDef, X or Y direction.
  *   This parameter can be one of the DIRECTION_TypeDef enum values:
  *     @arg X_DIRECTION: X direction
  *     @arg Y_DIRECTION: Y direction
  * @param  Point: Point number.
  * @retval None
  */
void Init_Receive_Point_Set(DIRECTION_TypeDef Direction, uint16_t Point)
{
	int			i;	
	uint8_t		Clumn, Row;
	//uint8_t		inShift, outShift, insertP;
	volatile uint16_t	portVal;
	static uint8_t		Current_Clumn = X_CLUMN_SIZE + Y_CLUMN_SIZE + 1;// Y_CLUMN_SIZE: 17,  X_CLUMN_SIZE: 30
	static uint8_t		Current_Row = 8;
		
	if(Direction == Y_DIRECTION)
	{
		Point = (Y_POINT_SIZE - 1) - Point;

		Row = Point & 0x07;
		Clumn = Point >> 3;
	}
	else
	{
		Row = Point & 0x07;
		Clumn = Point >> 3;
		
		Clumn = Y_CLUMN_SIZE + Clumn;
	}

	if(Clumn != Current_Clumn)
	{
		for(i=(X_CLUMN_SIZE+Y_CLUMN_SIZE-1); i>=0; i--)
		{
			if(i == Clumn)
			{
				RevClumn_164D_PORT->BRR = RevClumn_164D_PIN;
			}
			else
			{
				RevClumn_164D_PORT->BSRR = RevClumn_164D_PIN;
			}
			RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			RevClumn_164CP_PORT->BRR = RevClumn_164CP_PIN;
		}
		
		Current_Clumn = Clumn;
	}

	if(Row != Current_Row)
	{	
		portVal = RevRow_4051_PORT->ODR;

		// 0 1 3 2 6 7 5 4 
		switch(Row)
		{
			case 0:// 000	0
				portVal &= ~(RevRow_4051A_PIN|RevRow_4051B_PIN|RevRow_4051C_PIN);
				break;
				
			case 1:// 100	1
				portVal |= (RevRow_4051A_PIN);
				portVal &= ~(RevRow_4051B_PIN|RevRow_4051C_PIN);
				break;
				
			case 2:// 110	3
				portVal |= (RevRow_4051A_PIN | RevRow_4051B_PIN);
				portVal &= ~(RevRow_4051C_PIN);
				break;
				
			case 3:// 010	2
				portVal |= (RevRow_4051B_PIN);
				portVal &= ~(RevRow_4051A_PIN | RevRow_4051C_PIN);
				break;
				
			case 4:// 011	6
				portVal |= (RevRow_4051B_PIN | RevRow_4051C_PIN);
				portVal &= ~(RevRow_4051A_PIN);
				break;
				
			case 5:// 111	7
				portVal |= (RevRow_4051A_PIN | RevRow_4051B_PIN | RevRow_4051C_PIN);
				break;
				
			case 6:// 101	5
				portVal |= (RevRow_4051A_PIN | RevRow_4051C_PIN);
				portVal &= ~(RevRow_4051B_PIN);
				break;
				
			case 7: // 001	4
				portVal |= (RevRow_4051C_PIN);
				portVal &= ~(RevRow_4051A_PIN | RevRow_4051B_PIN);
				break;
				
			default:
				break;
		}
		
		RevRow_4051_PORT->ODR = portVal;
		Current_Row = Row;
	}
}

void Receive_Point_Set(DIRECTION_TypeDef Direction, uint16_t Point)
{
	int			i;	
	uint8_t		Clumn, Row;
	//uint8_t		inShift, outShift, insertP;
	volatile uint16_t	portVal;
	static uint8_t		Current_Clumn = X_CLUMN_SIZE + Y_CLUMN_SIZE + 1;// Y_CLUMN_SIZE: 17,  X_CLUMN_SIZE: 30
	static uint8_t		Current_Row = 8;
		
	if(Direction == Y_DIRECTION)
	{
		Point = (Y_POINT_SIZE - 1) - Point;

		Row = Point & 0x07;
		Clumn = Point >> 3;
	}
	else
	{
		Row = Point & 0x07;
		Clumn = Point >> 3;
		
		Clumn = Y_CLUMN_SIZE + Clumn;
	}

#if 0
	if(Clumn != Current_Clumn)
	{
		for(i=(X_CLUMN_SIZE+Y_CLUMN_SIZE-1); i>=0; i--)
		{
			if(i == Clumn)
			{
				RevClumn_164D_PORT->BRR = RevClumn_164D_PIN;
			}
			else
			{
				RevClumn_164D_PORT->BSRR = RevClumn_164D_PIN;
			}
			RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			RevClumn_164CP_PORT->BRR = RevClumn_164CP_PIN;
		}
		
		Current_Clumn = Clumn;
	}
#else	
	if(Current_Clumn == X_CLUMN_SIZE + Y_CLUMN_SIZE + 1)
	{
		//if(g_u8GpioDebug == 1)
		//printf("\r\n 0 R: %d -> %d \n",Current_Clumn,Clumn);
		
		for(i=(X_CLUMN_SIZE+Y_CLUMN_SIZE-1); i>=0; i--)
		{
			if(i == Clumn)
				RevClumn_164D_PORT->BRR = RevClumn_164D_PIN;
			else
				RevClumn_164D_PORT->BSRR = RevClumn_164D_PIN;

			RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			RevClumn_164CP_PORT->BRR = RevClumn_164CP_PIN;
		}
		
		Current_Clumn = Clumn;
	}
	else if(Clumn > Current_Clumn)
	{
		//if(g_u8GpioDebug == 1)
		//printf("\r\n 1 R: %d -> %d \n",Current_Clumn,Clumn);
		
		//push 1
		RevClumn_164D_PORT->BSRR = RevClumn_164D_PIN;
		for(i = Current_Clumn; i < Clumn; i ++) 
		{
			RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			RevClumn_164CP_PORT->BRR = RevClumn_164CP_PIN;
		}
		
		Current_Clumn = Clumn;
	}
	else if(Clumn < Current_Clumn)
	{
		//if(g_u8GpioDebug == 1)
		//printf("\r\n 2 R: %d -> %d \n",Current_Clumn,Clumn);
		
		//push 1		
		RevClumn_164D_PORT->BSRR = RevClumn_164D_PIN;
		for(i = Current_Clumn; i < X_CLUMN_SIZE+Y_CLUMN_SIZE; i ++) 
		{
			RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			RevClumn_164CP_PORT->BRR = RevClumn_164CP_PIN;
		}

		//push 0
		RevClumn_164D_PORT->BRR = RevClumn_164D_PIN;

		RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
		__ASM volatile ("nop");
		__ASM volatile ("nop");
		RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
		__ASM volatile ("nop");
		__ASM volatile ("nop");
		RevClumn_164CP_PORT->BRR = RevClumn_164CP_PIN;

		//push 1	
		RevClumn_164D_PORT->BSRR = RevClumn_164D_PIN;
		for(i = 0; i < Clumn; i ++)
		{
			RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			RevClumn_164CP_PORT->BSRR = RevClumn_164CP_PIN;
			__ASM volatile ("nop");
			__ASM volatile ("nop");
			RevClumn_164CP_PORT->BRR = RevClumn_164CP_PIN;
		}	
		
		Current_Clumn = Clumn;
	}
#endif	

	if(Row != Current_Row)
	{	
		portVal = RevRow_4051_PORT->ODR;

		// 0 1 3 2 6 7 5 4 
		switch(Row)
		{
			case 0:// 000	0
				portVal &= ~(RevRow_4051A_PIN|RevRow_4051B_PIN|RevRow_4051C_PIN);
				break;
				
			case 1:// 100	1
				portVal |= (RevRow_4051A_PIN);
				portVal &= ~(RevRow_4051B_PIN|RevRow_4051C_PIN);
				break;
				
			case 2:// 110	3
				portVal |= (RevRow_4051A_PIN | RevRow_4051B_PIN);
				portVal &= ~(RevRow_4051C_PIN);
				break;
				
			case 3:// 010	2
				portVal |= (RevRow_4051B_PIN);
				portVal &= ~(RevRow_4051A_PIN | RevRow_4051C_PIN);
				break;
				
			case 4:// 011	6
				portVal |= (RevRow_4051B_PIN | RevRow_4051C_PIN);
				portVal &= ~(RevRow_4051A_PIN);
				break;
				
			case 5:// 111	7
				portVal |= (RevRow_4051A_PIN | RevRow_4051B_PIN | RevRow_4051C_PIN);
				break;
				
			case 6:// 101	5
				portVal |= (RevRow_4051A_PIN | RevRow_4051C_PIN);
				portVal &= ~(RevRow_4051B_PIN);
				break;
				
			case 7: // 001	4
				portVal |= (RevRow_4051C_PIN);
				portVal &= ~(RevRow_4051A_PIN | RevRow_4051B_PIN);
				break;
				
			default:
				break;
		}
		
		RevRow_4051_PORT->ODR = portVal;
		Current_Row = Row;
	}
}


/**
  * @brief  Trans Turn Resistor.
  * @param  None
  * @retval None
  */
void Resistor_Set_12bit(uint16_t ResVal)
{
	int		i;
	
	for(i = 12-1; i >= 0; i --)
	{
		if((ResVal&0x800) == 0)
			TRohm_164D_PORT->BRR = TRohm_164D_PIN;
		else
			TRohm_164D_PORT->BSRR = TRohm_164D_PIN;

		TRohm_164CP_PORT->BSRR = TRohm_164CP_PIN;
		__ASM volatile ("nop");
		__ASM volatile ("nop");
		TRohm_164CP_PORT->BSRR = TRohm_164CP_PIN;
		__ASM volatile ("nop");
		__ASM volatile ("nop");
		TRohm_164CP_PORT->BRR = TRohm_164CP_PIN;
		
		ResVal <<= 1;
	}
}
///////chaojun.liu 20150608 add for smart signal switch start +++.
///////////////////////////////////////////////////////////////////////////////
//<FuncName>: Init_AVCheck_GPIO.
//<Description>: initialize the GPIOS.
//<Param[IN]>: void.
//<Returns>: void.
//<Author>: ChenYan.
//<Date>: 2014/10/15.
///////////////////////////////////////////////////////////////////////////////
void Init_AVCheck_GPIO(void)
{	
	/* Initialize I/Os */
	//PB input mode.
	GPIO_InitTypeDef StuGpio;
	StuGpio.GPIO_Pin= All_PB_CHK_IN;
	StuGpio.GPIO_Speed=GPIO_Speed_2MHz;
	StuGpio.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//enable afio clock.
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	GPIO_Init(GPIOB, &StuGpio);//all PB signal port.
	//PC13 input mode.
	StuGpio.GPIO_Pin=HD_CHK_IN;//pin for ypbpr.
	GPIO_Init(GPIOC, &StuGpio);//all PC signal port.
}
//chaojun.liu 20150608 add for smart signal switch end ---.
void Init_I2c_Gpio(void)
{
	GPIO_InitTypeDef StuI2cGpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB, ENABLE);
    /* I2C1 SDA and SCL configuration */
    StuI2cGpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    StuI2cGpio.GPIO_Speed = GPIO_Speed_50MHz;
    StuI2cGpio.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &StuI2cGpio);
}

   
/* End of File ****************************************************************/
