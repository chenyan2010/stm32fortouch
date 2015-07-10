/**
  ******************************************************************************
  * @file    tim.c
  * @author  Leo
  * @version V1.00
  * @date    20121015 - 
  * @brief   This file provides firmware functions to the TIM
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/


#include <stdarg.h>
#include <stdio.h>

#include "global_data.h"
#include "gpio.h"
#define TIM_GLOBALS
#include "tim.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Configures TIM2.
  * @param  None
  * @retval None
  */
void TIM2_Init(void)
{
	NVIC_InitTypeDef					NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef					TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;

	
	TIM2CCR1_Val = TIM2_LongWait;
	TIM2CCR2_Val = TIM2_ShortWait;
	TIM2CCR3_Val = TIM2_ShortWait;
	TIM2CCR4_Val = 6826;

	
	/* TIM2 clock init **********************************************************/
  /* PCLK1=HCLK/4, TIM clock will be (72MHz/4)*2=36MHz.
     The TIM clock was (72MHz/2)*2=72MHz if not do this. */
//   RCC_PCLK1Config(RCC_HCLK_Div4);
	
  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	
	/* NVIC init ****************************************************************/
  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
	/* ---------------------------------------------------------------
    TIM2 Configuration: Output Compare Timing Mode:
    TIM2 counter clock at 100000 (100KHz, 10us)
    CC1 350us update rate = TIM2 counter clock / TIM2CCR1_350us = 20000/7 times/s
		CC1 50us update rate = TIM2 counter clock / TIM2CCR1_50us = 20000/1 times/s
    CC2 update rate = TIM2 counter clock / CCR2_Val = 10 times/s
    CC3 update rate = TIM2 counter clock / CCR3_Val = 439.4 Hz
    CC4 update rate = TIM2 counter clock / CCR4_Val = 878.9 Hz
  --------------------------------------------------------------- */
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock / TIM2_Pre_0p5us) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
	/* TIM_ClockDivision was already set, but no use. Set it again. */
  TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);


  /* Output Compare Timing Mode configuration: Channel1 ***********************/
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TIM2CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
	
	
  /* Output Compare Timing Mode configuration: Channel2 ***********************/
//   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//   TIM_OCInitStructure.TIM_Pulse = TIM2CCR2_Val;
//   TIM_OC2Init(TIM2, &TIM_OCInitStructure);

//   TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);


  /* Output Compare Timing Mode configuration: Channel3 */
//   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//   TIM_OCInitStructure.TIM_Pulse = TIM2CCR3_Val;
//   TIM_OC3Init(TIM2, &TIM_OCInitStructure);

//   TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);

//   /* Output Compare Timing Mode configuration: Channel4 */
//   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//   TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
//   TIM_OC4Init(TIM2, &TIM_OCInitStructure);

//   TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);


  /* TIM IT enable */
//   TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
// 	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

  /* TIM2 enable counter */
//   TIM_Cmd(TIM2, ENABLE);
}

void TIM3_Init(void)
{
	NVIC_InitTypeDef			NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef			TIM_OCInitStructure;
	uint16_t 					PrescalerValue = 0;

	
	TIM3CCR1_Val = TIM2_LongWait;
	TIM3CCR2_Val = TIM2_ShortWait;
	TIM3CCR3_Val = TIM2_ShortWait;
	TIM3CCR4_Val = 6826;

	
	/* TIM2 clock init **********************************************************/
  /* PCLK1=HCLK/4, TIM clock will be (72MHz/4)*2=36MHz.
     The TIM clock was (72MHz/2)*2=72MHz if not do this. */
//   RCC_PCLK1Config(RCC_HCLK_Div4);
	
  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	
	/* NVIC init ****************************************************************/
  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
	/* ---------------------------------------------------------------
    TIM2 Configuration: Output Compare Timing Mode:
    TIM2 counter clock at 100000 (100KHz, 10us)
    CC1 350us update rate = TIM2 counter clock / TIM2CCR1_350us = 20000/7 times/s
		CC1 50us update rate = TIM2 counter clock / TIM2CCR1_50us = 20000/1 times/s
    CC2 update rate = TIM2 counter clock / CCR2_Val = 10 times/s
    CC3 update rate = TIM2 counter clock / CCR3_Val = 439.4 Hz
    CC4 update rate = TIM2 counter clock / CCR4_Val = 878.9 Hz
  --------------------------------------------------------------- */
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock / TIM2_Pre_0p5us) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
	/* TIM_ClockDivision was already set, but no use. Set it again. */
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);


  /* Output Compare Timing Mode configuration: Channel1 ***********************/
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TIM3CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
	
	
  /* Output Compare Timing Mode configuration: Channel2 ***********************/
//   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//   TIM_OCInitStructure.TIM_Pulse = TIM2CCR2_Val;
//   TIM_OC2Init(TIM2, &TIM_OCInitStructure);

//   TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);


  /* Output Compare Timing Mode configuration: Channel3 */
//   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//   TIM_OCInitStructure.TIM_Pulse = TIM2CCR3_Val;
//   TIM_OC3Init(TIM2, &TIM_OCInitStructure);

//   TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);

//   /* Output Compare Timing Mode configuration: Channel4 */
//   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//   TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
//   TIM_OC4Init(TIM2, &TIM_OCInitStructure);

//   TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);


  /* TIM IT enable */
//   TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
// 	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

  /* TIM2 enable counter */
//   TIM_Cmd(TIM2, ENABLE);
}

   
/* End of File ****************************************************************/
