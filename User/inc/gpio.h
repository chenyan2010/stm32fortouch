/**
  ******************************************************************************
  * @file    gpio.h
  * @author  rui.zhao
  * @version 2.00
  * @date    20130703 
  * @brief   This file contains definitions for STM32F103xx Leds, push-buttons
  *          and switch ports hardware resources.
  ******************************************************************************
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
/* Includes ------------------------------------------------------------------*/

#include "global_data.h"

/* Exported variables --------------------------------------------------------*/
 

#if (((X_POINT_SIZE)%8) == 0)
  #define X_CLUMN_SIZE		(X_POINT_SIZE/8)
#else
  #define X_CLUMN_SIZE		((X_POINT_SIZE/8)+1) 
#endif

#if (((Y_POINT_SIZE)%8) == 0)
  #define Y_CLUMN_SIZE		(Y_POINT_SIZE/8)
#else
  #define Y_CLUMN_SIZE		((Y_POINT_SIZE/8)+1) 
#endif

	 
/**
 * @brief Resistor set
 */

#define Ecurrent_1			(0x0001)
#define Ecurrent_2			(0x0002)
#define Ecurrent_4			(0x0004)
#define Ecurrent_8			(0x0008)
#define Ecurrent_16			(0x0010)
#define Ecurrent_32			(0x0020)
#define Ecurrent_64			(0x0040)
#define Ecurrent_128		(0x0080)
#define Ecurrent_256		(0x0100)
#define Ecurrent_512		(0x0200)
#define Ecurrent_1024		(0x0400)
#define Ecurrent_2048		(0x0800)
#define Ecurrent_None		(0x0000)


/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  LED enumeration  
  */typedef enum 
{
  	LED1 = 0,
	LED2 = 1
} LED_TypeDef;

/** 
  * @brief  Trans Resistor, Trans Clumn and Receive Clumn enumeration  
  */
typedef enum 
{
  	R_GROUP = 0,
	TRANS_GROUP = 1,
	REV_GROUP   = 2
} IrOPE_TypeDef;



//recerve point status,

typedef enum
{
	RP_OFF   = 0,
	RP_ON    = 1,
} RPStatus_TypeDef;


/** 
  * @brief  Turn Off and Turn On enumeration  
  */
typedef enum
{
	TURN_OFF = 0,
	TURN_ON  = !TURN_OFF
} TurnAction;


/* Exported constants --------------------------------------------------------*/
/**
 * @brief LED
 */
#define LED_GROUP				(2)

#define LED1_PIN				GPIO_Pin_10
#define LED1_PORT				GPIOB
#define LED1_CLK				RCC_APB2Periph_GPIOB

#define LED2_PIN				GPIO_Pin_1
#define LED2_PORT				GPIOA
#define LED2_CLK				RCC_APB2Periph_GPIOA


/**
 * @brief HC164 control group number
 */
#define CTR_GROUP_NMB			(3)

/**
 * @brief HC164 IrDA Trans Resistor control GPIO
 */
#define TRohm_164D_PIN			GPIO_Pin_7
#define TRohm_164D_PORT			GPIOA
#define TRohm_164D_CLK			RCC_APB2Periph_GPIOA


#ifndef	USE_ZW_BOARD
#define TRohm_164CP_PIN			GPIO_Pin_6
#else
#define TRohm_164CP_PIN			GPIO_Pin_5
#endif


#define TRohm_164CP_PORT		GPIOA
#define TRohm_164CP_CLK			RCC_APB2Periph_GPIOA


/**
 * @brief HC164 IrDA Trans Clumn control GPIO
 */
#if    1//def USE_UART2_FUNCTION 
#define TransClumn_164D_PIN		GPIO_Pin_1
#else
#define TransClumn_164D_PIN		GPIO_Pin_3
#endif

#define TransClumn_164D_PORT	GPIOA
#define TransClumn_164D_CLK		RCC_APB2Periph_GPIOA

#define TransClumn_164CP_PIN	GPIO_Pin_4
#define TransClumn_164CP_PORT	GPIOA
#define TransClumn_164CP_CLK	RCC_APB2Periph_GPIOA


/**
 * @brief HC164 IrDA Receive Clumn control GPIO
 */
#define RevClumn_164D_PIN		GPIO_Pin_15
#define RevClumn_164D_PORT		GPIOB
#define RevClumn_164D_CLK		RCC_APB2Periph_GPIOB

#define RevClumn_164CP_PIN		GPIO_Pin_8
#define RevClumn_164CP_PORT		GPIOA
#define RevClumn_164CP_CLK		RCC_APB2Periph_GPIOA


/**
 * @brief IrDA trans data Row select (HC138)
 */
#ifndef USE_ZW_BOARD 
#define TransRow_138A_PIN		GPIO_Pin_13
#define TransRow_138B_PIN		GPIO_Pin_14
#define TransRow_138C_PIN		GPIO_Pin_15
#else
#define TransRow_138A_PIN		GPIO_Pin_15
#define TransRow_138B_PIN		GPIO_Pin_14
#define TransRow_138C_PIN		GPIO_Pin_13
#endif

#define TransRow_138_PORT		GPIOC
#define TransRow_138_CLK		RCC_APB2Periph_GPIOC

#define TransRow_138CE_PIN		GPIO_Pin_9
#define TransRow_138CE_PORT		GPIOB
#define TransRow_138CE_CLK		RCC_APB2Periph_GPIOB


/**
 * @brief IrDA receive data Row select (CD4051)
 */
#define RevRow_4051A_PIN		GPIO_Pin_14
#define RevRow_4051B_PIN		GPIO_Pin_13
#define RevRow_4051C_PIN		GPIO_Pin_12
#define RevRow_4051_PORT		GPIOB
#define RevRow_4051_CLK			RCC_APB2Periph_GPIOB
/**
 * @brief all signal port select chaojun.liu 20150608 add for smart signal switch start +++.
 */
 //PB input mode

#define HDMI1_CHK_IN   GPIO_Pin_1
#define HDMI2_CHK_IN   GPIO_Pin_3
//#define HDMI3_CHK_IN   GPIO_Pin_13
#define HDMI4_CHK_IN   GPIO_Pin_5
#define HDMI5_CHK_IN   GPIO_Pin_8
#define HDMI6_CHK_IN   GPIO_Pin_9
#define VGA1_CHK_IN    GPIO_Pin_10
#define VGA2_CHK_IN    GPIO_Pin_11
#define VGA3_CHK_IN    GPIO_Pin_12
#define AV1_CHECK_IN   GPIO_Pin_13
#define AV2_CHECK_IN   GPIO_Pin_14
#define AV3_CHECK_IN   GPIO_Pin_15
#define SVIDEO_CHK_IN  GPIO_Pin_13 //unknow
#define YPBPR_CHK_IN   GPIO_Pin_13 //unknow

#define All_PB_CHK_IN  (AV1_CHECK_IN|AV2_CHECK_IN|YPBPR_CHK_IN|HDMI1_CHK_IN|HDMI2_CHK_IN|HDMI4_CHK_IN|HDMI5_CHK_IN|HDMI6_CHK_IN|VGA1_CHK_IN|VGA2_CHK_IN|VGA3_CHK_IN)
//PC13 input mode  
#define HD_CHK_IN      GPIO_Pin_13
	//PE5 input mode.
//#define HDMI0_CHK_IN

    //PF4 output mode.
#define RESET_CTR_O
//chaojun.liu 20150608 add for smart signal switch end ---.
/*define for smart signal switch function*/
#define REGADDR_OPEN_TOUCH       0X00             //write this register to open touchscreen.
#define REGADDR_CLOSE_TOUCH      0X01             //write this register to close touchscreen.
#define REGADDR_SEND_SHORTCUT    0X02             //write this register to send shortcut.
#define REGADDR_HASSIGNAL     0X03             //write this register to notice STM8 current chanel has signal,don't need check. 
#define REGADDR_CLOSED        0x04             //write this register to notice STM8 signal switch function has closed.
#define REGADDR_POWERON       0x05             //mst182 notice STM8 it has poweron system value should be reset.
/*define for encryption*/
#define REGADDR_ENCRYPTION_ID1  0x06
#define REGADDR_ENCRYPTION_ID2  0x07
#define REGADDR_ROUTINE1        0x08
#define REGADDR_ROUTINE2        0x09
#define REGADDR_CONFIRM         0x0a

#define REGADDR_UART_CMD1       0x10     //add for 818 customer uart command.
#define REGADDR_UART_CMD2       0x11     //add for 818 customer uart command.

//two bytes mask.
#define HDMI0_CONN_MASK       0X0001
#define HDMI1_CONN_MASK       0X0002
#define HDMI2_CONN_MASK       0X0004
#define HDMI3_CONN_MASK       0X0008
#define HDMI4_CONN_MASK       0X0010
#define HDMI5_CONN_MASK       0X0020
#define HDMI6_CONN_MASK       0X0040
#define HDMI7_CONN_MASK       0X0080
#define YPBPR_CONN_MASK       0X0100
#define AV1_CONN_MASK         0X0200
#define AV2_CONN_MASK         0X0400
#define AV3_CONN_MASK         0X0800
#define VGA1_CONN_MASK        0X1000
#define VGA2_CONN_MASK        0X2000
#define VGA3_CONN_MASK        0X4000
#define VGA4_CONN_MASK        0X8000


typedef enum
{
	UI_INPUT_SOURCE_HDMI,
	UI_INPUT_SOURCE_HDMI1,
	UI_INPUT_SOURCE_HDMI2,
	UI_INPUT_SOURCE_HDMI3,
	UI_INPUT_SOURCE_HDMI4,
	UI_INPUT_SOURCE_HDMI5,
	UI_INPUT_SOURCE_HDMI6,
	UI_INPUT_SOURCE_HDMI7,
	UI_INPUT_SOURCE_COMPONENT,//YPBPR
	UI_INPUT_SOURCE_AV,
	UI_INPUT_SOURCE_AV2,
	UI_INPUT_SOURCE_SVIDEO,
	UI_INPUT_SOURCE_RGB,
	UI_INPUT_SOURCE_RGB1,
	UI_INPUT_SOURCE_RGB2,
	UI_INPUT_SOURCE_RGB4,
	UI_INPUT_SOURCE_NONE
}E_UI_INPUT_SOURCE;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void LED_Init(LED_TypeDef Led);
void LED_On(LED_TypeDef Led);
void LED_Off(LED_TypeDef Led);
void LED_Toggle(LED_TypeDef Led);

void TR_Init(void);
void TR_GPIO_Init(IrOPE_TypeDef IrOpe);
void TR_Send164_1Bit(IrOPE_TypeDef IrOpe, BitAction BitVal);

void Trans_TurnOff(void);
void Trans_TurnOn(void);
void Init_Trans_Point_Set(DIRECTION_TypeDef Direction, uint16_t Point);
void Init_Receive_Point_Set(DIRECTION_TypeDef Direction, uint16_t Point);
void Trans_Point_Set(DIRECTION_TypeDef Direction, uint16_t Point);
void Receive_Point_Set(DIRECTION_TypeDef Direction, uint16_t Point);
void Resistor_Set_12bit(uint16_t OhmVal);
void Init_AVCheck_GPIO(void);//chaojun.liu 20150608 add for smart signal switch .
void Init_I2c_Gpio(void);



#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H */


/* End of File ****************************************************************/
