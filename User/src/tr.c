/**
  ******************************************************************************
  * @file    tr.c
  * @author  rui.zhao
  * @version 2.00
  * @date    20130617 
  * @brief   This file provides firmware functions for multi-touch
  ******************************************************************************
  */  


/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32f10x.h"
#include "global_data.h"

#include "gpio.h"
#include "uart.h"
#include "adc.h"
#include "tim.h"
#include "tr.h"


#ifdef USE_ZW_BOARD
#include "tr_data.inc"
#else
#include "tr_data_big.inc"
#endif

/* Debug message define ------------------------------------------------------------*/

#define TR_DEBUG
#ifdef TR_DEBUG
#undef DEBF
#define  DEBF(fmt, arg...)  printf(fmt, ##arg)
#else
#define DEBF(fmt, arg...)
#endif

//#define INIT_DEBUG
#ifdef INIT_DEBUG
#undef INIT_DEBF
#define  INIT_DEBF(fmt, arg...)  printf(fmt, ##arg)
#else
#define INIT_DEBF(fmt, arg...)
#endif

//#define POINT_DEBUG
#ifdef POINT_DEBUG
#undef POINT_DEBF
#define  POINT_DEBF(fmt, arg...)  printf(fmt, ##arg)
#else
#define POINT_DEBF(fmt, arg...)
#endif

//#define LEAN_DEBUG
#ifdef LEAN_DEBUG
#undef LEAN_DEBF
#define  LEAN_DEBF(fmt, arg...)  printf(fmt, ##arg)
#else
#define LEAN_DEBF(fmt, arg...)
#endif


//#define TRACE_DEBUG
#ifdef TRACE_DEBUG
#undef TRACE_DEBF
#define  TRACE_DEBF(fmt, arg...)  printf(fmt, ##arg)
#else
#define TRACE_DEBF(fmt, arg...)
#endif


#define TR_ERROR
#ifdef TR_ERROR
#undef ERRF
#define  ERRF(fmt, arg...)  printf(fmt, ##arg)
#else
#define ERRF(fmt, arg...)
#endif


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

#define		USE_DYNAMIC_SMOOTH_RATIO_FUNCTION
//#define		USE_FINE_TUNE_SCREEN_FUNCTION

#define		USE_TRACE_CROSS_CHECK_FUNCTION
#define		USE_RESEARCH_FUNCTION
//#define		USE_QUICK_CHECK_POINT_FUNCTION
//#define		USE_CHECK_LEAN_LINE_FUNCTION
//#define		USE_AVERAGE_POINT_SMOOTH_FUNCTION


//#define	NO_TOUCH_RESCAN_TIMES		(3)
//#define 	QUICK_GET_POINT_INTERVAL	(3)
#define 	QUICK_SCAN_EXTEND_SIZE		(10)//(15)
#define 	LEAN_SCAN_LOOP_TIMES		(5)
#define		VS_POINT_MAX_WIDTH			(20)
#define		POINT_MAX_WIDTH				(8)
#define		BIG_POINT_WIDTH				(3)
#define		BIG_POINT_SCAN_STEP			(0)

#define 	XV_INIT_ECUR_VALUE			(20)
#define 	XL_INIT_ECUR_VALUE			(25)
#define 	YV_INIT_ECUR_VALUE			(20)
#define 	YL_INIT_ECUR_VALUE			(25)

#define		BAD_POINT_FLAG				(0x8000)
#define		BAD_POINT_FLAG_UNMASK		(0x7fff)
#define		VBAD_POINT_FLAG				(0x4000)
#define		VBAD_POINT_FLAG_UNMASK		(0xBfff)

#define		LEAN_SCAN_POINT_BUFFER_SIZE	(5)

#define		LED_OFFSET_NMB_X			(33)//(33)
#define		LED_OFFSET_NMB_Y			(39)//(40)

#define 	OVER_CHECK_PRE_LINK_DS		(10)

#define   	PRE_LINK_DS					(1)
#define 	ONE_AREA_PRE_UNLINK_DS		(20)
#define 	ONE_AREA_UNLINK_DS			(50)

#define 	MULTI_AREA_PRE_UNLINK_DS	(40)
#define 	MULTI_AREA_UNLINK_DS		(80)

#ifndef USE_ZW_BOARD

#define 	X_DPI                   		(139)//(32767/X_POINT_SIZE)		//  (138)
#define 	Y_DPI                  			(248)//(32767/Y_POINT_SIZE)		//  (246)

#define 	ECUR_COM_MAX_VALUE				(512)//(768)//(512)// Ecurrent_512 
#define 	ECUR_LIMIT_MAX_VALUE			(768)// Ecurrent_1024

#define 	AD_RES_COM_VALUE				(2048)//(1536)
#define 	AD_NEAR_RES_COM_VALUE			(1800)//(1400)

#define 	AD_Y_RES_COM_VALUE				(2048)//(2048)//(1536)
#define 	AD_Y_NEAR_RES_COM_VALUE			(1800)//(1800)//(1400)

#define 	AD_RES_BAD_CHK_VALUE			(2048)//(1472)
#define 	AD_NEAR_RES_BAD_CHK_VALUE		(1800)//(1300)

#define		LEAN_FAKE_POINT_AD_VALUE		(1472)//(1280)// 1472 -> 0x05c0 (1500)
#define		LEAN_PRE_FAKE_POINT_AD_VALUE	(1024)//(896) // 1024 -> 0x400 (1000)
#define		LEAN_TRUE_POINT_AD_VALUE		(448)//(512)
#define		LEAN_PRE_TRUE_POINT_AD_VALUE	(512)//(640)

#define		VER_X_POINT_MASK_AD_VALUE		(420)//(384)// 0x180//(512)
#define		VER_Y_POINT_MASK_AD_VALUE		(460)//(448)// 0x1c0//(512) 
//#define	LEAN_X_POINT_MASK_AD_VALUE		(600)//(384)// 0x180
//#define	LEAN_Y_POINT_MASK_AD_VALUE		(600)//(448)// 0x1c0
//#define	VER_POINT_PRE_MASK_AD_VALUE	(600)//(512)// 0x200
//#define	VER_POINT_PRE_UNMASK_AD_VALUE	(1900)//(512)// 0x200

#define		DYNAMIC_X_SCAN_MASK_AD_VALUE	(450)//(640)//(700)
#define		DYNAMIC_Y_SCAN_MASK_AD_VALUE	(500)//(640)//(700)

#define 	AD_BAD_POINT_RES_VALUE			(1024)

#else

#define 	X_DPI                   		(139)//(32767/X_POINT_SIZE)		//  (138)
#define 	Y_DPI                  			(246)//(32767/Y_POINT_SIZE)		//  (246)

#define 	ECUR_COM_MAX_VALUE				(1000)
#define 	ECUR_LIMIT_MAX_VALUE			(1024)

#define 	AD_RES_COM_VALUE				(1400)//(2048)//(1536)
#define 	AD_NEAR_RES_COM_VALUE			(1200)//(1800)//(1400)

#define 	AD_Y_RES_COM_VALUE				(1400)//(1000)//(2048)//(1536)
#define 	AD_Y_NEAR_RES_COM_VALUE			(1200)//(900)//(1800)//(1400)

#define 	AD_RES_BAD_CHK_VALUE			(1400)//(1000)//(2048)//(1472)
#define 	AD_NEAR_RES_BAD_CHK_VALUE		(1200)//(900)//(1800)//(1300)

#define		LEAN_FAKE_POINT_AD_VALUE		(900)//(1472)//(1280)// 1472 -> 0x05c0 (1500)
#define		LEAN_PRE_FAKE_POINT_AD_VALUE	(700)//(1024)//(896) // 1024 -> 0x400 (1000)
#define		LEAN_TRUE_POINT_AD_VALUE		(300)//(512)
#define		LEAN_PRE_TRUE_POINT_AD_VALUE	(448)//(640)

#define		VER_X_POINT_MASK_AD_VALUE		(300)// 0x180//(512)
#define		VER_Y_POINT_MASK_AD_VALUE		(350)// 0x1c0//(512) 
//#define	LEAN_X_POINT_MASK_AD_VALUE		(600)//(384)// 0x180
//#define	LEAN_Y_POINT_MASK_AD_VALUE		(600)//(448)// 0x1c0
//#define	VER_POINT_PRE_MASK_AD_VALUE	(600)//(512)// 0x200
//#define	VER_POINT_PRE_UNMASK_AD_VALUE	(1900)//(512)// 0x200

#define		DYNAMIC_X_SCAN_MASK_AD_VALUE	(450)//(640)//(700)
#define		DYNAMIC_Y_SCAN_MASK_AD_VALUE	(500)//(640)//(700)

#define 	AD_BAD_POINT_RES_VALUE			(700)

#endif

#define		AD_DIFF_CHECK_MASK			(0xFFC0) // 1111 1111 1100 0000


#define		CACHE_TRACE_GROUP_SIZE		(MAX_POINT_CNT)// 4 point TraceGroupSize = 24,  5 point TraceGroupSize = 120, 6 point TraceGroupSize = 720,

#define 	MAX_CACHE_BUFFCNT		(5)
#define 	POINT_CACHE_CNT			(MAX_CACHE_BUFFCNT - 1)

#define   	UNKNOW_LINK				(0)
#define   	UNLINK					(0x10)
#define   	PRE_UNLINKED			(0x20)
#define   	PRE_LINKED				(0x40)		
#define   	LINKED					(0x80)

//#define		DYNAMIC_CHECK_VALVE		(1)
//#define		DYNAMIC_CHECK_SKIP_POINT	(2)
//#define		START_SCAN_POINT_NUM		(0)
//#define		END_SCAN_POINT_NUM			(0)
#define 	D_NO_MASK_POINT_CNT			(20)
#define 	V_NO_MASK_POINT_CNT			(10) 


#define 	N_BITS 					(32)
#define 	MAX_BIT 				((N_BITS + 1) / 2 - 1)

//#define 	N_BITS_HALF 			(16)
//#define 	MAX_BIT_HALF			((N_BITS_HALF + 1) / 2 - 1)


#define		GET_DIFF(a, b)			(a > b ? a - b : b - a)

/* Private typedef -----------------------------------------------------------*/
typedef enum 
{
	XR_LEAN = 0x01,
	XL_LEAN = 0x02,
	YR_LEAN = 0x04,
	YL_LEAN = 0x08,
}LeanScanType;

typedef enum 
{
	UNKNOW_POINT = 0x00,
	FAKE_POINT = 0x10,
	PRE_FAKE_POINT = 0x20,
	PRE_TRUE_POINT = 0x30,
	QUICK_PRE_TRUE_POINT = 0x40,
	TRUE_POINT = 0x50,
}FakePoint_Status;

#define POINT_STATUS_MASK	(0xf0)
#define POINT_PRE_STATUS_CNT_MASK	(0x0f)

typedef enum 
{
	KEEP_OFFSET = 0,
	INC_OFFSET = 1,
	DEC_OFFSET = 2,
}BottomTpointAdjust_Status;

typedef enum 
{
	INVALID_MODE = 0,
	CLICK_MODE = 1,
	WRITE_MODE = 2,
	SWITCH_MODE = 3,
	
	MODE_MAX,
}Mode_Status;


typedef enum 
{
	INVALID_STATUS = 0,
	PRESS_STATUS = 2,
	RUNNING_STATUS = 3,
	HALT_STATUS = 4,
	UP_STATUS = 5,
	END_STEP1_STATUS = 6,
	END_STEP2_STATUS = 7,
	END_STEP3_STATUS = 8,
	
	STATUS_MAX,
}Point_Status;

typedef enum 
{
	NORMAL = 0,
	REMOVE = 1,
	NEW = 2,
	IGNORE = 3,
	ATTRI_MAX,
}Point_Attri;


typedef enum 
{
	OLD_POINT_IS_PARENT = 0,
	NEW_POINT_IS_PARENT = 1,
}TraceNodeParentType;

typedef struct _FakePointDataAll_t
{
	uint8_t		trueCnt;
	uint16_t	x_start;
	uint8_t		x_width;
	uint16_t 	y_start;
	uint8_t		y_width;
	uint16_t	ad_value;
	uint16_t	patternS;
	uint8_t		patternSize;
} FakePointDataAll_t;

typedef struct _PointData_t
{
	uint16_t	x;
	uint16_t 	y;
} PointData_t;

typedef struct _GetPoint_t
{
	uint8_t		attri;
	uint8_t 	trueCnt;
	uint16_t 	ad_value;
	uint16_t 	x_position;
	uint16_t 	y_position;
} GetPoint_t;

//  touch attribute  is 1 bytes union
typedef union _TouchData_Attrib_u
{
	uint8_t u8Value;
	struct 
	{
		uint8_t tip_switch : 1;
		uint8_t in_rang : 1;
		uint8_t confidence : 1;
		uint8_t reserved : 5;
	}bits;
}TouchData_Attrib_u;

typedef struct _TouchData_t
{
	TouchData_Attrib_u status;
	uint8_t		attri;
	uint16_t 	x_position;
	uint16_t 	y_position;
} TouchData_t;

typedef struct _TouchCacheData_t
{	
	uint8_t		mode;
	uint8_t		status;
	uint16_t 	x_position;
	uint16_t 	y_position;
} TouchCacheData_t;

typedef struct _TransiDataSt_t
{
	uint8_t 	attib;
	uint8_t 	touch_id;
	uint8_t 	x_L;
	uint8_t 	x_H;
	uint8_t 	y_L;
	uint8_t 	y_H;
}TransiDataSt_t;

typedef struct _LeanScanPointPara_t
{
	uint8_t 	direc;
	uint8_t		type;
	uint16_t	p;
	uint16_t 	patternPtr; 
	uint8_t 	patternBit;
	uint16_t	revP;
	uint16_t	sendP;
	uint16_t    eCur;
} LeanScanPointPara_t;

typedef struct _CrossLine_t
{
	uint8_t		p;
	uint8_t 	top;
	uint8_t		bottom;
} CrossLine_t;

typedef struct _Link_t
{
	uint8_t 	SNode_ID;
	uint32_t 	Distance;
	uint8_t    	S_Link_Flag;	//0: unlink, 1<<8: linked, 1 pre link
}Link_t;

typedef struct _Node_t
{
	uint8_t 	PNode_ID;
	uint8_t    	P_Link_Flag;  //0:un link, 1<<8: linked, others:sun link count
	Link_t 		link[POINT_BUFFER_SIZE];
}Node_t;

typedef struct _Trace_t
{
	uint8_t		link;
	uint8_t 	last;
	uint8_t    	current;
	uint32_t	ds;
}Trace_t;

typedef struct _TracePreCheck_t
{
	uint8_t 	p;
	uint8_t    	s;
	uint8_t 	link;
	uint32_t	ds;
}TracePreCheck_t;

typedef struct _TraceDispersion_t
{	
	uint16_t   	Groupid;
	uint32_t 	Ddis;
}TraceDispersion_t;

typedef struct _TraceDistanceSum_t
{	
	uint16_t   	Groupid;
	uint32_t 	Dsum;
}TraceDistanceSum_t;

typedef struct _PointAd_t
{
	uint16_t	p;
	uint16_t	ad;
} PointAd_t;

typedef struct _FakePointData_t
{
	uint16_t	pointStart;
	uint16_t	pointEnd;
	//uint8_t		center;
} FakePointData_t;

typedef struct _FakeAreaData_t
{
	uint8_t		valid;
	uint8_t		start;
	uint8_t		end;
} FakeAreaData_t;


/* Extern variables ----------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
//uint8_t 	g_u8xDirStep;
//uint8_t 	g_u8yDirStep;

bool		g_bInitCheck;
bool		g_bXGetBadPoint;
bool		g_bYGetBadPoint;
uint16_t	g_u16XBadPointCnt,g_u16YBadPointCnt;

uint8_t		g_u16NextXScanindex;
uint8_t		g_u16NextYScanindex;

uint8_t		g_u8LastDirec;
uint16_t	g_u16LastRevPoint,g_u16LastSendPoint;

// Sample Res-ROhm Buffer
uint16_t 	g_u16XV_EcurBuff[X_POINT_SIZE];
uint16_t 	g_u16XR_Offset_EcurBuff[X_POINT_SIZE];
uint16_t 	g_u16XL_Offset_EcurBuff[X_POINT_SIZE];

uint16_t 	g_u16YV_EcurBuff[Y_POINT_SIZE];
uint16_t 	g_u16YR_Offset_EcurBuff[Y_POINT_SIZE];
uint16_t 	g_u16YL_Offset_EcurBuff[Y_POINT_SIZE];

// receive point init value 
uint16_t 	g_u16XV_ResBuff[X_POINT_SIZE];
//uint16_t 	g_u16XR_Offset_ResBuff[X_POINT_SIZE];
//uint16_t 	g_u16XL_Offset_ResBuff[X_POINT_SIZE];

uint16_t 	g_u16YV_ResBuff[Y_POINT_SIZE];
//uint16_t 	g_u16YR_Offset_ResBuff[Y_POINT_SIZE];
//uint16_t 	g_u16YL_Offset_ResBuff[Y_POINT_SIZE];

uint8_t 	g_u8AdjustScreen;

uint8_t		g_u8ScreenAreaNmb,g_u8CurArea;
uint8_t 	g_u8XTotalAreaCnt;
uint8_t 	g_u8YTotalAreaCnt;
FakeAreaData_t	X_TotalArea[POINT_MARIX_MAX_SIZE];
FakeAreaData_t	Y_TotalArea[POINT_MARIX_MAX_SIZE];

uint8_t 	g_u8XTotalFakePointCnt;
uint8_t 	g_u8YTotalFakePointCnt;
FakePointData_t 	X_TotalFakePosition[POINT_MARIX_MAX_SIZE];
FakePointData_t 	Y_TotalFakePosition[POINT_MARIX_MAX_SIZE];

uint8_t 	g_u8XFakePointCnt;
uint8_t 	g_u8YFakePointCnt;
FakePointData_t 	X_FakePosition[POINT_MARIX_MAX_SIZE];
FakePointData_t 	Y_FakePosition[POINT_MARIX_MAX_SIZE];

uint8_t 	g_u8AllFakePointCnt,g_u8FakePointCnt,g_u8TruePointCnt;
FakePointDataAll_t g_stAllFakePoint[POINT_MARIX_MAX_SIZE][POINT_MARIX_MAX_SIZE];

uint16_t	g_u16FakePatternCnt,g_u16AllFakeDotCnt;
uint8_t 	g_stDotMark[POINT_MARIX_MAX_SIZE][POINT_MAX_WIDTH];

uint8_t		g_u8FakePattern[POINT_MARIX_MAX_SIZE*POINT_MAX_WIDTH];

uint8_t		g_stPatternPreFakeCnt[POINT_MARIX_MAX_SIZE*POINT_MAX_WIDTH][8];
uint16_t	g_stPatternFakeScore[POINT_MARIX_MAX_SIZE*POINT_MAX_WIDTH][8];

uint8_t 	g_u8XR_shiftCnt0[POINT_MARIX_MAX_SIZE];
uint8_t 	g_u8XR_shiftCnt1[POINT_MARIX_MAX_SIZE];
uint8_t 	g_u8XL_shiftCnt0[POINT_MARIX_MAX_SIZE];
uint8_t 	g_u8XL_shiftCnt1[POINT_MARIX_MAX_SIZE];
uint8_t 	g_u8YR_shiftCnt0[POINT_MARIX_MAX_SIZE];
uint8_t 	g_u8YR_shiftCnt1[POINT_MARIX_MAX_SIZE];
uint8_t 	g_u8YL_shiftCnt0[POINT_MARIX_MAX_SIZE];
uint8_t 	g_u8YL_shiftCnt1[POINT_MARIX_MAX_SIZE];

bool		g_bReSearch;
bool		g_bFakeCheckOk,g_bQuickFakeCheck;
bool		g_bOneFakeArea,g_bOneLineFake;
uint8_t		g_u8TrueArea[POINT_MARIX_MAX_SIZE];
uint8_t 	g_u8MaxFake,g_u8MaxTrue,g_u8MarixSize;

PointShift_t g_stDiverScanPara;

uint8_t		g_u8ValidPointCnt;
uint8_t		g_u8TrueCntScore[POINT_MARIX_MAX_SIZE];
uint16_t	g_u16AdScore[POINT_MARIX_MAX_SIZE];

uint8_t		g_u8XR_CrossCnt;
CrossLine_t	g_stXR_Cross[POINT_MARIX_MAX_SIZE * 5];

uint8_t		g_u8XL_CrossCnt;
CrossLine_t	g_stXL_Cross[POINT_MARIX_MAX_SIZE * 5];

uint8_t		g_u8YR_CrossCnt;
CrossLine_t	g_stYR_Cross[POINT_MARIX_MAX_SIZE * 5];

uint8_t		g_u8YL_CrossCnt;
CrossLine_t	g_stYL_Cross[POINT_MARIX_MAX_SIZE * 5];

uint8_t	g_u8XCrossCnt,g_u8YCrossCnt;
PointData_t	g_stXCrossPoint[POINT_MARIX_MAX_SIZE * 5],g_stYCrossPoint[POINT_MARIX_MAX_SIZE * 5];

uint8_t	g_u8XRYRCrossCnt,g_u8XRYLCrossCnt;
PointData_t	g_stXRYRCrossPoint[POINT_MARIX_MAX_SIZE * 5],g_stXRYLCrossPoint[POINT_MARIX_MAX_SIZE * 5];

uint8_t	g_u8XLYRCrossCnt,g_u8XLYLCrossCnt;
PointData_t	g_stXLYRCrossPoint[POINT_MARIX_MAX_SIZE * 5],g_stXLYLCrossPoint[POINT_MARIX_MAX_SIZE * 5];

uint8_t  	g_u8QuickSampleFlag;
LeanScanPointPara_t	g_stShiftLineCache;

uint16_t	g_u16DoubleChkAd0;

uint8_t 		g_u8CurTouchPointCnt;
GetPoint_t 		g_stCurTouchPoint[POINT_BUFFER_SIZE];

uint8_t 		g_u8LastTouchPointCnt;
TouchData_t 	g_stLastTouchPoint[POINT_BUFFER_SIZE];

uint8_t 		g_u8CurTracePointCnt,g_u8LastTracePointCnt;

bool			g_bLinkUp;

uint8_t			g_u8PreUnLinkDs,g_u8UnLinkDs;

uint8_t 		g_u8TraceDire;
uint8_t 		g_u8TraceCnt,g_u8SonPoint;
Node_t  		g_stTraceNode[POINT_BUFFER_SIZE];
Node_t			g_stTraceDS[POINT_BUFFER_SIZE];

uint16_t 		g_u16TraceGroupNmb;//4 
uint16_t 		g_u16TraceGroupNmb;//4 


uint8_t			g_u8PreLinkCnt,g_u8PreUnlinkCnt;
s8				g_s8ReChkPreLinkCnt,g_s8ReChkPreUnlinkCnt;

TracePreCheck_t		g_stPreLink[POINT_MARIX_MAX_SIZE],g_stPreUnlink[POINT_MARIX_MAX_SIZE];
TracePreCheck_t		g_stReChkPreLink[POINT_BUFFER_SIZE],g_stReChkPreUnlink[POINT_BUFFER_SIZE];

TraceDispersion_t 	g_stLinkDdisSortGroup[CACHE_TRACE_GROUP_SIZE];
TraceDistanceSum_t  g_stLinkDsumSortGroup[CACHE_TRACE_GROUP_SIZE];

uint8_t 		g_bTouchBufferRDPtr;
uint8_t 		g_bTouchBufferWRPtr;
uint8_t 		g_bTouchBufferCurPtr;
uint8_t			g_bTouchMode[POINT_BUFFER_SIZE],g_bTouchBufferDsSum[POINT_BUFFER_SIZE],g_bTouchBufferPtrCnt[POINT_BUFFER_SIZE];
uint8_t			g_bTouchDoSmooth[POINT_BUFFER_SIZE];
uint16_t		g_u16InsertX[POINT_BUFFER_SIZE],g_u16InsertY[POINT_BUFFER_SIZE];

uint8_t			g_u8TouchCachePointStatus[POINT_BUFFER_SIZE];
TouchCacheData_t 	g_stTouchDataBuffer[MAX_CACHE_BUFFCNT][POINT_BUFFER_SIZE];

#define  USB_WAITTING_DROP_CNT	(100)

//uint8_t  g_u8UsbWaittingCnt;
volatile uint8_t  g_u8UsbSendBuffer[MULTI_MODE_REPORT_COUNT];


#define  UART_RECEIVE_DATA_LENGTH	(4)
#define  UART_WAITTING_DROP_CNT	(10)




bool g_bUsbSendOpen,g_bUartSendOpen;
//uint8_t  g_u8UartLength;
uint8_t  g_u8UartCmdErrCode;
uint8_t  g_u8UartRevBuffer[8];
uint8_t  g_u8UartSendBuffer[20];

uint8_t  g_u8UartSendCnt;
uint8_t g_u8MaskFlag;

//uint8_t	 g_u8GpioDebug;
//uint16_t debug_before,debug_after;
//uint16_t debug_rev_offset,debug_send_offset,debug_ecur;
//uint16_t 	g_u16TotalDotCnt, g_u16ShiftDotCnt, g_u16GetDotFitCnt;


typedef enum 
{
	MOUSE_MODE = 0x00,
	SINGLE_MODE = 0x01,
	MULTI_MODE = 0x02,

	TOUCH_MODE_MAX,
}TouchModeType;

typedef enum 
{
	NO_KEY = 0x00,
	LEFT_KEY = 0x01,
	RIGHT_KEY = 0x02,
	MID_KEY = 0x03,
	SWITCH_KEY = 0x04,

	SINGLE_KEY_STATUS_MAX,
}SingleModeStatusType;

#define SINGLE_MODE_RIGHT_KEY_RESPOND_CNT		(40)

uint8_t g_u8TouchMode;
uint8_t g_u8SingleModeStatus;
uint8_t g_u8HaltStatusCnt[POINT_BUFFER_SIZE];


typedef enum 
{
	CMD_PC_RESET = 0xa0,
	CMD_PC_START = 0xa1,
	CMD_PC_END = 0xa2,
	CMD_PC_DATA = 0xa3,

	CMD_MCU_ERROR = 0x50,
	CMD_MCU_READY = 0x51,
	CMD_MCU_DO_OK = 0x52,
	CMD_MCU_EXIT = 0x53,
	CMD_MCU_WAIT = 0x54,
	CMD_MCU_ACK = 0x55,
	CMD_MCU_NACK = 0x56,

	CMD_ADJUST_MAX,
}AdjustCmdType;

typedef enum 
{
	ADJUST_NO = 0,
	ADJUST_READY = 1,
	ADJUST_OK = 2,

	ADJUST_MAX,
}AdjustStatusType;

typedef enum 
{
	ADJUST_ERR_ID_CNT = 0xf1,
	ADJUST_ERR_ID = 0xf2,
	ADJUST_ERR_CMD = 0xf3,

	ADJUST_ERR_MAX,
}AdjustErrorType;


//
//		0  ------------------------> 1
//									|	
//									|
//									|
//									|
//
//		3 <------------------------  2
//
//PC Send:
// reset adjust msg
// 0xaa | 1        | 0xa0
// start adjust msg
// 0xaa | 1        | 0xa1
// end adjust msg
// 0xaa | 1        | 0xa2
// data msg                 
// 0xaa | length | 0xa3 | 
//     id | direc   | x diff L | x diff H |  direc   | y diff L |y diff H | 
//     id | direc   | x diff L | x diff H |  direc   | y diff L |y diff H | 
//     ...



//Touch Send:
// adjust error cmd
// 0x55 | 2        | 0x50 | error code
// enter adjust
// 0x55 | 2        | 0x51 | status
// adjust ok
// 0x55 | 2        | 0x52 | status
// adjust exit 
// 0x55 | 2        | 0x53 | status
// adjust wait 
// 0x55 | 2        | 0x54 | status
// adjust ack
// 0x55 | 2        | 0x55 | status
// adjust nack
// 0x55 | 2        | 0x56 | status


typedef struct _PcAdjustData_t
{
	uint8_t		id;
	uint8_t		x_l;
	uint8_t		x_h;
	uint8_t		y_l;
	uint8_t		y_h;
	uint8_t		x_direc;
	uint8_t		x_diff_l;
	uint8_t		x_diff_h;
	uint8_t		y_direc;
	uint8_t 	y_diff_l;	
	uint8_t		y_diff_h;
} PcAdjustData_t;

#define ADJUST_USE_POINT_NMB	(4)
#define USB_REV_BUFFER_LENGTH	(64)
#define ADJUST_REV_MAX_LENGTH	(48+1)
#define ADJUST_SEND_CMD_LENGTH	(5)

uint8_t  g_u8AdjustStatus,g_u8AdjustErrorCode;
s16 	 g_s16AdjustOffsetX,g_s16AdjustOffsetY;
uint8_t  g_u8AdjustBuffer[USB_REV_BUFFER_LENGTH];

uint8_t  g_u8AdjustIdCnt;
uint8_t  g_stAdjustCalFlag[ADJUST_USE_POINT_NMB+1];



/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
void Delay(vu16 cnt)
{
	uint16_t i,j;
	for (i=0;i<cnt;i++)
	{ 
		for (j=0;j<200;j++);
 	}
}

void DelayUS(uint16_t count)
{
	TIM_SetCounter(TIM2, 0);
	TIM_SetCompare1(TIM2, count*2);  
	TIM_ClearFlag(TIM2, TIM_FLAG_CC1);
	TIM_Cmd(TIM2, ENABLE);
	while(TIM_GetFlagStatus(TIM2, TIM_FLAG_CC1) == RESET);
	TIM_Cmd(TIM2, DISABLE);
}

void DelayMS(vu16 cnt)
{
	uint16_t i,j;
	for (i=0;i<cnt;i++)
	{ 
		for (j=0;j<1000;j++);
 	}
}

void StartTimer3	(uint16_t count)
{
	TIM_SetCounter(TIM3, 0);
	TIM_SetCompare1(TIM3, count*2);  
	TIM_ClearFlag(TIM3, TIM_FLAG_CC1);
	TIM_Cmd(TIM3, ENABLE);
}

uint8_t CheckTimer3CurTime(void)
{
	if(TIM_GetFlagStatus(TIM3, TIM_FLAG_CC1) == SET)
	{
		TIM_ClearFlag(TIM3, TIM_FLAG_CC1);
		TIM_Cmd(TIM3, DISABLE);
		return 1;
	}

	return 0;
}

void DisableTimer3(void)
{
	TIM_ClearFlag(TIM3, TIM_FLAG_CC1);
	TIM_Cmd(TIM3, DISABLE);
}

void Init_TouchData(void)
{
	uint16_t i,j; 

	g_u8ScreenAreaNmb = 0;
	g_u8XTotalAreaCnt = 0;
	g_u8YTotalAreaCnt = 0;
	g_u8XTotalFakePointCnt = 0;
	g_u8XTotalFakePointCnt = 0;
		
	for(i = 0; i < POINT_MARIX_MAX_SIZE; i++)
	{
		X_TotalArea[i].start = 0;
		X_TotalArea[i].end = 0;

		Y_TotalArea[i].start = 0;
		Y_TotalArea[i].end = 0;

		X_TotalFakePosition[i].pointStart = 0;
		X_TotalFakePosition[i].pointEnd = 0;
		//X_TotalFakePosition[i].center = 0;

		Y_TotalFakePosition[i].pointStart = 0;
		Y_TotalFakePosition[i].pointEnd = 0;
		//Y_TotalFakePosition[i].center = 0;
	}

	g_u8LastTouchPointCnt = 0;

	g_bTouchBufferRDPtr = 0;
	g_bTouchBufferWRPtr = 0;
	g_bTouchBufferCurPtr = 0;

	for(i = 0; i < POINT_BUFFER_SIZE; i++)
	{
		g_bTouchMode[i] = INVALID_MODE;
		g_u8HaltStatusCnt[i] = 0;
		g_bTouchBufferDsSum[i] = 0; 
		g_bTouchBufferPtrCnt[i] = 0;

		g_stLastTouchPoint[i].status.u8Value = INVALID_STATUS;
		g_stLastTouchPoint[i].attri = NORMAL;
	 	g_stLastTouchPoint[i].x_position = 0;
	 	g_stLastTouchPoint[i].y_position = 0;

		g_u8TouchCachePointStatus[i] = INVALID_STATUS;
		for(j = 0; j < MAX_CACHE_BUFFCNT ; j ++)
		{
			g_stTouchDataBuffer[j][i].mode = INVALID_MODE;
			g_stTouchDataBuffer[j][i].status = INVALID_STATUS;
			g_stTouchDataBuffer[j][i].x_position = 0;
			g_stTouchDataBuffer[j][i].y_position = 0;
		}
	}
}

void Init_FakePointData(void)
{
	uint16_t i,j;

	g_u8XFakePointCnt = 0;
	g_u8YFakePointCnt = 0;
	
	g_u8QuickSampleFlag = 0;

	g_stShiftLineCache.direc = 0;
	g_stShiftLineCache.type = 0;
	g_stShiftLineCache.p = 0;
	g_stShiftLineCache.patternPtr = 0;
	g_stShiftLineCache.patternBit = 0;
	g_stShiftLineCache.revP = 0;
	g_stShiftLineCache.sendP = 0;
	g_stShiftLineCache.eCur = 0;

	g_u8ValidPointCnt = 0;

	for(i = 0; i < POINT_MARIX_MAX_SIZE; i++)
	{
		X_FakePosition[i].pointStart = 0;
		X_FakePosition[i].pointEnd = 0;
		
		Y_FakePosition[i].pointStart = 0;
		Y_FakePosition[i].pointEnd = 0;
		
		g_u8XR_shiftCnt0[i] = 0;
		g_u8XR_shiftCnt1[i] = 1;
		g_u8XL_shiftCnt0[i] = 0;
		g_u8XL_shiftCnt1[i] = 1;
		g_u8YR_shiftCnt0[i] = 0;
		g_u8YR_shiftCnt1[i] = 1;
		g_u8YL_shiftCnt0[i] = 0;
		g_u8YL_shiftCnt1[i] = 1;

		g_u16AdScore[i] = 0xffff;
		g_u8TrueCntScore[i] = 0;
	}

	for(i = 0; i < (POINT_MARIX_MAX_SIZE * POINT_MAX_WIDTH); i++)
	{
		g_u8FakePattern[i] = 0;

		for(j = 0; j < POINT_MAX_WIDTH; j++)
		{
			g_stPatternPreFakeCnt[i][j] = 0;
			g_stPatternFakeScore[i][j] = 0;
		}
	}	
}

void Init_GlobalFakePointData(void)
{
	uint16_t i,j;

	g_u8AllFakePointCnt = 0;
	
	for(i = 0; i < POINT_MARIX_MAX_SIZE; i++)
	{
		for(j = 0; j < POINT_MARIX_MAX_SIZE; j++)
		{
			g_stAllFakePoint[i][j].trueCnt = 0;
			g_stAllFakePoint[i][j].x_start = 0;
			g_stAllFakePoint[i][j].x_width = 0;
			g_stAllFakePoint[i][j].y_start = 0;
			g_stAllFakePoint[i][j].y_width = 0;
			g_stAllFakePoint[i][j].ad_value = 0xffff;
			g_stAllFakePoint[i][j].patternS = 0;
			g_stAllFakePoint[i][j].patternSize = 0;
		}
	}	
}

void Init_PointTraceData(void)
{
	uint16_t i,j;
	
	g_u8PreLinkCnt = 0; 
	g_s8ReChkPreLinkCnt = 0;
	g_u8PreUnlinkCnt = 0;
	g_s8ReChkPreUnlinkCnt = 0;

	for(i = 0; i < POINT_BUFFER_SIZE; i ++)
	{
		g_stCurTouchPoint[i].attri = NORMAL;
		g_stCurTouchPoint[i].trueCnt = 0;
		g_stCurTouchPoint[i].ad_value = 0;
		g_stCurTouchPoint[i].x_position = 0;
		g_stCurTouchPoint[i].y_position = 0;	

		g_stLastTouchPoint[i].attri = NORMAL;

		g_stReChkPreLink[i].p = 0;
		g_stReChkPreLink[i].s = 0;
		g_stReChkPreLink[i].link = 0;
		
		g_stReChkPreUnlink[i].p = 0;
		g_stReChkPreUnlink[i].s = 0;
		g_stReChkPreUnlink[i].link = 0;
		
		g_stTraceNode[i].PNode_ID = 0;
		g_stTraceNode[i].P_Link_Flag = UNLINK;

		g_stTraceDS[i].PNode_ID = 0;
		g_stTraceDS[i].P_Link_Flag = UNLINK;
		
		for(j = 0; j < POINT_BUFFER_SIZE; j ++)
		{
			g_stTraceNode[i].link[j].SNode_ID = 0;
			g_stTraceNode[i].link[j].Distance = 0;
			g_stTraceNode[i].link[j].S_Link_Flag = UNLINK;

			g_stTraceDS[i].link[j].SNode_ID = 0;
			g_stTraceDS[i].link[j].Distance = 0;
			g_stTraceDS[i].link[j].S_Link_Flag = UNLINK;
		}
	}

	for(i = 0; i < POINT_MARIX_MAX_SIZE; i ++)
	{
		g_stPreLink[i].p = 0;
		g_stPreLink[i].s = 0;
		g_stPreLink[i].link = 0;
		g_stPreLink[i].ds = 0;

		g_stPreUnlink[i].p = 0;
		g_stPreUnlink[i].s = 0;
		g_stPreUnlink[i].link = 0;
		g_stPreUnlink[i].ds = 0;
	}

	g_u16TraceGroupNmb = 0;

	for(i = 0; i < CACHE_TRACE_GROUP_SIZE; i ++)
	{
		g_stLinkDdisSortGroup[i].Groupid = 0;
		g_stLinkDdisSortGroup[i].Ddis = 0;

		g_stLinkDsumSortGroup[i].Groupid = 0;
		g_stLinkDsumSortGroup[i].Dsum = 0;
	}
}

uint32_t sqrt_int(uint32_t x)
{
	uint32_t  xroot, m2, x2;

	xroot = 0;
	m2 = 1 << MAX_BIT * 2;

	do
	{
		x2 = xroot + m2;
		xroot >>= 1;

		if (x2 <= x) 
		{ 
			x -= x2; 
			xroot += m2; 
		}
	} while (m2 >>= 2);

	if (xroot < x)
	{
		return xroot + 1;
	}

	return xroot;
} 

uint32_t CalPointDistance(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint32_t x =0,y =0;

	if(x2 <= x1)
		x = (x1 - x2) * (x1 - x2);
	else
		x = (x2 - x1) * (x2 - x1);

	if(y2 <= y1)
		y = (y1 - y2) * (y1 - y2);
	else
		y = (y2 - y1) * (y2 - y1);
	
	x += y;
	
	y = sqrt_int(x);
	
	//DEBF("\r\n CalPointDistance: P(%d,%d), S(%d,%d),  Length: %ld", x1/X_DEBUG_DPI,y1/Y_DEBUG_DPI, x2/X_DEBUG_DPI,y2/Y_DEBUG_DPI, y);
	
	return y;
}

uint8_t sqrt_int_16bit(uint16_t x)
{
	uint32_t  xroot, m2, x2;

	xroot = 0;
	m2 = 1 << MAX_BIT * 2;

	do
	{
		x2 = xroot + m2;
		xroot >>= 1;

		if (x2 <= x) 
		{ 
			x -= x2; 
			xroot += m2; 
		}
	} while (m2 >>= 2);

	if (xroot < x)
	{
		return xroot + 1;
	}

	return xroot;
} 

uint8_t CalPointDistance8Bit(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
	uint16_t x =0,y =0;

	if(x2 <= x1)
		x = (x1 - x2) * (x1 - x2);
	else
		x = (x2 - x1) * (x2 - x1);

	if(y2 <= y1)
		y = (y1 - y2) * (y1 - y2);
	else
		y = (y2 - y1) * (y2 - y1);
	
	x += y;
	
	y = sqrt_int_16bit(x);
	
	//DEBF("\r\n CalPointDistance: P(%d,%d), S(%d,%d),  Length: %ld", x1/X_DEBUG_DPI,y1/Y_DEBUG_DPI, x2/X_DEBUG_DPI,y2/Y_DEBUG_DPI, y);
	
	return y;
}

uint32_t _CalOneGroupTraceDispersion(uint32_t *pDsSt,uint8_t TraceCnt)
{
	uint8_t i;
	uint32_t Dsum =0,Davr=0,Ddis=0;
	
	for(i = 0; i < TraceCnt; i ++)
	{
		Dsum += pDsSt[i];
	}
	
	Davr = Dsum/TraceCnt;
	
	Dsum = 0;
	for(i = 0; i < TraceCnt; i ++)
	{
		if(Davr < pDsSt[i])
			Dsum += (pDsSt[i] - Davr) * (pDsSt[i] - Davr);
		else
			Dsum += (Davr - pDsSt[i]) * (Davr - pDsSt[i]);
	}
	
	Ddis = Dsum / TraceCnt;
	
	return Ddis;
}

// line a : ( x1, y1) - (x2, y2)
// line b : ( x3, y3) - (x4, y4)
bool IsLineSegmentCross(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t x3,uint16_t y3,uint16_t x4,uint16_t y4)
{
	double  x,y;
	/*
	uint16_t x1x2,x1x3,x1x4,x2x4,x3x4;
	uint16_t y1y2,y1y3,y1y4,y2y4,y3y4;
	x1x2 = GET_DIFF(x1,x2);
	x1x3 = GET_DIFF(x1,x3);
	x1x4 = GET_DIFF(x1,x4);
	x2x4 = GET_DIFF(x2,x4);
	x3x4 = GET_DIFF(x3,x4);

	y1y2 = GET_DIFF(y1,y2);
	y1y3 = GET_DIFF(y1,y3);
	y1y4 = GET_DIFF(y1,y4);
	y2y4 = GET_DIFF(y2,y4);
	y3y4 = GET_DIFF(y3,y4);
	*/
		
	if(GET_DIFF(y2,y1) + GET_DIFF(x2,x1) + GET_DIFF(y4,y3) + GET_DIFF(x4,x3) == 0) 
	{  
		if((x3 - x1) + (y3 - y1) == 0) //a,b,c,d is same one point
		{
			//DEBF("\r\n (%d,%d) - (%d,%d), (%d,%d) - (%d,%d), a,b,c,d is same one point !! \n",x1,y1,x2,y2,x3,y3,x4,y4);
		}
		else //a,b is same one point,  c,d is same one point
		{
			//DEBF("\r\n (%d,%d) - (%d,%d), (%d,%d) - (%d,%d), (a-b),(c-d) both same one point !! \n",x1,y1,x2,y2,x3,y3,x4,y4);
		}
		
		return FALSE;
	}  

	if(GET_DIFF(y2,y1) + GET_DIFF(x2,x1) == 0) //a,b is same one point
	{  
		if((x1 - x4) * (y3 - y4) - (y1 - y4) * (x3 - x4) == 0)//a-b at c-d line
		{
			if((x1 - x3) * (x1 - x4) <= 0 && (y1 - y3) * (y1 - y4) <= 0)
			{
				//DEBF("\r\n (%d,%d) - (%d,%d), (%d,%d) - (%d,%d), a,b is same one point, at c-d line and cross !! \n",x1,y1,x2,y2,x3,y3,x4,y4);
				return TRUE;
			}
			else
			{
				//DEBF("\r\n (%d,%d) - (%d,%d), (%d,%d) - (%d,%d), a,b is same one point, at c-d line, not cross !! \n",x1,y1,x2,y2,x3,y3,x4,y4);	
				return FALSE;
			}
		} 
		else //a-b not at c-d line
		{
			//DEBF("\r\n (%d,%d) - (%d,%d), (%d,%d) - (%d,%d), a,b is same one point, not at c-d line !! \n",x1,y1,x2,y2,x3,y3,x4,y4);	
			return FALSE;
		}  
	}

	if(GET_DIFF(y4,y3) + GET_DIFF(x4,x3) == 0) //c,d is same one point
	{  
		if ((x4 - x2) * (y1 - y2) - (y4 - y2) * (x1 - x2) == 0) //c-d at a-b line
		{
			if((x3 - x1) * (x3 - x2) <= 0  && (y3 - y1) * (y3 - y2) <= 0)
			{
				//DEBF("\r\n (%d,%d) - (%d,%d), (%d,%d) - (%d,%d), c,d is same one point, at a-b line and cross !! \n",x1,y1,x2,y2,x3,y3,x4,y4);
				return TRUE;
			}
			else
			{
				//DEBF("\r\n (%d,%d) - (%d,%d), (%d,%d) - (%d,%d), c,d is same one point, at a-b line, not cross !! \n",x1,y1,x2,y2,x3,y3,x4,y4);
				return FALSE;
			}
		} 
		else//c-d not at a-b line
		{
			//DEBF("\r\n (%d,%d) - (%d,%d), (%d,%d) - (%d,%d), c,d is same one point, not at a-b line !! \n",x1,y1,x2,y2,x3,y3,x4,y4);
			return FALSE;
		}  
	}  

	if((y2 - y1) * (x3 - x4) == (x2 - x1) * (y3 - y4))// parallel line
	{
		//DEBF("\r\n (%d,%d) - (%d,%d), (%d,%d) - (%d,%d), parallel line!! \n",x1,y1,x2,y2,x3,y3,x4,y4);
		return FALSE;
	}
	
	x = ((x1 - x2) * ((x3 * y4) - (x4 * y3)) - (x3 - x4) * ((x1 * y2) - (x2 * y1)))  
	   / ((x3 - x4) * (y1 - y2) - (x1 - x2) * (y3 - y4));
	
	y = ((y1 - y2) * ((x3 * y4) - (x4 * y3)) - ((x1 * y2) - (x2 * y1)) * (y3 - y4))  
	   / ((y1 - y2) * (x3 - x4) - (x1 - x2) * (y3 - y4));

	
	if((x - x1) * (x - x2) <= 0  && (x - x3) * (x - x4) <= 0	
		   && (y - y1) * (y - y2) <= 0	&& (y - y3) * (y - y4) <= 0)
	{
		//DEBF("\r\n (%d,%d) - (%d,%d), (%d,%d) - (%d,%d), cross line!! \n",x1,y1,x2,y2,x3,y3,x4,y4);
		return TRUE;
	}
	
	//DEBF("\r\n (%d,%d) - (%d,%d), (%d,%d) - (%d,%d), not cross line!! \n",x1,y1,x2,y2,x3,y3,x4,y4);
	
	return FALSE;
}

//do smooth.
//Coord[3] = ((coord[2] + (coord[2]-(coord[1]+coord[0])/2)*2/3)*2+((coord[2]+coord[1])/2+(coord[4]-(coord[2]+coord[1])/2)*3/5)*2 +coord[3])/5
void DoSmoothPoint(void)
{
	s8  i,j,Index;
	uint8_t	ratio1,ratio2;
	//s32		diffDs = 0;
	uint32_t ds = 0, tmpDs = 0;
	uint16_t R1x = 0, R1y = 0, R2x = 0, R2y = 0, T1x = 0, T1y = 0, T2x = 0, T2y = 0;
	TouchCacheData_t  *pw_data[MAX_CACHE_BUFFCNT];
	
	Index = g_bTouchBufferCurPtr;
	
	for(j = MAX_CACHE_BUFFCNT - 1; j >= 0; j --)
	{
		pw_data[j] = &g_stTouchDataBuffer[Index][0];
		Index --;
		if(Index < 0)
			Index = MAX_CACHE_BUFFCNT - 1;
	}

	for(i = 0; i < TOUCH_DATA_COUNT; i ++)
	{
		if(g_bTouchBufferPtrCnt[i] >= MAX_CACHE_BUFFCNT)
		{
			//if(( (pw_data[3]+i)->status == RUNNING_STATUS || (pw_data[3]+i)->status == HALT_STATUS ) && ((pw_data[4]+i)->status != HALT_STATUS))
			if((pw_data[4]+i)->status == RUNNING_STATUS || (pw_data[4]+i)->status == HALT_STATUS)
			{
				//DEBF("\r\n Do Smooth point:%d \n",i);

				//(coord[1]+coord[0])/2
				R1x = ((pw_data[0]+i)->x_position + (pw_data[1]+i)->x_position) >> 1;
				R1y = ((pw_data[0]+i)->y_position + (pw_data[1]+i)->y_position) >> 1;

				//(coord[2]+coord[1])/2
				R2x = ((pw_data[2]+i)->x_position + (pw_data[1]+i)->x_position) >> 1;
				R2y = ((pw_data[2]+i)->y_position + (pw_data[1]+i)->y_position) >> 1; 
				
				//ds(R1 - P2) * 2/3
				ds = CalPointDistance8Bit(R1x&0x00ff, R1y&0x00ff, ((pw_data[2]+i)->x_position)&0x00ff, ((pw_data[2]+i)->y_position)&0x00ff);
				ratio1 = 2;

				#ifdef USE_DYNAMIC_SMOOTH_RATIO_FUNCTION
				tmpDs = CalPointDistance8Bit(R1x&0x00ff, R1y&0x00ff, ((pw_data[3]+i)->x_position)&0x00ff, ((pw_data[3]+i)->y_position)&0x00ff);
				if(ds == tmpDs)
					ratio1 = 0;
				else if (ds > tmpDs)
				{
					ratio1 = ds - tmpDs;
				}
				else
				{
					ratio1 = tmpDs - ds;
				}
				ds = ratio1 >> 1;
				#if 0
				if( tmpDs <= ds)
				{
					ratio1 = 0;
					DEBF("\r\n --- %s(%d)--ratio1 = 0 \n",__FUNCTION__,__LINE__);
				}
				else
				{
					diffDs = tmpDs - ds;
					DEBF("\r\n --- %s(%d)-ratio1-diffDs:%d \n",__FUNCTION__,__LINE__,diffDs);

					 if(diffDs < 30)// 30
					 {
						ratio1 = 1;
					 }
					 else if(diffDs > 130)// 130
					 {
						ratio1 = 3;
					 }
 				}
				#endif
				
				#else
				ds = ds * ratio1 / 3;
				#endif
				if((pw_data[2]+i)->x_position >= R1x)
					T1x = (pw_data[2]+i)->x_position + ds;
				else
					T1x = (pw_data[2]+i)->x_position - ds;
				
				if((pw_data[2]+i)->y_position >= R1y)
					T1y = (pw_data[2]+i)->y_position + ds;
				else
					T1y = (pw_data[2]+i)->y_position - ds;
				
				//ds(R2 - P4) * 3/5
				ds = CalPointDistance8Bit(R2x&0x00ff, R2y&0x00ff, ((pw_data[4]+i)->x_position)&0x00ff, ((pw_data[4]+i)->y_position)&0x00ff);
				ratio2 = 3;

				#if 0
				#ifdef USE_DYNAMIC_SMOOTH_RATIO_FUNCTION
 				tmpDs = CalPointDistance8Bit(((pw_data[4]+i)->x_position)&0x00ff, ((pw_data[4]+i)->y_position)&0x00ff, ((pw_data[3]+i)->x_position)&0x00ff, ((pw_data[3]+i)->y_position)&0x00ff);

				if(tmpDs <= ds)
				{
					ratio2 = 0;
					DEBF("\r\n --- %s(%d)--ratio2 = 2 \n",__FUNCTION__,__LINE__);
				}
				else
				{
					diffDs = tmpDs - ds;
					DEBF("\r\n --- %s(%d)-ratio2-diffDs:%d \n",__FUNCTION__,__LINE__,diffDs);

					 if(diffDs < 10)
					 {
					    ratio2 = 1;
					 }
					 else if(diffDs < 30)
					 {
						ratio2 = 2;
					 }
					 else if(diffDs > 140)
					 {
						ratio2 = 4;
					 }
 				}
				#endif
				ds = ds * ratio2 / 5;
				
				if(R2x <= (pw_data[4]+i)->x_position)
					T2x = R2x + ds;
				else
					T2x = R2x - ds;
				
				if(R2y <= (pw_data[4]+i)->y_position)
					T2y = R2y + ds;
				else
					T2y = R2y - ds;
				#endif

				#ifdef USE_DYNAMIC_SMOOTH_RATIO_FUNCTION
 				//tmpDs = CalPointDistance8Bit(R2x&0x00ff, R2y&0x00ff, ((pw_data[3]+i)->x_position)&0x00ff, ((pw_data[3]+i)->y_position)&0x00ff);
				tmpDs = CalPointDistance8Bit(R2x&0x00ff, R2y&0x00ff, T1x&0x00ff, T1y&0x00ff);
				if(ds == tmpDs)
					ratio2 = 0;
				else if (ds > tmpDs)
					ratio2 = ds - tmpDs;
				else
					ratio2 = tmpDs - ds;
				#if 0
				if(ds <= tmpDs)
				{
					ratio2 = 0;
					DEBF("\r\n --- %s(%d)--ratio2 = 0 \n",__FUNCTION__,__LINE__);
				}
				else
				{
					diffDs = ds - tmpDs;
					DEBF("\r\n --- %s(%d)-ratio2-diffDs:%d \n",__FUNCTION__,__LINE__,diffDs);

					 if(diffDs < 10)// 10
					 {
					    ratio2 = 1;
					 }
					 else if(diffDs < 30)// 30
					 {
						ratio2 = 2;
					 }
					 else if(diffDs > 120)// 130
					 {
						ratio2 = 4;
					 }
 				}
				#endif
					ds = ratio2 >> 1;
				#else
					ds = ds * ratio2 / 5;
				#endif
				if(R2x <= (pw_data[4]+i)->x_position)
					T2x = R2x + ds;
				else
					T2x = R2x - ds;
				
				if(R2y <= (pw_data[4]+i)->y_position)
					T2y = R2y + ds;
				else
					T2y = R2y - ds;
				
				#if 0  /*zhaorui 131011 +*/ 
				ratio2 = 5 - ratio2;

				ds = ds * ratio2 / 5;
				
				if(R2x <= (pw_data[4]+i)->x_position)
					T2x = (pw_data[4]+i)->x_position - ds;
				else
					T2x = (pw_data[4]+i)->x_position + ds;
				
				if(R2y <= (pw_data[4]+i)->y_position)
					T2y = (pw_data[4]+i)->y_position - ds;
				else
					T2y = (pw_data[4]+i)->y_position + ds;
				#endif  /*  #if 0 -*/

				//T1 : T2 : x -> 2 : 2 : 1
				(pw_data[3]+i)->x_position = ((T1x * 2) + (T2x * 2) + (pw_data[3]+i)->x_position) / 5;
				(pw_data[3]+i)->y_position = ((T1y * 2) + (T2y * 2) + (pw_data[3]+i)->y_position) / 5;
			}
		}
	}
}

void UART_SendChar(USART_TypeDef* USARTx, uint8_t *buff, uint8_t lenght)
{
	uint8_t i;// cnt;

	for(i = 0; i < lenght; i ++)
	{
		//cnt = 0;
		
   		USARTx->DR = buff[i];
		
		while (!(USARTx->SR & USART_FLAG_TC));
		#if 0  /*zhaorui 131023 +*/ 
		{			
			cnt ++;

			if(cnt > UART_WAITTING_DROP_CNT)
			{
				ERRF("\r\n >>> UART Send waitting cnt:%d, drop data! \n",cnt);
				return;
			}

			//DEBF("\r\n >>> UART Send waitting cnt:%d \n",cnt);
		}
		#endif  /*  #if 0 -*/
	}
}

void UART_SendCMD(uint8_t cmd)
{
	uint8_t j;

	j = 0;
	
	g_u8UartSendBuffer[j++] = 0x56;
	g_u8UartSendBuffer[j++] = 0x56;

	if(cmd == 0x60)
	{
		g_u8UartSendBuffer[j++] = 2;
		g_u8UartSendBuffer[j++] = cmd;
		g_u8UartSendBuffer[j++] = g_u8UartCmdErrCode;
	}
	else
	{
		g_u8UartSendBuffer[j++] = 1;
		g_u8UartSendBuffer[j++] = cmd;
	}

	UART_SendChar(MCU_COM_UART, g_u8UartSendBuffer, g_u8UartSendBuffer[2]+3);
}


void UART_SendTouchData(void)
{
	uint8_t 	i,k,touchCnt;
	volatile    uint16_t	dpitmp;

	k = 0;
	touchCnt = 0;

	g_u8UartSendBuffer[k++] = 0x56;
	g_u8UartSendBuffer[k++] = 0x56;
	g_u8UartSendBuffer[k++] = 8;
	g_u8UartSendBuffer[k++] = 0x62;

	for(i = 0; i < POINT_BUFFER_SIZE; i ++)
	{
		if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status != INVALID_STATUS)
		{
			if((g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == RUNNING_STATUS) || (g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == HALT_STATUS))
			{
				g_u8UartSendBuffer[k++] = 3;
			}
			else
			{
				g_u8UartSendBuffer[k++] = 2;
			}

			g_u8UartSendBuffer[k++] = i;//touch id

			dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].x_position;
			g_u8UartSendBuffer[k++] = dpitmp& 0xff;
			g_u8UartSendBuffer[k++] = dpitmp >> 8;

			dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].y_position;
			g_u8UartSendBuffer[k++] = dpitmp& 0xff;
			g_u8UartSendBuffer[k++] = dpitmp >> 8;

			touchCnt ++;

			break;
		}
	}

	g_u8UartSendBuffer[k++] = g_u8UartSendCnt;
	
#if 0
		DEBF("\r\n");
		DEBF("\r\n >>> UART Send Cnt: %d",g_u8UartSendCnt);
		DEBF("\r\n");
		for(i = 0; i < k; i ++)
		{
			DEBF(" 0x%02x ",g_u8UartSendBuffer[i]);
		}
#endif

	if(touchCnt > 0)
	{	
		UART_SendChar(MCU_COM_UART, g_u8UartSendBuffer, k);
	}

	g_u8UartSendCnt ++;
}


bool USB_SendIsValid(void)
{
	uint8_t cnt;

	if(g_bUsbSendOpen == FALSE)
	{
		return FALSE;
	}
	
	cnt = 0;
	
	while(GetEPTxStatus(ENDP1) != EP_TX_NAK)
	{
		cnt ++;

		if(cnt > USB_WAITTING_DROP_CNT)
		{
			SetEPTxValid(ENDP1);
			//ERRF("\r\n ### USB Send waitting cnt:%d, drop data! \n",cnt);
			return FALSE;
		}
		//DEBF("\r\n USB Send waitting... cnt:%d \n",cnt);
		DelayMS(1);
	};

	return TRUE;
}

void adjustInitData(void)
{
	uint8_t i;
	
	g_u8AdjustIdCnt = 0;
	g_u8AdjustErrorCode = 0;
	g_s16AdjustOffsetX = 0;
	g_s16AdjustOffsetY = 0;

	for(i = 0; i < ADJUST_USE_POINT_NMB; i ++)
	{
		g_stAdjustCalFlag[i] = 0;
	}
}

void adjustParseOffset(void)
{
	uint8_t i,j,get,totalGet;
	s16 x_total_diff,y_total_diff;
	PcAdjustData_t	*ptr;

	g_u8AdjustErrorCode = 0;

	totalGet = 0;
	x_total_diff = 0;
	y_total_diff = 0;
	
	ptr = (PcAdjustData_t *)(&g_u8AdjustBuffer[5]);
	
	for(i = 0; i < ADJUST_USE_POINT_NMB; i ++)
	{
		get = 0;
		for(j = 0; j < ADJUST_USE_POINT_NMB; j ++)
		{
			//DEBF("\r\n adjust-CalOffset: id:%d, j:%d, flag:%d  \n",ptr->id, j, g_stAdjustCalFlag[j]);
			if((ptr->id == j) && (g_stAdjustCalFlag[j] == 0))
			{
				get = 1;
				g_stAdjustCalFlag[j] = 1;

				if(ptr->x_direc == 0)
				{
					x_total_diff += ((ptr->x_diff_h << 8) + ptr->x_diff_l);
				}
				else if(ptr->x_direc == 1)
				{
					x_total_diff -= ((ptr->x_diff_h << 8) + ptr->x_diff_l);
				}
				
				if(ptr->y_direc == 0)
				{
					y_total_diff += ((ptr->y_diff_h << 8) + ptr->y_diff_l);
				}
				else if(ptr->y_direc == 1)
				{
					y_total_diff -= ((ptr->y_diff_h << 8) + ptr->y_diff_l);
				}
				
				totalGet ++;
				
				break;
			}
		}

		if(get != 1)
		{
			g_u8AdjustErrorCode = ADJUST_ERR_ID;
			
			//DEBF("\r\n adjust-CalOffset: totalGet:%d, error ID:%d \n",totalGet,ptr->id);
			return;
		}

		ptr ++;

		//DEBF("\r\n adjust-CalOffset: totalGet:%d, id:%d \n",totalGet,ptr->id);
	}

	if(totalGet != ADJUST_USE_POINT_NMB)
	{
		g_u8AdjustErrorCode = ADJUST_ERR_ID_CNT;
		//DEBF("\r\n adjust-CalOffset: error totalGet:%d \n",totalGet);
		return;
	}

	g_s16AdjustOffsetX = x_total_diff >> 2; //g_s16AdjustOffsetX = x_total_diff / ADJUST_USE_POINT_NMB;
	g_s16AdjustOffsetY = y_total_diff >> 2;

	g_u8AdjustStatus = ADJUST_OK;

	//DEBF("\r\n adjust-CalOffset: X:%d, Y:%d \n",g_s16AdjustOffsetX, g_s16AdjustOffsetY);
}

void adjustSendCmd(uint8_t cmd)
{
	uint8_t j;

	j = 0;
	
	g_u8AdjustBuffer[j++] = 0xfc;
	g_u8AdjustBuffer[j++] = 0x55;
	g_u8AdjustBuffer[j++] = 0x55;
	g_u8AdjustBuffer[j++] = 2;
	g_u8AdjustBuffer[j++] = cmd;

	if(cmd != CMD_MCU_ERROR)
		g_u8AdjustBuffer[j++] = g_u8AdjustStatus;
	else
		g_u8AdjustBuffer[j++] = g_u8AdjustErrorCode;

	//DEBF("\r\n adjust-SendCmd: 0x%x 0x%x 0x%x 0x%x 0x%x \n",g_u8AdjustBuffer[1],g_u8AdjustBuffer[2],g_u8AdjustBuffer[3],g_u8AdjustBuffer[4],g_u8AdjustBuffer[5]);

	if(USB_SendIsValid() == TRUE)
	{
		USB_SIL_Write(EP1_IN, (uint8_t *)g_u8AdjustBuffer, 64);
		SetEPTxValid(ENDP1);
	}
}


void adjustGetCmd(void)
{
	uint8_t i;
	uint16_t cmd;
	
	if(_GetEPRxStatus(ENDP1) == EP_RX_NAK)
	{
		//DEBF("\r\n adjust-GetCmd, CurAdjustStatus:%d \n", g_u8AdjustStatus);
		
		for(i = 0; i < ADJUST_REV_MAX_LENGTH; i ++)
		{
			g_u8AdjustBuffer[i] = 0;
		}
		
		USB_SIL_Read(EP1_OUT, g_u8AdjustBuffer);
		SetEPRxStatus(ENDP1, EP_RX_VALID);

		//DEBF("\r\n adjust-GetCmd: 0x%X 0x%X, Length:%d, cmd:0x%X \n",g_u8AdjustBuffer[1],g_u8AdjustBuffer[2],g_u8AdjustBuffer[3],g_u8AdjustBuffer[4]);
		//DEBF("\r\n data:\n");
		//for(i = 5; i < 5 + g_u8AdjustBuffer[3] - 1; i ++)
		//{
			//DEBF("0x%x ",g_u8AdjustBuffer[i]);
		//}
		//DEBF("\r\n \n");

		cmd = (g_u8AdjustBuffer[1] << 8) + g_u8AdjustBuffer[2];

		if(cmd == 0xaaaa)
		{
			switch(g_u8AdjustBuffer[4])//cmd type
			{
				case CMD_PC_RESET:
					adjustInitData();
					g_u8AdjustStatus = ADJUST_NO;
					adjustSendCmd(CMD_MCU_DO_OK);
					break;
					
				case CMD_PC_START:
					adjustInitData();
					g_u8AdjustStatus = ADJUST_READY;
					adjustSendCmd(CMD_MCU_READY);
					break;
					
				case CMD_PC_DATA:
					if(g_u8AdjustStatus == ADJUST_READY && (g_u8AdjustBuffer[3] == ADJUST_REV_MAX_LENGTH - 4))
					{
						adjustParseOffset();
						if(g_u8AdjustStatus == ADJUST_OK)
						{
							adjustSendCmd(CMD_MCU_DO_OK);
						}
						else
						{
							adjustSendCmd(CMD_MCU_ERROR);
						}
					}
					else
					{
						g_u8AdjustErrorCode = ADJUST_ERR_CMD;
						adjustSendCmd(CMD_MCU_ERROR);
					}
					break;
					
				case CMD_PC_END:
					if(g_u8AdjustStatus == ADJUST_OK || g_u8AdjustStatus == ADJUST_NO)
					{
						adjustSendCmd(CMD_MCU_EXIT);
					}
					else
					{
						g_u8AdjustErrorCode = ADJUST_ERR_CMD;
						adjustSendCmd(CMD_MCU_ERROR);
					}
					break;
					
				default:
					g_u8AdjustErrorCode = ADJUST_ERR_CMD;
					adjustSendCmd(CMD_MCU_ERROR);
					break;
			}

			if(g_u8AdjustErrorCode != 0)
			{
				adjustInitData();
				g_u8AdjustStatus = ADJUST_NO;
			}
		}
	}
}


void adjustScreenFun(void)
{
	switch(g_u8AdjustStatus)
	{
		case ADJUST_OK:
		case ADJUST_NO:	
		case ADJUST_READY:
			adjustGetCmd();
			break;
		default:
			DEBF("\r\n g_u8AdjustStatus:%d, error case!! \n", g_u8AdjustStatus);
			break;
	}
}


void USB_SingleModeSendTouchEndCmd(void)
{
	uint8_t 	i,j,touchCnt;
	uint16_t    dpitmp;
	
	j = 0;
	touchCnt = 0;

	g_u8UsbSendBuffer[j++] = SINGLE_MODE_REPORT_ID;

	for(i = 0; i < POINT_BUFFER_SIZE; i++)
	{
		if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status != INVALID_STATUS)
		{
			g_u8UsbSendBuffer[j++] = 0;// key = 0
			
			dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].x_position;
			g_u8UsbSendBuffer[j++] = dpitmp & 0xff;
			g_u8UsbSendBuffer[j++] = dpitmp >> 8;

			dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].y_position;
			g_u8UsbSendBuffer[j++] = dpitmp & 0xff;
			g_u8UsbSendBuffer[j++] = dpitmp >> 8;
			
			g_u8UsbSendBuffer[j++] = 0;// status = 0
			
			touchCnt ++;

			break;
		}
	}

	if(touchCnt > 0)
	{
		g_u8UsbSendBuffer[j++] = 0;
		
		if(USB_SendIsValid() == TRUE)
		{
			USB_SIL_Write(EP1_IN, (uint8_t *)g_u8UsbSendBuffer, SINGLE_MODE_REPORT_COUNT);  
			SetEPTxValid(ENDP1);
		}
	#if 0
		printf("\r\n");
		printf("\r\n ##########SingleMode-End: ");
		printf("\r\n");
		for(i = 0; i < SINGLE_MODE_REPORT_COUNT; i ++)
		{
			printf(" 0x%02X ",g_u8UsbSendBuffer[i]);
		}
		printf("\r\n");
	#endif
	}
}
// report id, status, x, y, status
void USB_SingleModeSendString(void)
{
	uint8_t 	i,j,touchCnt,ptr;
	uint16_t    dpitmp;
	
	j = 0;
	touchCnt = 0;

	g_u8UsbSendBuffer[j++] = SINGLE_MODE_REPORT_ID;
	
	for(i = 0; i < POINT_BUFFER_SIZE; i++)
	{
		if(g_bTouchMode[i] == INVALID_MODE)
			continue;
		
		//DEBF("\r\n Multi touch USB Send P:%d, mode:%d \n",i, g_bTouchMode[i]);
		
		if(g_bTouchMode[i] == CLICK_MODE)
			ptr = g_bTouchBufferCurPtr;
		else
			ptr = g_bTouchBufferRDPtr;

		if(g_stTouchDataBuffer[ptr][i].status != INVALID_STATUS)
		{
			if(g_u8HaltStatusCnt[i] > SINGLE_MODE_RIGHT_KEY_RESPOND_CNT)
			{
				switch(g_u8SingleModeStatus)
				{
					case LEFT_KEY:
						g_u8SingleModeStatus = SWITCH_KEY;
						g_u8UsbSendBuffer[j++] = 0;
						break;
					case SWITCH_KEY:
						g_u8SingleModeStatus = RIGHT_KEY;
						g_u8UsbSendBuffer[j++] = 2;
						break;
					case RIGHT_KEY:
						g_u8SingleModeStatus = NO_KEY;
						g_u8UsbSendBuffer[j++] = 0;
						break;
					case NO_KEY:// while(1)
						g_u8SingleModeStatus = NO_KEY;
						g_u8UsbSendBuffer[j++] = 0;
						break;	
					default:
						g_u8SingleModeStatus = LEFT_KEY;
						g_u8UsbSendBuffer[j++] = 1;
						break;
				}
			}
			else
			{
				g_u8SingleModeStatus = LEFT_KEY;
				g_u8UsbSendBuffer[j++] = 1;
			}
			
			dpitmp = g_stTouchDataBuffer[ptr][i].x_position;
			g_u8UsbSendBuffer[j++] = dpitmp& 0xff;
			g_u8UsbSendBuffer[j++] = dpitmp >> 8;

			dpitmp = g_stTouchDataBuffer[ptr][i].y_position;
			g_u8UsbSendBuffer[j++] = dpitmp& 0xff;
			g_u8UsbSendBuffer[j++] = dpitmp >> 8;
			
			if(g_stTouchDataBuffer[ptr][i].status != UP_STATUS)
			{
				if(g_u8SingleModeStatus == SWITCH_KEY || g_u8SingleModeStatus == RIGHT_KEY || g_u8SingleModeStatus == NO_KEY)
				{
					g_u8UsbSendBuffer[j++] = 0;
				}
				else
				{
					g_u8UsbSendBuffer[j++] = 1;
				}
			}
			else
			{
				g_u8UsbSendBuffer[j++] = 0;
			}

			touchCnt ++;

			break;
		}
	}

	g_u8UsbSendBuffer[j++] = 0;

#if 0
		DEBF("\r\n");
		DEBF("\r\n ##########SingleMode-SendString: ");
		DEBF("\r\n");
		for(i = 0; i < SINGLE_MODE_REPORT_COUNT; i ++)
		{
			DEBF(" 0x%02X ",g_u8UsbSendBuffer[i]);
		}
		DEBF("\r\n");
#endif

	if(touchCnt > 0)
	{
		if(USB_SendIsValid() == TRUE)
		{
			USB_SIL_Write(EP1_IN, (uint8_t *)g_u8UsbSendBuffer, SINGLE_MODE_REPORT_COUNT);  
			SetEPTxValid(ENDP1);
		}
	}	
}

void USB_MultiModeSendTouchEndStep2(void)
{
	uint8_t 	i,j,touchCnt,totalCnt;
	uint16_t    dpitmp;
	
	j = 0;
	touchCnt = 0;
	
	g_u8UsbSendBuffer[j++] = MULTI_MODE_REPORT_ID;
	
	for(i = 0; i < POINT_BUFFER_SIZE; i++)
	{
		if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == END_STEP2_STATUS)// now is press
		{
			g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status = END_STEP3_STATUS;

			g_u8UsbSendBuffer[j++] = 2;
			g_u8UsbSendBuffer[j++] = i;

			dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].x_position;
			g_u8UsbSendBuffer[j++] = dpitmp& 0xff;
			g_u8UsbSendBuffer[j++] = dpitmp >> 8;

			dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].y_position;
			g_u8UsbSendBuffer[j++] = dpitmp& 0xff;
			g_u8UsbSendBuffer[j++] = dpitmp >> 8;
						
			touchCnt ++;
		}
		else if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == END_STEP3_STATUS)// last is press, now is invalid
		{
			g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status = INVALID_STATUS;
		}
	}

	if(touchCnt > 0)
	{
		totalCnt = touchCnt;
		
		for(i = 0; i < POINT_BUFFER_SIZE; i++)
		{
			if(totalCnt >= USB_SEND_POINT_CNT)
				break;
			
			if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == INVALID_STATUS)
			{
				 g_u8UsbSendBuffer[j++] = 0;
				 g_u8UsbSendBuffer[j++] = i;//touch id;	
				 g_u8UsbSendBuffer[j++] = 0;
				 g_u8UsbSendBuffer[j++] = 0;
				 g_u8UsbSendBuffer[j++] = 0;
				 g_u8UsbSendBuffer[j++] = 0;

				 totalCnt ++;
			}
		}
		
		g_u8UsbSendBuffer[j] = touchCnt;
		
		#if 1
		DEBF("\r\n ##########EndCmd-step2 SendBuffer: ");
		DEBF("\r\n");
		for(i = 1; i < MULTI_MODE_REPORT_COUNT; i ++)
		{
			DEBF(" 0x%02X ",g_u8UsbSendBuffer[i]);
			if(i%6 == 0)
				DEBF("\r\n");
		}
		#endif

		if(USB_SendIsValid() == TRUE)
		{
			USB_SIL_Write(EP1_IN, (uint8_t *)g_u8UsbSendBuffer, MULTI_MODE_REPORT_COUNT);  
			SetEPTxValid(ENDP1);
		}
	}
}

void USB_MultiModeSendTouchEndStep1(void)
{
	uint8_t 	i,j,touchCnt,totalCnt;
	uint16_t    dpitmp;
	
	j = 0;
	touchCnt = 0;
	
	g_u8UsbSendBuffer[j++] = MULTI_MODE_REPORT_ID;
	
	for(i = 0; i < POINT_BUFFER_SIZE; i++)
	{
		if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == END_STEP1_STATUS)// now is press
		{
			g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status = END_STEP2_STATUS;

			g_u8UsbSendBuffer[j++] = 3;
			g_u8UsbSendBuffer[j++] = i;

			dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].x_position;
			g_u8UsbSendBuffer[j++] = dpitmp & 0xff;
			g_u8UsbSendBuffer[j++] = dpitmp >> 8;

			dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].y_position;
			g_u8UsbSendBuffer[j++] = dpitmp & 0xff;
			g_u8UsbSendBuffer[j++] = dpitmp >> 8;

			touchCnt ++;
		}
		else if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == END_STEP2_STATUS)// last is press, now is invalid
		{
			g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status = END_STEP3_STATUS;
			
			g_u8UsbSendBuffer[j++] = 2;
			g_u8UsbSendBuffer[j++] = i;

			dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].x_position;
			g_u8UsbSendBuffer[j++] = dpitmp & 0xff;
			g_u8UsbSendBuffer[j++] = dpitmp >> 8;

			dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].y_position;
			g_u8UsbSendBuffer[j++] = dpitmp & 0xff;
			g_u8UsbSendBuffer[j++] = dpitmp >> 8;

			touchCnt ++;
		}
		else if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == END_STEP3_STATUS)
		{
			g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status = INVALID_STATUS;
		}
	}

	if(touchCnt > 0)
	{
		totalCnt = touchCnt;
		
		for(i = 0; i < POINT_BUFFER_SIZE; i++)
		{
			if(totalCnt >= USB_SEND_POINT_CNT)
				break;
			
			if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == INVALID_STATUS)
			{
				 g_u8UsbSendBuffer[j++] = 0;
				 g_u8UsbSendBuffer[j++] = i;//touch id;
				 g_u8UsbSendBuffer[j++] = 0;
				 g_u8UsbSendBuffer[j++] = 0;
				 g_u8UsbSendBuffer[j++] = 0;
				 g_u8UsbSendBuffer[j++] = 0;

				 totalCnt ++;
			}
		}
		
		g_u8UsbSendBuffer[j] = touchCnt;
		
		#if 1
		DEBF("\r\n ##########EndCmd-step1 SendBuffer: ");
		DEBF("\r\n");
		for(i = 1; i < MULTI_MODE_REPORT_COUNT; i ++)
		{
			DEBF(" 0x%02X ",g_u8UsbSendBuffer[i]);
			if(i%6 == 0)
				DEBF("\r\n");
		}
		#endif
		
		if(USB_SendIsValid() == TRUE)
		{
			USB_SIL_Write(EP1_IN, (uint8_t *)g_u8UsbSendBuffer, MULTI_MODE_REPORT_COUNT);  
			SetEPTxValid(ENDP1);
		}
	}
}

void USB_MultiModeSendTouchEndStep0(void)
{
	uint8_t 	i,j,touchCnt,totalCnt;
	uint16_t    dpitmp;
	
	j = 0;
	touchCnt = 0;

#if 0
	DEBF("\r\n ##########SendTouchEndCmd, Last-SendBuffer: ");
	DEBF("\r\n");
	for(i = 1; i < MULTI_MODE_REPORT_COUNT; i ++)
	{
		DEBF(" 0x%02X ",g_u8UsbSendBuffer[i]);
		if(i%6 == 0)
			DEBF("\r\n");
	}
#endif

	g_u8UsbSendBuffer[j++] = MULTI_MODE_REPORT_ID;

	//DEBF("\r\n");
	
	for(i = 0; i < POINT_BUFFER_SIZE; i++)
	{
		if(g_bTouchMode[i] == WRITE_MODE)// cache 1 point
		{
			if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status != INVALID_STATUS)
			{
				if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status != PRESS_STATUS) // running, halt, up
				{
					//DEBF("\r\n EndCmd, touch:%d, write mode, in case 0 \n", i);
					g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status = END_STEP3_STATUS; // 2 -> 0
				}
				else// press
				{
					//DEBF("\r\n EndCmd, touch:%d, write mode, in case 1 \n", i);
					g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status = END_STEP1_STATUS; // 2 -> 3 -> 2
				}
				
				g_u8UsbSendBuffer[j++] = 2;
				g_u8UsbSendBuffer[j++] = i;//touch id

				dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].x_position;
				g_u8UsbSendBuffer[j++] = dpitmp & 0xff;
				g_u8UsbSendBuffer[j++] = dpitmp >> 8;

				dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].y_position;
				g_u8UsbSendBuffer[j++] = dpitmp & 0xff;
				g_u8UsbSendBuffer[j++] = dpitmp >> 8;
				
				touchCnt ++;
			}
		}
		else if(g_bTouchMode[i] == CLICK_MODE || g_bTouchMode[i] == SWITCH_MODE)// no cache
		{
			if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status != INVALID_STATUS && g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status != UP_STATUS)
			{
				if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == RUNNING_STATUS || g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == HALT_STATUS) // running, halt
				{
					//DEBF("\r\n EndCmd, touch:%d, mode:%d, in case 0 \n", i, g_bTouchMode[i]);
					g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status = END_STEP3_STATUS;// 2 -> 0
					g_u8UsbSendBuffer[j++] = 2;
				}
				else if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == PRESS_STATUS)// now press
				{
					//DEBF("\r\n EndCmd, touch:%d, mode:%d, in case 1 \n", i, g_bTouchMode[i]);
					g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status = END_STEP2_STATUS; // 3 -> 2
					g_u8UsbSendBuffer[j++] = 3;
				}
				else
				{
					//ERRF("\r\n EndCmd, touch:%d, mode:%d, status:%d, error!! \n", i, g_bTouchMode[i], g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status);
				}

				g_u8UsbSendBuffer[j++] = i;//touch id

				dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].x_position;
				g_u8UsbSendBuffer[j++] = dpitmp & 0xff;
				g_u8UsbSendBuffer[j++] = dpitmp >> 8;

				dpitmp = g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].y_position;
				g_u8UsbSendBuffer[j++] = dpitmp & 0xff;
				g_u8UsbSendBuffer[j++] = dpitmp >> 8;
				
				touchCnt ++;
			}
			else if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == UP_STATUS)//  up
			{
				//DEBF("\r\n EndCmd, touch:%d, mode:%d, in case 2 \n", i, g_bTouchMode[i]);
				g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status = INVALID_STATUS; // 0
			}
		}
	}

	totalCnt = touchCnt;

	for(i = 0; i < POINT_BUFFER_SIZE; i++)
	{
		if(totalCnt >= USB_SEND_POINT_CNT)
			break;
		
		if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == INVALID_STATUS)
		{
			 g_u8UsbSendBuffer[j++] = 0;
			 g_u8UsbSendBuffer[j++] = i;//touch id;	
			 g_u8UsbSendBuffer[j++] = 0;
			 g_u8UsbSendBuffer[j++] = 0;
			 g_u8UsbSendBuffer[j++] = 0;
			 g_u8UsbSendBuffer[j++] = 0;

			 totalCnt ++;
		}
	}
	
	g_u8UsbSendBuffer[j] = touchCnt;

	#if 1
	DEBF("\r\n");
	DEBF("\r\n ##########EndCmd-step0 SendBuffer: ");
	DEBF("\r\n");
	for(i = 1; i < MULTI_MODE_REPORT_COUNT; i ++)
	{
		DEBF(" 0x%02X ",g_u8UsbSendBuffer[i]);
		if(i%6 == 0)
			DEBF("\r\n");
	}
	#endif
	
	if(USB_SendIsValid() == TRUE)
	{
		USB_SIL_Write(EP1_IN, (uint8_t *)g_u8UsbSendBuffer, MULTI_MODE_REPORT_COUNT);  
		SetEPTxValid(ENDP1);
	}
}

void USB_MultiModeSendString(void)
{
	uint8_t 	i,j,touchCnt,totalCnt,ptr;
	uint16_t    dpitmp;
	
	j = 0;
	touchCnt = 0;
	totalCnt = 0;

	g_u8UsbSendBuffer[j++] = MULTI_MODE_REPORT_ID;
	
	for(i = 0; i < POINT_BUFFER_SIZE; i++)
	{
		if(g_bTouchMode[i] == INVALID_MODE)
			continue;
		
		//DEBF("\r\n Multi touch USB Send P:%d, mode:%d \n",i, g_bTouchMode[i]);
		
		if(g_bTouchMode[i] == CLICK_MODE)
			ptr = g_bTouchBufferCurPtr;
		else
			ptr = g_bTouchBufferRDPtr;
		
		if(g_stTouchDataBuffer[ptr][i].status != INVALID_STATUS)
		{
			if((g_stTouchDataBuffer[ptr][i].status == RUNNING_STATUS) || (g_stTouchDataBuffer[ptr][i].status == HALT_STATUS))
			{
				g_u8UsbSendBuffer[j++] = 3;
			}
			else
			{
				g_u8UsbSendBuffer[j++] = 2;
			}

			g_u8UsbSendBuffer[j++] = i;//touch id

			dpitmp = g_stTouchDataBuffer[ptr][i].x_position;
			g_u8UsbSendBuffer[j++] = dpitmp& 0xff;
			g_u8UsbSendBuffer[j++] = dpitmp >> 8;

			dpitmp = g_stTouchDataBuffer[ptr][i].y_position;
			g_u8UsbSendBuffer[j++] = dpitmp& 0xff;
			g_u8UsbSendBuffer[j++] = dpitmp >> 8;

			touchCnt++;
		}
	}

	totalCnt = touchCnt;

	for(i = 0; i < POINT_BUFFER_SIZE; i++)
	{
		if(totalCnt >= USB_SEND_POINT_CNT)
			break;
		
		if(g_bTouchMode[i] == WRITE_MODE)
			ptr = g_bTouchBufferRDPtr;
		else
			ptr = g_bTouchBufferCurPtr;
		
		if(g_stTouchDataBuffer[ptr][i].status == INVALID_STATUS)
		{
			 g_u8UsbSendBuffer[j++] = 0;
			 g_u8UsbSendBuffer[j++] = i;//touch id;
			 g_u8UsbSendBuffer[j++] = 0;
			 g_u8UsbSendBuffer[j++] = 0;
			 g_u8UsbSendBuffer[j++] = 0;
			 g_u8UsbSendBuffer[j++] = 0;

			 totalCnt ++;
		}
	}
	
	g_u8UsbSendBuffer[j] = touchCnt;
	
#if 0
	DEBF("\r\n");
	DEBF("\r\n ##########SendString: touch:%d",touchCnt);
	DEBF("\r\n");
	for(i = 1; i < MULTI_MODE_REPORT_COUNT; i ++)
	{
		DEBF(" 0x%02X ",g_u8UsbSendBuffer[i]);
		if(i%6 == 0)
			DEBF("\r\n");
	}
#endif

	if(touchCnt > 0)
	{
		if(USB_SendIsValid() == TRUE)
		{
			USB_SIL_Write(EP1_IN, (uint8_t *)g_u8UsbSendBuffer, MULTI_MODE_REPORT_COUNT);  
			SetEPTxValid(ENDP1);
		}
	}	
}

void SendTouchEndToTerminal(void)
{
	if(g_u8TouchMode != MULTI_MODE)
	{
		USB_SingleModeSendTouchEndCmd();
		//if(g_bUartSendOpen == TRUE)
		//	UART_SendTouchData();
	}
	else
	{
		USB_MultiModeSendTouchEndStep0();
		//if(g_bUartSendOpen == TRUE)
		//	UART_SendTouchData();

		USB_MultiModeSendTouchEndStep1();
		//if(g_bUartSendOpen == TRUE)
		//	UART_SendTouchData();
		
		USB_MultiModeSendTouchEndStep2();
		//if(g_bUartSendOpen == TRUE)
		//	UART_SendTouchData();
	}
}

//put Cur touch data into array,and send string via usb.
void FillBufferAndSendData(void)
{
	uint8_t i;
	uint16_t	tmpX,tmpY;

	//put touch data into buffer 
	for(i = 0; i < POINT_BUFFER_SIZE; i ++)
	{
		if(g_stLastTouchPoint[i].status.u8Value != INVALID_STATUS )//&& g_stLastTouchPoint[i].status.u8Value != UP_STATUS)
		{
			tmpX = (g_stLastTouchPoint[i].x_position * X_DPI);
			tmpY = (g_stLastTouchPoint[i].y_position * Y_DPI);
			
		
 			//DEBF("\r\n  FillBuffer, p:%d, (%d, %d), (0x%x, 0x%x) \n", i, g_stLastTouchPoint[i].x_position, g_stLastTouchPoint[i].y_position, tmpX, tmpY);
			
			if((g_stLastTouchPoint[i].status.u8Value != UP_STATUS) && (g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].x_position == tmpX && g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].y_position == tmpY))
			{
				g_stTouchDataBuffer[g_bTouchBufferWRPtr][i].status = HALT_STATUS;

				if(g_stTouchDataBuffer[g_bTouchBufferCurPtr][i].status == HALT_STATUS)
				{
					if(g_u8HaltStatusCnt[i] < 255)
						g_u8HaltStatusCnt[i] ++;
				}
				else
				{
					g_u8HaltStatusCnt[i] = 0;
				}
			}
			else
			{
				g_stTouchDataBuffer[g_bTouchBufferWRPtr][i].status = g_stLastTouchPoint[i].status.u8Value;
			}
			
			g_stTouchDataBuffer[g_bTouchBufferWRPtr][i].x_position = tmpX;
			g_stTouchDataBuffer[g_bTouchBufferWRPtr][i].y_position = tmpY;

			if(g_bTouchBufferPtrCnt[i] < MAX_CACHE_BUFFCNT)
			{
				g_bTouchBufferPtrCnt[i] ++;
			}

			switch(g_bTouchBufferPtrCnt[i])
			{
				case MAX_CACHE_BUFFCNT - 1:
					g_bTouchMode[i] = SWITCH_MODE;
					break;
				case MAX_CACHE_BUFFCNT:
					g_bTouchMode[i] = WRITE_MODE;
					break;
				default:
					g_bTouchMode[i] = CLICK_MODE;
					break;
			}
		}
		else
		{
			g_stTouchDataBuffer[g_bTouchBufferWRPtr][i].status = INVALID_STATUS;
			g_stTouchDataBuffer[g_bTouchBufferWRPtr][i].x_position = 0;
			g_stTouchDataBuffer[g_bTouchBufferWRPtr][i].y_position = 0;
			//haleliu@20131114 modify for jump wire
			g_bTouchBufferPtrCnt[i] = 0;  
			g_bTouchMode[i] = INVALID_MODE;
		}
	}

	g_bTouchBufferCurPtr = g_bTouchBufferWRPtr;
	g_bTouchBufferRDPtr = g_bTouchBufferCurPtr > 0 ? g_bTouchBufferCurPtr - 1 : MAX_CACHE_BUFFCNT - 1;

	g_bTouchBufferWRPtr ++;
	if(g_bTouchBufferWRPtr >= MAX_CACHE_BUFFCNT)
	{
		g_bTouchBufferWRPtr = 0;
	}

}


/**
  * @brief  AD Samp One Point (Any X & Y Point)
	*     @Note The Freq is 74.35KHz, 13.5us 
  * @param  Direction: DIRECTION_TypeDef, X or Y direction.
  *   This parameter can be one of the DIRECTION_TypeDef enum values:
  *     @arg X_DIRECTION: X direction
  *     @arg Y_DIRECTION: Y direction
  * @param  T_Point: Trans Point number.
  *   The number is in 0 - (X_POINT_SIZE-1)
  * @param  R_Point: Receive Point number.
  *   The number is in 0 - (Y_POINT_SIZE-1)
  * @param  OhmVal: Ohm value.
  * @retval The AD Sample Value
  */
uint16_t Init_Sample_1_Point(DIRECTION_TypeDef Direction, uint16_t T_Point, uint16_t R_Point, uint16_t OhmVal)
{
	uint16_t AdcBefore,AdcAffter,AdcSum;
	
	Init_Receive_Point_Set(Direction, R_Point);
	Init_Trans_Point_Set(Direction, T_Point);
	Resistor_Set_12bit(OhmVal);
	
	DelayUS(30);
	
	AdcSum = 0;
	AdcBefore = 0;
	AdcAffter = 0;
	AdcBefore = ADC_GetConversionValue(ADC1);
	Trans_TurnOn();

	//DelayUS(8);
	DelayUS(5);
	
	AdcAffter = ADC_GetConversionValue(ADC1);
	Trans_TurnOff();

	if(AdcAffter >= AdcBefore)
	   	AdcSum =  AdcAffter - AdcBefore;
	else
		AdcSum = 0;
	
	//debug_before = AdcBefore;
	//debug_after = AdcAffter;
	
	return (AdcSum);
} 
uint16_t Sample_1_Point(DIRECTION_TypeDef Direction, uint16_t T_Point, uint16_t R_Point, uint16_t OhmVal)
{
	uint16_t AdcBefore,AdcAffter,AdcSum;
	
	Receive_Point_Set(Direction, R_Point);
	Trans_Point_Set(Direction, T_Point);
	Resistor_Set_12bit(OhmVal);
	
	//DelayUS(30);
	DelayUS(20);
	
	AdcSum = 0;
	AdcBefore = 0;
	AdcAffter = 0;
	AdcBefore = ADC_GetConversionValue(ADC1);
	Trans_TurnOn();

	//DelayUS(8);
	DelayUS(5);
	
	AdcAffter = ADC_GetConversionValue(ADC1);
	Trans_TurnOff();

	if(AdcAffter >= AdcBefore)
	   	AdcSum =  AdcAffter - AdcBefore;
	else
		AdcSum = 0;
	
	//debug_before = AdcBefore;
	//debug_after = AdcAffter;
	
	return (AdcSum);
}

void QuickSamplePointSetParam(LeanScanPointPara_t p)
{
	//TIM_SetCounter(TIM3, 0);
	//TIM_Cmd(TIM3, ENABLE);

	Receive_Point_Set((DIRECTION_TypeDef)p.direc, p.revP);
	Trans_Point_Set((DIRECTION_TypeDef)p.direc, p.sendP);
	Resistor_Set_12bit(p.eCur);
	
	//DEBF("\r\n  cnt:%d \n",TIM_GetCounter(TIM3));
	//TIM_Cmd(TIM3, DISABLE);

	StartTimer3(30);
	
	g_u8LastDirec = p.direc;
	g_u16LastRevPoint = p.revP;
	g_u16LastSendPoint = p.sendP;
		
	g_u8QuickSampleFlag ++;
	//DEBF("\r\n  %s, (QuickSampleFlag:%d) \n",__FUNCTION__,g_u8QuickSampleFlag);
}

uint16_t QuickSamplePointGetAd(void)
{
	uint16_t AdcBefore,AdcAffter,AdcSum;
	
	AdcBefore = ADC_GetConversionValue(ADC1);	
	Trans_TurnOn();
	
	//DelayUS(8);
	DelayUS(5);
	
	AdcAffter = ADC_GetConversionValue(ADC1);
	Trans_TurnOff();

	if(AdcAffter >= AdcBefore)
	   	AdcSum =  AdcAffter - AdcBefore;
	else
		AdcSum = 0;

	//debug_before = AdcBefore;
	//debug_after = AdcAffter;

	if(g_u8QuickSampleFlag > 0)
		g_u8QuickSampleFlag --;	
	//DEBF("\r\n  %s, (QuickSampleFlag:%d) \n",__FUNCTION__,g_u8QuickSampleFlag);
	
	return (AdcSum);
}

void Init_StaticPointEcurAndResData(void)
{
	int i;
	uint16_t eCur_value,ad_value;

	//DEBF("\r\n  %s:  \n",__FUNCTION__);
	
	for(i = 0; i < X_POINT_SIZE; i ++)
	{
		g_u16XV_ResBuff[i] = 0;		
		g_u16XV_EcurBuff[i] = Ecurrent_None;
		
		//g_u16XR_Offset_ResBuff[i] = 0;
		g_u16XR_Offset_EcurBuff[i] = Ecurrent_None;
		
		//g_u16XL_Offset_ResBuff[i] = 0;
		g_u16XL_Offset_EcurBuff[i] = Ecurrent_None;
	}
	
	for(i = 0; i < Y_POINT_SIZE; i ++)
	{
		g_u16YV_ResBuff[i] = 0;		
		g_u16YV_EcurBuff[i] = Ecurrent_None;
		
		//g_u16YR_Offset_ResBuff[i] = 0;
		g_u16YR_Offset_EcurBuff[i] = Ecurrent_None;

		//g_u16YL_Offset_ResBuff[i] = 0;
		g_u16YL_Offset_EcurBuff[i] = Ecurrent_None;
	}
	
	printf("\r\n	INIT YV >>>>>>>> \n");
	for(i = Y_POINT_SIZE - 1; i >= 0; i --)
	{
		eCur_value = YV_INIT_ECUR_VALUE;
		
		do
		{
			DelayUS(100);
			ad_value = Init_Sample_1_Point(Y_DIRECTION, i, i, eCur_value); 
			
			if(ad_value >= AD_Y_RES_COM_VALUE)
				break;
			
			if(ad_value < AD_Y_NEAR_RES_COM_VALUE)
				eCur_value += 5;
			else
				eCur_value += 2;
			
		}while(eCur_value < ECUR_COM_MAX_VALUE);

		if(eCur_value >= ECUR_COM_MAX_VALUE)
			eCur_value -= 2;
		
		g_u16YV_EcurBuff[i] = eCur_value;
		g_u16YV_ResBuff[i] = ad_value >> 2;//0.25
		g_u16YV_ResBuff[i] &= AD_DIFF_CHECK_MASK;
		
		if(ad_value <= AD_BAD_POINT_RES_VALUE)
		{
			g_bYGetBadPoint = TRUE;
			g_u16YV_EcurBuff[i] |= BAD_POINT_FLAG;
			g_u16YV_ResBuff[i] |= BAD_POINT_FLAG;
		}

		INIT_DEBF("\r\n YV: P%d, Ecur:%d,Res:%d \n", i, eCur_value,ad_value);
	}
	
	DelayMS(10);

	printf("\r\n	INIT XV >>>>>>>> \n");
	for(i = 0; i < X_POINT_SIZE; i ++)
	{	
		eCur_value = XV_INIT_ECUR_VALUE;

		do
		{
			DelayUS(100);
			ad_value = Init_Sample_1_Point(X_DIRECTION, i, i, eCur_value);

			if(ad_value >= AD_RES_COM_VALUE)
				break;
			
			//INIT_DEBF("\r\n XV: P%d, Ecur:%d,Res:%d \n", i, eCur_value, ad_value);

			if(ad_value < AD_NEAR_RES_COM_VALUE)
				eCur_value += 5;
			else
				eCur_value += 2;
			
		}while(eCur_value < ECUR_COM_MAX_VALUE);

		if(eCur_value >= ECUR_COM_MAX_VALUE)
			eCur_value -= 2;
		
		g_u16XV_EcurBuff[i] = eCur_value;
		g_u16XV_ResBuff[i] = ad_value >> 2;//0.25
		g_u16XV_ResBuff[i] &= AD_DIFF_CHECK_MASK;

		if(ad_value <= AD_BAD_POINT_RES_VALUE)
		{
			g_bXGetBadPoint = TRUE;
			g_u16XV_EcurBuff[i] |= BAD_POINT_FLAG;
			g_u16XV_ResBuff[i] |= BAD_POINT_FLAG;
		}
		
		INIT_DEBF("\r\n XV: P%d, Ecur:%d,Res:%d \n", i, eCur_value, ad_value);
	}

	DelayMS(10);
	
	printf("\r\n	INIT YR offset >>>>>>>> \n");
	for(i = Y_POINT_SIZE - 1; i >= 0; i --)
	{
		g_u16YR_Offset_EcurBuff[i] = g_u16YV_EcurBuff[i];
		//g_u16YR_Offset_ResBuff[i] = g_u16YV_ResBuff[i] + g_u16YV_ResBuff[i] << 1 + g_u16YV_ResBuff[i] >> 1;
		//g_u16YR_Offset_ResBuff[i] &= AD_DIFF_CHECK_MASK;

		if(i + LED_OFFSET_NMB_Y >= Y_POINT_SIZE)
			continue;

		eCur_value = YL_INIT_ECUR_VALUE;
		
		do
		{
			DelayUS(100);
			ad_value = Init_Sample_1_Point(Y_DIRECTION, i, i + LED_OFFSET_NMB_Y, eCur_value);	

			if(ad_value >= AD_RES_COM_VALUE)
				break;
			
			if(ad_value < AD_NEAR_RES_COM_VALUE)
				eCur_value += 5;
			else
				eCur_value += 2;
			
			
		}while(eCur_value < ECUR_COM_MAX_VALUE);
		
		if(eCur_value >= ECUR_COM_MAX_VALUE)
			eCur_value -= 2;
		
		g_u16YR_Offset_EcurBuff[i] = eCur_value;
		
		//g_u16YR_Offset_ResBuff[i + LED_OFFSET_NMB_Y] = (ad_value >> 1) + (ad_value >> 2) + (ad_value >> 3);
		//g_u16YR_Offset_ResBuff[i + LED_OFFSET_NMB_Y] &= AD_DIFF_CHECK_MASK;
		
		//INIT_DEBF("\r\n YR: S(P%d - Ecur:%d),R(P%d - Res:%d) \n", i, eCur_value, i + LED_OFFSET_NMB_Y, ad_value);
	}

	DelayMS(10);
		
	printf("\r\n	INIT XR offset >>>>>>>> \n");
	for(i = 0; i < X_POINT_SIZE; i ++)
	{
		g_u16XR_Offset_EcurBuff[i] = g_u16XV_EcurBuff[i];
		//g_u16XR_Offset_ResBuff[i] = g_u16XV_ResBuff[i] + g_u16XV_ResBuff[i] << 1 + g_u16XV_ResBuff[i] >> 1;
		//g_u16XR_Offset_ResBuff[i] &= AD_DIFF_CHECK_MASK;

		if(i - LED_OFFSET_NMB_X < 0)
			continue;
		
		eCur_value = XL_INIT_ECUR_VALUE;
		
		do
		{	
			DelayUS(100);
			ad_value = Init_Sample_1_Point(X_DIRECTION, i, i - LED_OFFSET_NMB_X, eCur_value);

			if(ad_value >= AD_RES_COM_VALUE)
				break;
			
			
			if(ad_value < AD_NEAR_RES_COM_VALUE)
				eCur_value += 5;
			else
				eCur_value += 2;
			
		}while(eCur_value < ECUR_COM_MAX_VALUE);

		if(eCur_value >= ECUR_COM_MAX_VALUE)
			eCur_value -= 2;

		g_u16XR_Offset_EcurBuff[i] = eCur_value;
		
		//0.5 + 0.25 + 0.125 = 0.875
		//g_u16XR_Offset_ResBuff[i - LED_OFFSET_NMB_X] = (ad_value >> 1) + (ad_value >> 2) + (ad_value >> 3);
		//g_u16XR_Offset_ResBuff[i - LED_OFFSET_NMB_X] &= AD_DIFF_CHECK_MASK;

		//INIT_DEBF("\r\n XR: S(P%d - Ecur:%d),R(P%d - Res:%d) \n", i, eCur_value, i - LED_OFFSET_NMB_X, ad_value); 	
	}

	DelayMS(10);

	printf("\r\n	INIT YL offset >>>>>>>> \n");
	for(i = Y_POINT_SIZE - 1; i >= 0; i --)
	{
		g_u16YL_Offset_EcurBuff[i] = g_u16YV_EcurBuff[i];
		//g_u16YL_Offset_ResBuff[i] = g_u16YV_ResBuff[i] + g_u16YV_ResBuff[i] << 1 + g_u16YV_ResBuff[i] >> 1;
		//g_u16YL_Offset_ResBuff[i] &= AD_DIFF_CHECK_MASK;
		
		if(i - LED_OFFSET_NMB_Y < 0)
			continue;

		eCur_value = YL_INIT_ECUR_VALUE;
		
		do
		{
			DelayUS(100);
			ad_value = Init_Sample_1_Point(Y_DIRECTION, i, i - LED_OFFSET_NMB_Y, eCur_value);	

			if(ad_value >= AD_RES_COM_VALUE)
				break;
					
			if(ad_value < AD_NEAR_RES_COM_VALUE)
				eCur_value += 5;
			else
				eCur_value += 2;
			
		}while(eCur_value < ECUR_COM_MAX_VALUE);

		if(eCur_value >= ECUR_COM_MAX_VALUE)
			eCur_value -= 2;
		
		g_u16YL_Offset_EcurBuff[i] = eCur_value;
		//g_u16YL_Offset_ResBuff[i - LED_OFFSET_NMB_Y] = (ad_value >> 1) + (ad_value >> 2) + (ad_value >> 3);
		//g_u16YL_Offset_ResBuff[i - LED_OFFSET_NMB_Y] &= AD_DIFF_CHECK_MASK;
		
		//INIT_DEBF("\r\n YL: S(P%d - Ecur:%d),R(P%d - Res:%d) \n", i, eCur_value, i - LED_OFFSET_NMB_Y, ad_value);
	}

	DelayMS(10);
	
	printf("\r\n	INIT XL offset >>>>>>>> \n");
	for(i = 0; i < X_POINT_SIZE; i ++)
	{		
		g_u16XL_Offset_EcurBuff[i] = g_u16XV_EcurBuff[i];
		//g_u16XL_Offset_ResBuff[i] = g_u16XV_ResBuff[i] + g_u16XV_ResBuff[i] << 1 + g_u16XV_ResBuff[i] >> 1;
		//g_u16XL_Offset_ResBuff[i] &= AD_DIFF_CHECK_MASK;

		if(i + LED_OFFSET_NMB_X >= X_POINT_SIZE)
			continue;

		eCur_value = XL_INIT_ECUR_VALUE;
		
		do
		{	
			DelayUS(100);
			ad_value = Init_Sample_1_Point(X_DIRECTION, i, i + LED_OFFSET_NMB_X, eCur_value);

			if(ad_value >= AD_RES_COM_VALUE)
				break;
			
			if(ad_value < AD_NEAR_RES_COM_VALUE)
				eCur_value += 5;
			else
				eCur_value += 2;
			
		}while(eCur_value < ECUR_COM_MAX_VALUE);
		
		if(eCur_value >= ECUR_COM_MAX_VALUE)
			eCur_value -= 2;
		
		g_u16XL_Offset_EcurBuff[i] = eCur_value;
		//0.5 + 0.25 + 0.125 = 0.875
		//g_u16XL_Offset_ResBuff[i + LED_OFFSET_NMB_X] = (ad_value >> 1) + (ad_value >> 2) + (ad_value >> 3);
		//g_u16XL_Offset_ResBuff[i + LED_OFFSET_NMB_X] &= AD_DIFF_CHECK_MASK;
		
		//INIT_DEBF("\r\n XL: S(P%d - Ecur:%d),R(P%d - Res:%d) \n", i, eCur_value, i + LED_OFFSET_NMB_X, ad_value);
	}
}

void Init_BadPointCheck(void)
{
	s16 i,j,k,m,n;
	uint16_t eCur_value,ad_value;

	if(g_bXGetBadPoint == FALSE && g_bYGetBadPoint == FALSE)
		return;

	if(g_bInitCheck == TRUE)
	printf("\r\n  init check bad point!! \n");

	g_u16XBadPointCnt = 0;
	g_u16YBadPointCnt = 0;

	eCur_value = 0;
	ad_value = 0;

	if(g_bXGetBadPoint == TRUE)
	{
		for(i = 0; i < X_POINT_SIZE; i ++)
		{
			if((g_u16XV_EcurBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG || (g_u16XV_ResBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG ||
				(g_u16XV_EcurBuff[i]&VBAD_POINT_FLAG) == VBAD_POINT_FLAG || (g_u16XV_ResBuff[i]&VBAD_POINT_FLAG) == VBAD_POINT_FLAG)
			{
				m = 0;
				n = 1;
				for(j = 0; j < 16; j ++)
				{
					if((j%2) == 0)
					{
						k = i + m;
						m ++;
					}
					else
					{
						k = i - n;
						n ++;
					}
					
					if( k < 0 || k >= X_POINT_SIZE)
						continue;
					
					eCur_value = XV_INIT_ECUR_VALUE;
					do
					{	
						DelayUS(100);
						ad_value = Init_Sample_1_Point(X_DIRECTION, i, k, eCur_value);
				
						if(ad_value >= AD_RES_BAD_CHK_VALUE)
							break;
				
						if(ad_value < AD_NEAR_RES_BAD_CHK_VALUE)
							eCur_value += 5;
						else
							eCur_value += 2;
						
					}while(eCur_value < ECUR_COM_MAX_VALUE);
					
					if(ad_value >= AD_RES_BAD_CHK_VALUE)
					{
						g_u16XV_EcurBuff[i] = eCur_value;
						break;
					}		
				}
				if((g_u16XV_EcurBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
				{
					g_u16XBadPointCnt ++;
					if(g_bInitCheck == TRUE)
					printf("\r\n X send point:%d is bad!! ecur:%d, res:%d. \n", i, eCur_value, ad_value);
				}
				else
				{
					if(g_bInitCheck == TRUE)
					printf("\r\n X send point:%d is good!! RevP:%d, set ecur:%d, (res:%d). \n", i, k, eCur_value, ad_value);
					
					if(i != k)
					{
						g_u16XV_EcurBuff[i] |= VBAD_POINT_FLAG;
						
						if(g_bInitCheck == TRUE)
						printf("\r\n eCur:%d, ad:%d. mean: V Line is bad. \n", g_u16XV_EcurBuff[i]&VBAD_POINT_FLAG_UNMASK, ad_value);
					}
					else
					{
						g_u16XV_ResBuff[i] = ad_value >> 2;
					}
				}

				m = 0;
				n = 1;
				for(j = 0; j < 16; j ++)
				{
					if((j%2) == 0)
					{
						k = i + m;
						m ++;
					}
					else
					{
						k = i - n;
						n ++;
					}

					if( k < 0 || k >= X_POINT_SIZE)
						continue;
					
					eCur_value = XV_INIT_ECUR_VALUE;
					do
					{	
						DelayUS(100);
						ad_value = Init_Sample_1_Point(X_DIRECTION, k, i, eCur_value);
				
						if(ad_value >= AD_RES_BAD_CHK_VALUE)
							break;
				
						if(ad_value < AD_NEAR_RES_BAD_CHK_VALUE)
							eCur_value += 5;
						else
							eCur_value += 2;
						
					}while(eCur_value < ECUR_COM_MAX_VALUE);
					
					if(ad_value >= AD_RES_BAD_CHK_VALUE)
					{
						g_u16XV_ResBuff[i] = ad_value >> 2;

						if(i != k)
						{
							g_u16XV_ResBuff[i] |= VBAD_POINT_FLAG;
						}
						else
						{
							g_u16XV_EcurBuff[i] = eCur_value;
						}
						break;
					}
				}

				if((g_u16XV_ResBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
				{
					g_u16XBadPointCnt ++;
					if(g_bInitCheck == TRUE)
					printf("\r\n X rev point:%d is bad!! ecur:%d, res:%d. \n", i, eCur_value, ad_value);
				}
				else if((g_u16XV_ResBuff[i]&VBAD_POINT_FLAG) == VBAD_POINT_FLAG)
				{
					g_u16XBadPointCnt ++;
					g_u16YBadPointCnt ++;
					if(g_bInitCheck == TRUE)
					printf("\r\n X point:%d rev and send all is good!! eCur:%d, ad:%d. mean: V Line is bad. \n", i, g_u16XV_EcurBuff[i]&VBAD_POINT_FLAG_UNMASK, ad_value);
				}
				else
				{
					if(g_bInitCheck == TRUE)
					printf("\r\n X rev point:%d is good!! SendP:%d, set res:%d, (ecur:%d).  \n", i, k, ad_value, eCur_value);
				}
			}
		}
	}

	if(g_bYGetBadPoint == TRUE)
	{
		for(i = 0; i < Y_POINT_SIZE; i ++)
		{
			if((g_u16YV_EcurBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG || (g_u16YV_ResBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG ||
				(g_u16YV_EcurBuff[i]&VBAD_POINT_FLAG) == VBAD_POINT_FLAG || (g_u16YV_ResBuff[i]&VBAD_POINT_FLAG) == VBAD_POINT_FLAG)
			{
				m = 0;
				n = 1;
				for(j = 0; j < 16; j ++)
				{
					if((j%2) == 0)
					{
						k = i + m;
						m ++;
					}
					else
					{
						k = i - n;
						n ++;
					}

					if( k < 0 || k >= Y_POINT_SIZE)
						continue;
					
					eCur_value = YV_INIT_ECUR_VALUE;
					do
					{	
						DelayUS(100);
						ad_value = Init_Sample_1_Point(Y_DIRECTION, i, k, eCur_value);
				
						if(ad_value >= AD_RES_BAD_CHK_VALUE)
							break;
				
						if(ad_value < AD_NEAR_RES_BAD_CHK_VALUE)
							eCur_value += 5;
						else
							eCur_value += 2;
						
					}while(eCur_value < ECUR_COM_MAX_VALUE);
					
					if(ad_value >= AD_RES_BAD_CHK_VALUE)
					{
						g_u16YV_EcurBuff[i] = eCur_value;
						break;
					}				
				}
				if((g_u16YV_EcurBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
				{
					g_u16YBadPointCnt ++;
					if(g_bInitCheck == TRUE)
					printf("\r\n Y send point:%d is bad!! ecur:%d, ad:%d. \n", i, eCur_value, ad_value);
				}
				else
				{
					if(g_bInitCheck == TRUE)
					printf("\r\n Y send point:%d is good!! RevP:%d, set ecur:%d, (res:%d). \n", i, k, eCur_value, ad_value);
					if(i != k)
					{
						g_u16YV_EcurBuff[i] |= VBAD_POINT_FLAG;
						
						if(g_bInitCheck == TRUE)
						printf("\r\n eCur:%d, ad:%d. mean: V Line is bad. \n", g_u16YV_EcurBuff[i]&VBAD_POINT_FLAG_UNMASK, ad_value);
					}
					else
					{
						g_u16YV_ResBuff[i] = ad_value >> 2;
					}
				}

				m = 0;
				n = 1;
				for(j = 0; j < 16; j ++)
				{
					if((j%2) == 0)
					{
						k = i + m;
						m ++;
					}
					else
					{
						k = i - n;
						n ++;
					}

					if( k < 0 || k >= Y_POINT_SIZE)
						continue;
					
					eCur_value = YV_INIT_ECUR_VALUE;
					do
					{
						DelayUS(100);
						ad_value = Init_Sample_1_Point(Y_DIRECTION, k, i, eCur_value);
				
						if(ad_value >= AD_RES_BAD_CHK_VALUE)
							break;
				
						if(ad_value < AD_NEAR_RES_BAD_CHK_VALUE)
							eCur_value += 5;
						else
							eCur_value += 2;
						
					}while(eCur_value < ECUR_COM_MAX_VALUE);
					
					if(ad_value >= AD_RES_BAD_CHK_VALUE)
					{
						g_u16YV_ResBuff[i] = ad_value >> 2;
						
						if(i != k)
						{
							g_u16YV_ResBuff[i] |= VBAD_POINT_FLAG;
						}
						else
						{
							g_u16YV_EcurBuff[i] = eCur_value;
						}
						break;
					}				
				}

				if((g_u16YV_ResBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
				{
					g_u16YBadPointCnt ++;
					if(g_bInitCheck == TRUE)
					printf("\r\n Y rev point:%d is bad!! ecur:%d, res:%d. \n", i, eCur_value, ad_value);
				}
				else if((g_u16YV_ResBuff[i]&VBAD_POINT_FLAG) == VBAD_POINT_FLAG)
				{
					g_u16XBadPointCnt ++;
					g_u16YBadPointCnt ++;
					if(g_bInitCheck == TRUE)
					printf("\r\n Y point:%d rev and send all is good!! eCur:%d, ad:%d. mean: V Line is bad. \n", i, g_u16YV_EcurBuff[i]&VBAD_POINT_FLAG_UNMASK, ad_value);
				}
				else
				{
					if(g_bInitCheck == TRUE)
					printf("\r\n Y rev point:%d is good!! SendP:%d, set res:%d, (ecur:%d). \n", i, k, ad_value, eCur_value);
				}
			}
		}
	}

	if(g_u16XBadPointCnt == 0)
	{
		g_bXGetBadPoint = FALSE;
	}

	if(g_u16YBadPointCnt == 0)
	{
		g_bYGetBadPoint = FALSE;
	}

	if(g_bInitCheck == TRUE)
	{
		g_bInitCheck = FALSE;
	}
}
void  Fast_Sample_X_Points(void)
{	
	s16 	i;	
	uint16_t	ad_value;
	ad_value = 0;
	g_u8MaskFlag = 0;
	
	for(i = 0;  i < X_POINT_SIZE; i ++)
	{
		if((g_u16XV_ResBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG ||
					(g_u16XV_EcurBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG ||
					(g_u16XV_EcurBuff[i]&VBAD_POINT_FLAG) == VBAD_POINT_FLAG)
		{
			//ERRF("\r\n XD, P:%d, bad! skip! \n", i);
			continue;
		}

		ad_value =	Sample_1_Point(X_DIRECTION, i, i, g_u16XV_EcurBuff[i]);

		//DEBF("\r\n XD, P%d: Ad:%d, Res:%d \n", i, ad_value, g_u16XV_ResBuff[i]);

		if((ad_value <= g_u16XV_ResBuff[i]) || (ad_value <= DYNAMIC_X_SCAN_MASK_AD_VALUE))// mask
		{
			//DEBF("\r\n XD, P%d: Ad:%d, Res:%d, Ecur:%d \n", i, ad_value, g_u16YV_ResBuff[i],g_u16YV_EcurBuff[i]);
			g_u8MaskFlag ++;
			if(g_u8MaskFlag >= 2)
				break;
		}
		else//no mask		
		{
			g_u8MaskFlag = 0;
		}
				
	}
} 

void  Fast_Sample_Y_Points(void)
{	
	s16 	i;	
	uint16_t	ad_value;
	ad_value = 0;
	g_u8MaskFlag = 0;
	
	for(i = Y_POINT_SIZE - 1;  i >= 0; i --)
	{
		if((g_u16YV_ResBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG ||
			(g_u16YV_EcurBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG ||
			(g_u16YV_EcurBuff[i]&VBAD_POINT_FLAG) == VBAD_POINT_FLAG)
		{
			//ERRF("\r\n YD, P:%d, bad! skip! \n", i);
			continue;
		}

		ad_value =	Sample_1_Point(Y_DIRECTION, i, i, g_u16YV_EcurBuff[i]);

		//DEBF("\r\n YD, P%d: Ad:%d, Res:%d \n", i, ad_value, g_u16YV_ResBuff[i]);

		if((ad_value <= g_u16YV_ResBuff[i]) || (ad_value <= DYNAMIC_Y_SCAN_MASK_AD_VALUE))// mask
		{
			//DEBF("\r\n YD, P%d: Ad:%d, Res:%d, Ecur:%d \n", i, ad_value, g_u16YV_ResBuff[i],g_u16YV_EcurBuff[i]);
			g_u8MaskFlag ++;
			if(g_u8MaskFlag >= 2)
				break;
		}
		else//no mask		
		{
			g_u8MaskFlag = 0;
		}
				
	}
} 


// 5000 us once scan
void  Dynamic_Sample_Y_Points(void)
{	
	s16 	i;	
	uint8_t     pStatusO,pStatusN;
	int 		pointStart,pointEnd;
	int 		areaStart,areaEnd;
	uint16_t	ad_value;
	uint8_t		firstPoint,NoMaskcnt;

	pointStart = -1;
	pointEnd = -1;
	areaStart = -1;
	areaEnd = -1;
	ad_value = 0;
	pStatusO = 0xff;
	pStatusN = 0;
	NoMaskcnt = 0;
	firstPoint = 0;
	
	for(i = Y_POINT_SIZE - 1;  i >= 0; i --)
	{
		if((g_u16YV_ResBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG ||
			(g_u16YV_EcurBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG ||
			(g_u16YV_EcurBuff[i]&VBAD_POINT_FLAG) == VBAD_POINT_FLAG)
		{
			//ERRF("\r\n YD, P:%d, bad! skip! \n", i);
			continue;
		}

		ad_value =	Sample_1_Point(Y_DIRECTION, i, i, g_u16YV_EcurBuff[i]);

		//DEBF("\r\n YD, P%d: Ad:%d, Res:%d \n", i, ad_value, g_u16YV_ResBuff[i]);

		if((ad_value <= g_u16YV_ResBuff[i]) || (ad_value <= DYNAMIC_Y_SCAN_MASK_AD_VALUE))// mask
		{
			//DEBF("\r\n YD, P%d: Ad:%d, Res:%d, Ecur:%d \n", i, ad_value, g_u16YV_ResBuff[i],g_u16YV_EcurBuff[i]);
			
			NoMaskcnt = 0;
			pStatusN = 1;//set mask flag
			
			if(pStatusO == 0 && pointEnd == -1)//get mask start point
			{
				pointEnd = i;
				//DEBF("\r\n YD, pointEnd:%d \n",pointEnd);
			}
			else if(pointEnd >= 0 && i <= 0 && pointStart == -1)
			{
				pointStart = 0;
				//DEBF("\r\n YD, pointStart:%d \n",pointStart);
			}

			if(pStatusO == 0 && areaEnd == -1)//get mask start point
			{
				areaEnd = i;
				//DEBF("\r\n YD, areaEnd:%d \n",areaEnd);
			}
			else if(areaEnd >= 0 && i <= 0 && areaStart == -1)
			{
				areaStart = 0;
				NoMaskcnt = D_NO_MASK_POINT_CNT;
				//DEBF("\r\n YD, areaStart:%d \n",areaStart);
			}
		}
		else//no mask		
		{
			pStatusN = 0;//clear mask flag
			
			if(pStatusO == 1 && pointEnd >= 0)
			{
				pointStart = i+1;
				//DEBF("\r\n YD, pointStart:%d \n",pointStart);
			}

			if(pStatusO == 1 && areaEnd >= 0)
			{
				areaStart = i+1;
				//DEBF("\r\n YD, areaStart:%d \n",areaStart);
			}
			
			if(areaStart >= 0 && firstPoint == 0)
			{
				firstPoint = 1;
			}

			if(firstPoint == 1)
			{
				NoMaskcnt ++;
			}
		}
				
		pStatusO = pStatusN;

		if(pointStart >= 0 && pointEnd >= 0)
		{			
			//DEBF("\r\n YD, P%d: (%d-%d) \n",g_u8YTotalFakePointCnt,pointStart,pointEnd);
			if(g_u8YTotalFakePointCnt < MAX_POINT_CNT)
			{
				Y_TotalFakePosition[g_u8YTotalFakePointCnt].pointStart = pointStart;
				Y_TotalFakePosition[g_u8YTotalFakePointCnt].pointEnd = pointEnd;
				g_u8YTotalFakePointCnt ++;
			}

			pointStart = -1;
			pointEnd = -1;
		}
		
		if((NoMaskcnt >= D_NO_MASK_POINT_CNT) || (i <= 0 && areaEnd >= 0))
		{
			if(areaEnd >= 0 && areaStart == -1)
				areaStart = areaEnd;
			
			//DEBF("\r\n YD, Area(%d): (%d-%d) \n",g_u8YTotalAreaCnt,areaStart,areaEnd);
			Y_TotalArea[g_u8YTotalAreaCnt].start = areaStart;
			Y_TotalArea[g_u8YTotalAreaCnt].end = areaEnd;
			g_u8YTotalAreaCnt ++;

			if(g_u8YTotalAreaCnt >= MAX_POINT_CNT)
				break;
			
			firstPoint = 0;
			NoMaskcnt = 0;
			areaStart = -1;
			areaEnd = -1;
		}
	}
} 

// 8931 us once scan
void  Dynamic_Sample_X_Points(void)
{	 
	uint16_t	 i;  
	uint8_t	 pStatusO,pStatusN;
	int		 pointStart,pointEnd;
	int 	 areaStart,areaEnd;
	uint16_t	 ad_value;
	uint8_t	 firstPoint,NoMaskcnt;

	pointStart = -1;
	pointEnd = -1;
	areaStart = -1;
	areaEnd = -1;
	ad_value = 0;
	pStatusO = 0xff;
	pStatusN = 0;
	NoMaskcnt = 0;
	firstPoint = 0;

	for(i = 0;  i < X_POINT_SIZE; i ++)
	{
		if((g_u16XV_ResBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG ||
			(g_u16XV_EcurBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG ||
			(g_u16XV_EcurBuff[i]&VBAD_POINT_FLAG) == VBAD_POINT_FLAG)
		{
			//ERRF("\r\n XD, P:%d, bad! skip! \n", i);
			continue;
		}

		ad_value =  Sample_1_Point(X_DIRECTION, i, i, g_u16XV_EcurBuff[i]);

		//DEBF("\r\n XD, P%d: Ad:%d, Res:%d \n", i, ad_value, g_u16XV_ResBuff[i]);

		if((ad_value <= g_u16XV_ResBuff[i]) || (ad_value <= DYNAMIC_X_SCAN_MASK_AD_VALUE))// mask
		{
			//DEBF("\r\n XD, P%d: Ad:%d, Res:%d \n", i, ad_value, g_u16XV_ResBuff[i]);

			NoMaskcnt = 0;
			pStatusN = 1;//set mask flag

			if(pStatusO == 0 && pointStart == -1)//get mask start point
			{
				pointStart = i;
			}
			else if(pointStart >= 0 && i >= (X_POINT_SIZE-1) && pointEnd == -1)
			{
				pointEnd = X_POINT_SIZE-1;
			}

			if(pStatusO == 0 && areaStart == -1)//get mask start point
			{
				areaStart = i;
			}
			else if(areaStart >= 0 && i >= (X_POINT_SIZE-1) && areaEnd == -1)
			{
				areaEnd = X_POINT_SIZE-1;
				NoMaskcnt = D_NO_MASK_POINT_CNT;
			}
		}
		else//no mask		 
		{
			pStatusN = 0;//clear mask flag
			
			if(pStatusO == 1 && pointStart >= 0)//get mask end point
			{
				pointEnd = i-1;
			}

			if(pStatusO == 1 && areaStart >= 0)
			{
				areaEnd = i-1;
			}

			if(areaEnd >= 0 && firstPoint == 0)
			{
				firstPoint = 1;
			}

			if(firstPoint == 1)
			{
				NoMaskcnt ++;
			}
		}

		//pre_ad_value = ad_value;
		pStatusO = pStatusN;

		if(pointStart >= 0 && pointEnd >= 0)
		{			
			//DEBF("\r\n XD, P%d: (%d-%d) \n",g_u8XTotalFakePointCnt,pointStart,pointEnd);
			if(g_u8XTotalFakePointCnt < MAX_POINT_CNT)
			{
				X_TotalFakePosition[g_u8XTotalFakePointCnt].pointStart = pointStart;
				X_TotalFakePosition[g_u8XTotalFakePointCnt].pointEnd = pointEnd;
				g_u8XTotalFakePointCnt ++;
			}

			pointStart = -1;
			pointEnd = -1;
		}
				
		if(NoMaskcnt >= D_NO_MASK_POINT_CNT || (i >= (X_POINT_SIZE-1) && areaStart >= 0))
		{
			if(areaStart >= 0 && areaEnd == -1)
				areaEnd = areaStart;
			
			//DEBF("\r\n XD, Area(%d): (%d-%d) \n",g_u8XTotalAreaCnt,areaStart,areaEnd);
			X_TotalArea[g_u8XTotalAreaCnt].start = areaStart;
			X_TotalArea[g_u8XTotalAreaCnt].end = areaEnd;
			g_u8XTotalAreaCnt ++;

			if(g_u8XTotalAreaCnt >= MAX_POINT_CNT)
				break;

			firstPoint = 0;
			NoMaskcnt = 0;
			areaStart = -1;
			areaEnd = -1;
		}
	}
}

uint16_t  VerticalScan_Y_GetFakePoint(uint16_t StartIndex, uint16_t ScanEnd)
{	
	s16 		i;	
	uint8_t     pStatusO,pStatusN;
	int 		pointStart,pointEnd;
	uint16_t	ad_value;
	uint16_t	Count,Sum;

	//uint8_t     j,set,get,chk0;
	//PointAd_t	Pdata[VS_POINT_MAX_WIDTH];

	Count = 0;
	Sum = 0;
	pointStart = -1;
	pointEnd = -1;
	ad_value = 0;
	pStatusO = 0xff;
	pStatusN = 0;

	i = StartIndex;
	
	do
	{		
		if((g_u16YV_ResBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG || 
			(g_u16YV_EcurBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG ||
			(g_u16YV_EcurBuff[i]&VBAD_POINT_FLAG) == VBAD_POINT_FLAG )
		{
			//ERRF("\r\n YV,  P:%d, bad! skip! \n", i);
			i --;
			continue;
		}

		ad_value =	Sample_1_Point(Y_DIRECTION, i, i, g_u16YV_EcurBuff[i]);
		
		//DEBF("\r\n YV, P%d: Ad:%d, Res:%d \n", i, ad_value, g_u16YV_ResBuff[i]);

		if((ad_value <= g_u16YV_ResBuff[i]) || (ad_value <= VER_Y_POINT_MASK_AD_VALUE))// mask
		{
			//DEBF("\r\n YV, P%d: Ad:%d, Res:%d \n", i, ad_value, g_u16YV_ResBuff[i]);
			
			pStatusN = 1;//set mask flag

			//Pdata[Count].ad = ad_value;
			//Pdata[Count].p = i;
			
			Count ++;
			Sum += i;
			
			if(pStatusO == 0 && pointEnd == -1)//get mask start point
			{
				pointEnd = i;
			}
			else if(pointEnd >= 0 && i <= 0 && pointStart == -1)
			{
				pointStart = 0;
			}
		}
		else//no mask		
		{
			pStatusN = 0;//clear mask flag
			if(pStatusO == 1 && pointEnd >= 0 && pointStart == -1)//get mask end point
			{
				pointStart = i+1;
			}
		}
			
		pStatusO = pStatusN;
				
		if((pointStart >= 0) && (pointEnd >= 0))
		{
			//DEBF("\r\n YV, P%d: (%d-%d) ",g_u8YFakePointCnt,pointStart,pointEnd);

			//if(Count <= 3)
			{
				Y_FakePosition[g_u8YFakePointCnt].pointStart = pointStart;
				Y_FakePosition[g_u8YFakePointCnt].pointEnd = pointEnd;
				g_u8YFakePointCnt ++;
			}
			#if 0
			else
			{
				ad_value = Pdata[0].ad;
				Y_FakePosition[g_u8YFakePointCnt].pointEnd = pointEnd;

				get = 0;
				set = 0;
				chk0 = 0;
				for(j = 1; j < Count; j ++)
				{
					if((set == 0) && (chk0 == 0) && ((Pdata[j].ad + Pdata[j].ad >> 1) < ad_value) && (ad_value > 300))
					{
						chk0 = 1;
						Y_FakePosition[g_u8YFakePointCnt].pointEnd = Pdata[j].p;
						ad_value = Pdata[j].ad;
					}
					else if((set == 0) && (Pdata[j].ad < ad_value))
					{
						ad_value = Pdata[j].ad;
					}
					else if((set == 0) && (Pdata[j].ad > (ad_value + ad_value >> 1)) && (Pdata[j].ad > 300))
					{
						get = 1;
						set = 1;
						ad_value = Pdata[j].ad;
						
						Y_FakePosition[g_u8YFakePointCnt].pointStart = Pdata[j].p;
						DEBF("\r\n 0 sub, P%d: (%d-%d) ",g_u8YFakePointCnt,Y_FakePosition[g_u8YFakePointCnt].pointStart,Y_FakePosition[g_u8YFakePointCnt].pointEnd);
						g_u8YFakePointCnt ++;

						if(g_u8YFakePointCnt >= MAX_POINT_CNT)
							break;
					}

					if((set == 1) && (Pdata[j].ad > ad_value))
					{
						ad_value = Pdata[j].ad;
					}
					else if((set == 1) && ((Pdata[j].ad + Pdata[j].ad >> 1) < ad_value))
					{
						set = 0;
						chk0 = 0;
						ad_value = Pdata[j].ad;
						Y_FakePosition[g_u8YFakePointCnt].pointEnd = Pdata[j].p;
					}
				}
				
				if(get == 0)
				{
					Y_FakePosition[g_u8YFakePointCnt].pointStart = pointStart;
					//DEBF("\r\n 1 sub, P%d: (%d-%d) ",g_u8YFakePointCnt,Y_FakePosition[g_u8YFakePointCnt].pointStart,Y_FakePosition[g_u8YFakePointCnt].pointEnd);

					g_u8YFakePointCnt ++;
				}
			}
			#endif
			
			if(g_u8YFakePointCnt >= MAX_POINT_CNT)
				break;

			pointStart = -1;
			pointEnd = -1;
			Count = 0;
			Sum = 0;
		}

		i --;
	}while((i > ScanEnd) || (pointEnd >= 0 && pointStart == -1));

	if(i >= ScanEnd)
	{
		return 0;
	}
	else
	{
		return i;
	}
}

uint16_t  VerticalScan_X_GetFakePoint(uint16_t StartIndex, uint16_t ScanEnd)
{	
	uint16_t 	i;	
	uint8_t     pStatusO,pStatusN;
	int 		pointStart,pointEnd;
	uint16_t	ad_value;
	uint16_t	Count,Sum;
	//uint8_t       j,set,get,chk0;
	//PointAd_t	Pdata[VS_POINT_MAX_WIDTH];

	//DEBF("\r\n VScan step1 direc:0x%x, StartIndex:%d \n",Direction,StartIndex);

	Count = 0;
	Sum = 0;
	pointStart = -1;
	pointEnd = -1;
	ad_value = 0;
	pStatusO = 0xff;
	pStatusN = 0;
	
	i = StartIndex;
	
	do
	{
		if( (g_u16XV_ResBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG || 
			(g_u16XV_EcurBuff[i]&BAD_POINT_FLAG) == BAD_POINT_FLAG ||
			(g_u16XV_EcurBuff[i]&VBAD_POINT_FLAG) == VBAD_POINT_FLAG )
		{
			//ERRF("\r\n VScan direc:0x%x,  point:%d, bad! skip! \n", Direction,i);
			i ++;
			continue;
		}
		
		ad_value = Sample_1_Point(X_DIRECTION, i, i, g_u16XV_EcurBuff[i]);
		
		//DEBF("\r\n P%d: Ad:%d, Res:%d \n", i, ad_value, g_u16XV_ResBuff[i]);

		if((ad_value <= g_u16XV_ResBuff[i]) || (ad_value <= VER_X_POINT_MASK_AD_VALUE))// mask
		{
			//DEBF("\r\n XV, P%d: Ad:%d, Res:%d \n", i, ad_value, g_u16XV_ResBuff[i]);
			
			pStatusN = 1;//set mask flag

			//Pdata[Count].ad = ad_value;
			//Pdata[Count].p = i;

			Count ++;
			Sum += i;
			
			if(pStatusO == 0 && pointStart == -1)//get mask start point
			{
				pointStart = i;
			}
			else if(pointStart >= 0 && i >= (X_POINT_SIZE - 1) && pointEnd == -1)
			{
				pointEnd = X_POINT_SIZE - 1;
			}
		}
		else//no mask		
		{
			pStatusN = 0;//clear mask flag
			if(pStatusO == 1 && pointStart >= 0 && pointEnd == -1)//get mask end point
			{
				pointEnd = i - 1;
			}
		}

		pStatusO = pStatusN;
				
		if(pointStart >= 0 && pointEnd >= 0)
		{
			//DEBF("\r\n XV, P%d: (%d-%d) ",g_u8XFakePointCnt,pointStart,pointEnd);
			
			//if(Count <= 3)
			{
				X_FakePosition[g_u8XFakePointCnt].pointStart = pointStart;
				X_FakePosition[g_u8XFakePointCnt].pointEnd = pointEnd;
				g_u8XFakePointCnt ++;
			}
			#if 0
			else
			{
				ad_value = Pdata[0].ad;
				X_FakePosition[g_u8XFakePointCnt].pointStart = pointStart;
				
				get = 0;
				set = 0;
				chk0 = 0;
				for(j = 1; j < Count; j ++)
				{
					if((set == 0) && (chk0 == 0) && ((Pdata[j].ad + Pdata[j].ad >> 1) < ad_value) && (ad_value > 300))
					{
						chk0 = 1;
						ad_value = Pdata[j].ad;
						X_FakePosition[g_u8XFakePointCnt].pointStart = Pdata[j].p;
					}
					else if((set == 0) && (Pdata[j].ad < ad_value))
					{
						ad_value = Pdata[j].ad;
					}
					else if((set == 0) && (Pdata[j].ad > (ad_value + ad_value >> 1)) && (Pdata[j].ad > 300))
					{
						get = 1;
						set = 1;
						ad_value = Pdata[j].ad;
						
						X_FakePosition[g_u8XFakePointCnt].pointEnd = Pdata[j].p;
						DEBF("\r\n 0 sub, P%d: (%d-%d) ",g_u8XFakePointCnt,X_FakePosition[g_u8XFakePointCnt].pointStart,X_FakePosition[g_u8XFakePointCnt].pointEnd);

						g_u8XFakePointCnt ++;

						if(g_u8XFakePointCnt >= MAX_POINT_CNT)
							break;
					}

					if((set == 1) && (Pdata[j].ad > ad_value))
					{
						ad_value = Pdata[j].ad;
					}
					else if((set == 1) && ((Pdata[j].ad + Pdata[j].ad >> 1) < ad_value))
					{
						set = 0;
						chk0 = 0;
						ad_value = Pdata[j].ad;
						X_FakePosition[g_u8XFakePointCnt].pointStart = Pdata[j].p;
					}
				}
				
				if(get == 0)
				{
					X_FakePosition[g_u8XFakePointCnt].pointEnd = pointEnd;
					//DEBF("\r\n 1 sub, P%d: (%d-%d) ",g_u8XFakePointCnt,X_FakePosition[g_u8XFakePointCnt].pointStart,X_FakePosition[g_u8XFakePointCnt].pointEnd);
					g_u8XFakePointCnt ++;
				}
			}
			#endif

			if(g_u8XFakePointCnt >= MAX_POINT_CNT)
				break;

			pointStart = -1;
			pointEnd = -1;
			Count = 0;
			Sum=0;
		}
		
		i ++;
	}while((i < ScanEnd) || (pointStart >= 0 && pointEnd == -1));

	if(i <= ScanEnd)
	{
		return 0;
	}
	else
	{
		return i;
	}
}

void  QuickReVerticalScanGetFakePoint(uint8_t area)
{
	//uint16_t nextXstart,nextYstart,tmp;
	uint16_t XscanStart,XscanEnd;
	uint16_t YscanStart,YscanEnd;
	uint8_t  xArea,yArea,cnt;

	cnt = 0;
	xArea = 0;
	yArea = 0;
	
	for(yArea = 0; yArea < g_u8YTotalAreaCnt; yArea ++)
	{
		for(xArea = 0; xArea < g_u8XTotalAreaCnt; xArea ++)
		{
			cnt ++;

			if(cnt == area+1)
				break;
		}

		if(cnt == area+1)
			break;
	}
	
	if(Y_TotalArea[yArea].start > QUICK_SCAN_EXTEND_SIZE)
		YscanStart = Y_TotalArea[yArea].start - QUICK_SCAN_EXTEND_SIZE;
	else
		YscanStart = 0;

	YscanEnd = Y_TotalArea[yArea].end + QUICK_SCAN_EXTEND_SIZE;
	if(YscanEnd > Y_POINT_SIZE)
		YscanEnd = Y_POINT_SIZE;

	if(X_TotalArea[xArea].start > QUICK_SCAN_EXTEND_SIZE)
		XscanStart = X_TotalArea[xArea].start - QUICK_SCAN_EXTEND_SIZE;
	else
		XscanStart = 0;
	
	XscanEnd = X_TotalArea[xArea].end + QUICK_SCAN_EXTEND_SIZE;
	if(XscanEnd > X_POINT_SIZE)
		XscanEnd = X_POINT_SIZE;
	
	//DEBF("\r\n ###-Vscan AREA:%d, X(%d-%d), Y(%d-%d)-### \n",area,XscanStart,XscanEnd,YscanStart,YscanEnd);

	if(YscanEnd >= Y_POINT_SIZE)
		YscanEnd = Y_POINT_SIZE - 1;
		
	VerticalScan_Y_GetFakePoint(YscanEnd, YscanStart);
	VerticalScan_X_GetFakePoint(XscanStart, XscanEnd);

#if 0
	nextYstart = VerticalScan_Y_GetFakePoint(YscanEnd, YscanStart);
	nextXstart = VerticalScan_X_GetFakePoint(XscanStart, XscanEnd);

	if(nextYstart != 0)
	{
		if(g_u8YTotalAreaCnt > yArea + 1)
		{
			if(nextYstart < Y_TotalArea[yArea + 1].start)
			{
				Y_TotalArea[yArea + 1].valid = 0;
			}
			else if(Y_TotalArea[yArea + 1].end > nextYstart)
			{
				tmp = Y_TotalArea[yArea + 1].end - nextYstart;
				Y_TotalArea[yArea + 1].end = nextYstart;

				if(Y_TotalArea[yArea + 1].start >= tmp)
					Y_TotalArea[yArea + 1].start -= tmp;
				else
					Y_TotalArea[yArea + 1].start = 0;
			}
		}
	}
	
	if(nextXstart != 0)
	{
		if(g_u8XTotalAreaCnt > xArea + 1)
		{
			if(X_TotalArea[xArea + 1].end < nextXstart)
			{
				X_TotalArea[xArea + 1].valid = 0;
			}
			else if(X_TotalArea[xArea + 1].start < nextXstart)
			{
				tmp = nextXstart - X_TotalArea[xArea + 1].start;
				X_TotalArea[xArea + 1].start = nextXstart;

				if(X_TotalArea[xArea + 1].end + tmp < X_POINT_SIZE)
					X_TotalArea[xArea + 1].end += tmp;
				else
					X_TotalArea[xArea + 1].end = X_POINT_SIZE;
			}
		}
	}
#endif

}

void BuildFakePointMarix(uint8_t area)
{
	uint8_t i,j,k;

	k = 0;

	if(area == 0xff)
	{
		if(g_u8XTotalFakePointCnt > MAX_POINT_CNT)
			g_u8XTotalFakePointCnt = MAX_POINT_CNT;
		if(g_u8YTotalFakePointCnt > MAX_POINT_CNT)
			g_u8YTotalFakePointCnt = MAX_POINT_CNT;

		for(i = 0; i < g_u8XTotalFakePointCnt; i++)// X : Column 
		{
			for(j = 0; j < g_u8YTotalFakePointCnt; j ++)// Y : Line
			{
				g_stAllFakePoint[0][k].trueCnt = 0;
				g_stAllFakePoint[0][k].ad_value = 0xffff;
				
				g_stAllFakePoint[0][k].x_start = X_TotalFakePosition[i].pointStart;
				g_stAllFakePoint[0][k].x_width = X_TotalFakePosition[i].pointEnd - X_TotalFakePosition[i].pointStart + 1;
				g_stAllFakePoint[0][k].y_start = Y_TotalFakePosition[j].pointStart;
				g_stAllFakePoint[0][k].y_width = Y_TotalFakePosition[j].pointEnd - Y_TotalFakePosition[j].pointStart + 1;
			
				//DEBF("\r\n Total P%d,(x:%d,y:%d) \n",k, g_stAllFakePoint[k].x_start + g_stAllFakePoint[k].x_width >> 1, g_stAllFakePoint[k].y_start + g_stAllFakePoint[k].y_width >> 1);
			
				k ++;
				g_u8AllFakePointCnt ++;
			}
		}
	}
	else
	{
		if(g_u8XFakePointCnt > MAX_POINT_CNT)
			g_u8XFakePointCnt = MAX_POINT_CNT;
		if(g_u8YFakePointCnt > MAX_POINT_CNT)
			g_u8YFakePointCnt = MAX_POINT_CNT;

		for(i = 0; i < g_u8XFakePointCnt; i++)// X : Column 
		{
			for(j = 0; j < g_u8YFakePointCnt; j ++)// Y : Line
			{
				g_stAllFakePoint[area][k].trueCnt = 0;
				g_stAllFakePoint[area][k].ad_value = 0xffff;

				g_stAllFakePoint[area][k].x_start = X_FakePosition[i].pointStart;
				g_stAllFakePoint[area][k].x_width = X_FakePosition[i].pointEnd - X_FakePosition[i].pointStart + 1;
				g_stAllFakePoint[area][k].y_start = Y_FakePosition[j].pointStart;
				g_stAllFakePoint[area][k].y_width = Y_FakePosition[j].pointEnd - Y_FakePosition[j].pointStart + 1;
			
				//DEBF("\r\n P%d,(x:%d,y:%d) \n",k, g_stAllFakePoint[k].x_start + (g_stAllFakePoint[k].x_width >> 1), g_stAllFakePoint[k].y_start + (g_stAllFakePoint[k].y_width >> 1));
			
				k ++;
				g_u8AllFakePointCnt ++;
			}
		}
	}
}

void Only1LineFakePointMarixGetPoint(uint8_t area)
{
	uint8_t 	i,marixSize;	

	if(area == 0xff)
	{
		marixSize = g_u8XTotalFakePointCnt * g_u8YTotalFakePointCnt;

		for(i = 0; i < marixSize; i ++)
		{
			g_stCurTouchPoint[g_u8CurTouchPointCnt].x_position = g_stAllFakePoint[0][i].x_start + (g_stAllFakePoint[0][i].x_width >> 1);
			g_stCurTouchPoint[g_u8CurTouchPointCnt].y_position = g_stAllFakePoint[0][i].y_start + (g_stAllFakePoint[0][i].y_width >> 1);	
			DEBF("\r\n TotalFake 1 Line fake(%d) -> true(%d), (x:%d, y:%d) \n",i,g_u8CurTouchPointCnt,g_stCurTouchPoint[g_u8CurTouchPointCnt].x_position,g_stCurTouchPoint[g_u8CurTouchPointCnt].y_position);
			g_u8CurTouchPointCnt ++;

			if(g_u8CurTouchPointCnt >= POINT_BUFFER_SIZE)
			{
				DEBF("\r\n Line:%d, Get point cnt:%d, exit!! \n",__LINE__,g_u8CurTouchPointCnt);
				break;
			}
		}
	}
	else
	{
		marixSize = g_u8XFakePointCnt * g_u8YFakePointCnt;

		for(i = 0; i < marixSize; i ++)
		{
			g_stCurTouchPoint[g_u8CurTouchPointCnt].x_position = g_stAllFakePoint[area][i].x_start + (g_stAllFakePoint[area][i].x_width >> 1);
			g_stCurTouchPoint[g_u8CurTouchPointCnt].y_position = g_stAllFakePoint[area][i].y_start + (g_stAllFakePoint[area][i].y_width >> 1);	
			DEBF("\r\n 1 Line fake(%d) -> true(%d), (x:%d, y:%d) \n",i,g_u8CurTouchPointCnt,g_stCurTouchPoint[g_u8CurTouchPointCnt].x_position,g_stCurTouchPoint[g_u8CurTouchPointCnt].y_position);
			g_u8CurTouchPointCnt ++;

			if(g_u8CurTouchPointCnt >= POINT_BUFFER_SIZE)
			{
				DEBF("\r\n Line:%d, Get point cnt:%d, exit!! \n",__LINE__,g_u8CurTouchPointCnt);
				break;
			}
		}
	}
}

uint16_t  LeanScanAdjustEcur(uint8_t Direction, uint8_t scantype, uint16_t ShiftIndex, uint16_t point)
{
	uint8_t sendShift;
	uint16_t eCur_value,eCur_valueV;
		
	if(Direction == X_DIRECTION || Direction == YX_DIRECTION)
	{
		eCur_valueV = g_u16XV_EcurBuff[point]&VBAD_POINT_FLAG_UNMASK;
		
		if(scantype == XR_LEAN)
			eCur_value = g_u16XR_Offset_EcurBuff[point];
		else
			eCur_value = g_u16XL_Offset_EcurBuff[point];

		sendShift = g_stX_ScanShift[ShiftIndex].sendShift;
		
		if(sendShift < 24)
		{
			eCur_value = eCur_valueV;
		}
		else if(sendShift < 43)
		{
			switch(sendShift)
			{
				case 27:
				case 26:
				case 25:
				case 24:
					eCur_value = eCur_valueV + (sendShift - 23)*10;
					break;
					
				case 28:
					eCur_value -=20;
					break;
				case 29:	
					eCur_value -= 10;
					break;
				case 31:
					eCur_value += 20;
					break;
				case 32:
					eCur_value += 40;
					break;

				case 37:
				case 36:
				case 35:
				case 34:
				case 33:
					eCur_value += 100 + (sendShift - 32)*10;
					break;
					
				case 42:
				case 41:	
				case 40:
				case 39:	
				case 38:
					eCur_value += 200 + (sendShift - 37)*10;
					break;
				
				case 30:
				default:
					break;
			}
			
			if(eCur_value > ECUR_COM_MAX_VALUE)
				eCur_value = ECUR_COM_MAX_VALUE;
		}
		else
		{
			//eCur_value = ECUR_LIMIT_MAX_VALUE;
			eCur_value = ECUR_COM_MAX_VALUE;

		}
	}
	else
	{
		eCur_valueV = g_u16YV_EcurBuff[point]&VBAD_POINT_FLAG_UNMASK;
		
		if(scantype == YR_LEAN)
			eCur_value = g_u16YR_Offset_EcurBuff[point];
		else
			eCur_value = g_u16YL_Offset_EcurBuff[point];

		sendShift = g_stY_ScanShift[ShiftIndex].sendShift;
		
		if(sendShift < 24)
		{
			eCur_value = eCur_valueV;
		}
		else if(sendShift < 65)
		{
			switch(sendShift)
			{
				case 24:
				case 25:	
					eCur_value = eCur_valueV + 10;
					break;
				case 26:
				case 27:
					eCur_value = eCur_valueV + 20;
					break;
				case 28:	
				case 29:
					eCur_value = eCur_valueV + 30;
					break;
				case 30:	
				case 31:
					eCur_value = eCur_valueV + 40;
					break;
				case 32:	
				case 33:
					eCur_value -= 20;
					break;
				case 34:	
				case 35:
				case 36:	
					eCur_value -= 10;
					break;

				case 38:
					eCur_value += 20;
					break;
				case 39:
					eCur_value += 40;
					break;

				case 44:
				case 43:
				case 42:
				case 41:
				case 40:
					eCur_value += 100 + (sendShift - 39)*10;
					break;

				case 49:
				case 48:
				case 47:	
				case 46:
				case 45:
					eCur_value += 200 + (sendShift - 44)*10;
					break;
					
				case 50:
				case 51:
				case 52:
				case 53:
				case 54:
				case 55:
				case 56:
				case 57:
				case 58:
				case 59:
				case 60:
				case 61:
				case 62:
				case 63:
				case 64:
					eCur_value += 260;
					break;
					
				case 37:
				default:
					break;
			}

			if(eCur_value > ECUR_COM_MAX_VALUE)
				eCur_value = ECUR_COM_MAX_VALUE;
		}
		else
		{
			//eCur_value = ECUR_LIMIT_MAX_VALUE;
			eCur_value = ECUR_COM_MAX_VALUE;
		}
	}

	return eCur_value;

}

uint16_t  GetShiftParaIndex(uint8_t ScanDirection, uint8_t scantype, uint16_t x, uint16_t y, uint8_t p)
{
	uint8_t 	k,totalCnt;
	uint16_t	e,offset;
	s16			ps,pr;

	if(ScanDirection == X_DIRECTION)
	{
		offset = g_stX_ShiftArrayIndex[y];
		totalCnt = g_stX_ScanShift[offset].sendShift;
		
		offset ++;
		
		if(scantype == XR_LEAN)
		{
			if(g_u8XR_shiftCnt0[p] < totalCnt)//dec get point time
			{
				//LEAN_DEBF("\r\n %s(%d), XR_shiftCnt0 totalCnt:%d,start:%d. \n",__FUNCTION__,__LINE__,totalCnt,g_u8XR_shiftCnt0[p]);
				for(k = g_u8XR_shiftCnt0[p]; k < totalCnt; k += 2)
				{
					e = offset + k;
					pr = x - g_stX_ScanShift[e].revShift;
					ps = pr + g_stX_ScanShift[e].sendShift;
					
					if(pr >= 0 && ps < X_POINT_SIZE)
					{
						if((g_u16XV_ResBuff[pr]&BAD_POINT_FLAG) == BAD_POINT_FLAG || (g_u16XV_EcurBuff[ps]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
						{
							//ERRF("\r\n X Right Scan, rev or send point:%d, bad! skip! \n", pr);
							continue;
						}

						g_u8XR_shiftCnt0[p] = k + 2;
						g_stDiverScanPara.revShift = pr;
						g_stDiverScanPara.sendShift = ps;
						
						return e;
					}
				}
				g_u8XR_shiftCnt0[p] = totalCnt;
			}
			else
			{
				//LEAN_DEBF("\r\n %s(%d), XR_shiftCnt1 totalCnt:%d,start:%d. \n",__FUNCTION__,__LINE__,totalCnt,g_u8XR_shiftCnt1[p]);
				for(k = g_u8XR_shiftCnt1[p]; k < totalCnt; k += 2)
				{
					e = offset + k;
					pr = x - g_stX_ScanShift[e].revShift;
					ps = pr + g_stX_ScanShift[e].sendShift;
					
					if(pr >= 0 && ps < X_POINT_SIZE)
					{
						if((g_u16XV_ResBuff[pr]&BAD_POINT_FLAG) == BAD_POINT_FLAG || (g_u16XV_EcurBuff[ps]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
						{
							//ERRF("\r\n X Right Scan, rev or send point:%d, bad! skip! \n", pr);
							continue;
						}
						
						g_u8XR_shiftCnt1[p] = k + 2;
						g_stDiverScanPara.revShift = pr;
						g_stDiverScanPara.sendShift = ps;
						
						return e;
					}
				}
				g_u8XR_shiftCnt1[p] = totalCnt;
			}
		}
		else
		{
			if(g_u8XL_shiftCnt0[p] < totalCnt)//dec get point time
			{
				//LEAN_DEBF("\r\n %s(%d), XL_shiftCnt0 totalCnt:%d,start:%d. \n",__FUNCTION__,__LINE__,totalCnt,g_u8XL_shiftCnt0[p]);
				for(k = g_u8XL_shiftCnt0[p]; k < totalCnt; k += 2)
				{
					e = offset + k;
					pr = x + g_stX_ScanShift[e].revShift;
					ps = pr - g_stX_ScanShift[e].sendShift;
					
					if(ps >= 0 && pr < X_POINT_SIZE)
					{
						if((g_u16XV_ResBuff[pr]&BAD_POINT_FLAG) == BAD_POINT_FLAG || (g_u16XV_EcurBuff[ps]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
						{
							//ERRF("\r\n X Left Scan, rev or send point:%d, bad! skip! \n", pr);
							continue;
						}

						g_u8XL_shiftCnt0[p] = k + 2;
						g_stDiverScanPara.revShift = pr;
						g_stDiverScanPara.sendShift = ps;

						return e;
					}
				}
				
				g_u8XL_shiftCnt0[p] = totalCnt;
			}
			else
			{
				//LEAN_DEBF("\r\n %s(%d), XL_shiftCnt1 totalCnt:%d,start:%d. \n",__FUNCTION__,__LINE__,totalCnt,g_u8XL_shiftCnt1[p]);
				for(k = g_u8XL_shiftCnt1[p]; k < totalCnt; k += 2)
				{
					e = offset + k;
					pr = x + g_stX_ScanShift[e].revShift;
					ps = pr - g_stX_ScanShift[e].sendShift;
					
					if(ps >= 0 && pr < X_POINT_SIZE)
					{
						if((g_u16XV_ResBuff[pr]&BAD_POINT_FLAG) == BAD_POINT_FLAG || (g_u16XV_EcurBuff[ps]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
						{
							//ERRF("\r\n X Left Scan, rev or send point:%d, bad! skip! \n", pr);
							continue;
						}

						g_u8XL_shiftCnt1[p] = k + 2;
						g_stDiverScanPara.revShift = pr;
						g_stDiverScanPara.sendShift = ps;

						return e;
					}
				}
				
				g_u8XL_shiftCnt1[p] = totalCnt;
			}
		}
	}
	else
	{
		offset = g_stY_ShiftArrayIndex[x];
		totalCnt = g_stY_ScanShift[offset].sendShift;
		
		offset ++;
		
		if(scantype == YR_LEAN)
		{
			if(g_u8YR_shiftCnt0[p] < totalCnt)
			{
				//LEAN_DEBF("\r\n %s(%d), YR_shiftCnt0 totalCnt:%d,start:%d. \n",__FUNCTION__,__LINE__,totalCnt,g_u8YR_shiftCnt0[p]);
			
				for(k = g_u8YR_shiftCnt0[p]; k < totalCnt; k += 2)
				{
					e = offset + k;
					pr = y + g_stY_ScanShift[e].revShift;
					ps = pr - g_stY_ScanShift[e].sendShift;
					
					if(ps >= 0 && pr < Y_POINT_SIZE)
					{
						if((g_u16YV_ResBuff[pr]&BAD_POINT_FLAG) == BAD_POINT_FLAG || (g_u16YV_EcurBuff[ps]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
						{
							//ERRF("\r\n Y Right Scan, rev or send point:%d, bad! skip! \n", pr);
							continue;
						}

						g_u8YR_shiftCnt0[p] = k + 2;
						g_stDiverScanPara.revShift = pr;
						g_stDiverScanPara.sendShift = ps;
						
						return e;
					}
				}

				g_u8YR_shiftCnt0[p] = totalCnt;
			}
			else
			{
				//LEAN_DEBF("\r\n %s(%d), YR_shiftCnt1 totalCnt:%d,start:%d. \n",__FUNCTION__,__LINE__,totalCnt,g_u8YR_shiftCnt1[p]);
				for(k = g_u8YR_shiftCnt1[p]; k < totalCnt; k += 2)
				{
					e = offset + k;
					pr = y + g_stY_ScanShift[e].revShift;
					ps = pr - g_stY_ScanShift[e].sendShift;
					
					if(ps >= 0 && pr < Y_POINT_SIZE)
					{
						if((g_u16YV_ResBuff[pr]&BAD_POINT_FLAG) == BAD_POINT_FLAG || (g_u16YV_EcurBuff[ps]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
						{
							//ERRF("\r\n Y Right Scan, rev or send point:%d, bad! skip! \n", pr);
							continue;
						}

						g_u8YR_shiftCnt1[p] = k + 2;
						g_stDiverScanPara.revShift = pr;
						g_stDiverScanPara.sendShift = ps;
						
						return e;
					}
				}

				g_u8YR_shiftCnt1[p] = totalCnt;
			}
		}
		else
		{
			if(g_u8YL_shiftCnt0[p] < totalCnt)
			{
				//LEAN_DEBF("\r\n %s(%d), YL_shiftCnt0 totalCnt:%d,start:%d. \n",__FUNCTION__,__LINE__,totalCnt,g_u8YL_shiftCnt0[p]);
				for(k = g_u8YL_shiftCnt0[p]; k < totalCnt; k += 2)
				{
					e = offset + k;
					pr = y - g_stY_ScanShift[e].revShift;
					ps = pr + g_stY_ScanShift[e].sendShift;
					
					if(pr >= 0 && ps < Y_POINT_SIZE)
					{
						if((g_u16YV_ResBuff[pr]&BAD_POINT_FLAG) == BAD_POINT_FLAG || (g_u16YV_EcurBuff[ps]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
						{
							//ERRF("\r\n Y Left Scan, rev or send point:%d, bad! skip! \n", pr);
							continue;
						}

						g_u8YL_shiftCnt0[p] = k + 2;
						g_stDiverScanPara.revShift = pr;
						g_stDiverScanPara.sendShift = ps;
						
						return e;
					}
				}
				
				g_u8YL_shiftCnt0[p] = totalCnt;
			}
			else
			{
				//LEAN_DEBF("\r\n %s(%d), YL_shiftCnt1 totalCnt:%d,start:%d. \n",__FUNCTION__,__LINE__,totalCnt,g_u8YL_shiftCnt1[p]);
				for(k = g_u8YL_shiftCnt1[p]; k < totalCnt; k += 2)
				{
					e = offset + k;
					pr = y - g_stY_ScanShift[e].revShift;
					ps = pr + g_stY_ScanShift[e].sendShift;
					
					if(pr >= 0 && ps < Y_POINT_SIZE)
					{
						if((g_u16YV_ResBuff[pr]&BAD_POINT_FLAG) == BAD_POINT_FLAG || (g_u16YV_EcurBuff[ps]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
						{
							//ERRF("\r\n Y Left Scan, rev or send point:%d, bad! skip! \n", pr);
							continue;
						}

						g_u8YL_shiftCnt1[p] = k + 2;
						g_stDiverScanPara.revShift = pr;
						g_stDiverScanPara.sendShift = ps;
						
						return e;
					}
				}
				
				g_u8YL_shiftCnt1[p] = totalCnt;
			}
		}
	}

	//ERRF("\r\n P%d (%d,%d), total line Cnt:%d < nowLoop:%d, no fit line, skip! continue! \n", p, x, y,totalCnt,k);

	return 0xffff;
}

uint16_t  GetShiftParaIndexReferLast(uint8_t ScanDirection, uint8_t scantype, uint16_t x, uint16_t y, uint8_t p)
{
	uint8_t 	k,totalCnt;
	uint16_t	e,offset;
	s16			ps,pr;

	if(ScanDirection != g_u8LastDirec)
		return 0xffff;
	
	if(ScanDirection == X_DIRECTION)
	{
		offset = g_stX_ShiftArrayIndex[y];
		totalCnt = g_stX_ScanShift[offset].sendShift;
		
		offset ++;
		
		if(scantype == XR_LEAN)
		{
			for(k = 0; k < totalCnt; k ++)
			{
				e = offset + k;
				pr = x - g_stX_ScanShift[e].revShift;
				ps = pr + g_stX_ScanShift[e].sendShift;
				
				if(pr >= 0 && ps < X_POINT_SIZE)
				{
					if((g_u16XV_ResBuff[pr]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
					{
						//ERRF("\r\n ReferLast X Right Scan, rev point:%d, bad! skip! \n", pr);
						continue;
					}
					
					if((g_u16XV_EcurBuff[ps]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
					{
						//ERRF("\r\n ReferLast X Right Scan, send point:%d, bad! skip! \n", ps);
						continue;
					}

					if(pr != g_u16LastRevPoint && ps != g_u16LastSendPoint)
						continue;
					
					//DEBF("\r\n %s, XR Get:%d. (pr:%d -> %d) (ps:%d -> %d) \n",__FUNCTION__,k,pr,g_u16LastRevPoint,ps,g_u16LastSendPoint);
					g_stDiverScanPara.revShift = pr;
					g_stDiverScanPara.sendShift = ps;
					
					return e;
				}
			}
		}
		else
		{
			for(k = 0; k < totalCnt; k ++)
			{
				e = offset + k;
				pr = x + g_stX_ScanShift[e].revShift;
				ps = pr - g_stX_ScanShift[e].sendShift;
				
				if(ps >= 0 && pr < X_POINT_SIZE)
				{
					if((g_u16XV_ResBuff[pr]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
					{
						//ERRF("\r\n ReferLast X Left Scan, rev point:%d, bad! skip! \n", pr);
						continue;
					}
					
					if((g_u16XV_EcurBuff[ps]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
					{
						//ERRF("\r\n ReferLast X Left Scan, send point:%d, bad! skip! \n", ps);
						continue;
					}
					
					if(pr != g_u16LastRevPoint && ps != g_u16LastSendPoint)
						continue;

					//DEBF("\r\n %s, XL Get:%d. (pr:%d -> %d) (ps:%d -> %d) \n",__FUNCTION__,k,pr,g_u16LastRevPoint,ps,g_u16LastSendPoint);
					g_stDiverScanPara.revShift = pr;
					g_stDiverScanPara.sendShift = ps;

					return e;
				}
			}
		}
	}
	else
	{
		offset = g_stY_ShiftArrayIndex[x];
		totalCnt = g_stY_ScanShift[offset].sendShift;
		
		offset ++;
		
		if(scantype == YR_LEAN)
		{
			for(k = 0; k < totalCnt; k ++)
			{
				e = offset + k;
				pr = y + g_stY_ScanShift[e].revShift;
				ps = pr - g_stY_ScanShift[e].sendShift;
				
				if(ps >= 0 && pr < Y_POINT_SIZE)
				{
					if((g_u16YV_ResBuff[pr]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
					{
						//ERRF("\r\n ReferLast Y Right Scan, rev point:%d, bad! skip! \n", pr);
						continue;
					}
					
					if((g_u16YV_EcurBuff[ps]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
					{
						//ERRF("\r\n ReferLast Y Right Scan, send point:%d, bad! skip! \n", ps);
						continue;
					}
					
					if(pr != g_u16LastRevPoint && ps != g_u16LastSendPoint)
						continue;
					
					//DEBF("\r\n %s, YR Get:%d. (pr:%d -> %d) (ps:%d -> %d) \n",__FUNCTION__,k,pr,g_u16LastRevPoint,ps,g_u16LastSendPoint);
					g_stDiverScanPara.revShift = pr;
					g_stDiverScanPara.sendShift = ps;
					
					return e;
				}
			}
		}
		else
		{
			for(k = 0; k < totalCnt; k ++)
			{
				e = offset + k;
				pr = y - g_stY_ScanShift[e].revShift;
				ps = pr + g_stY_ScanShift[e].sendShift;
				
				if(pr >= 0 && ps < Y_POINT_SIZE)
				{
					if((g_u16YV_ResBuff[pr]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
					{
						//ERRF("\r\n ReferLast Y Left Scan, rev point:%d, bad! skip! \n", pr);
						continue;
					}
					
					if((g_u16YV_EcurBuff[ps]&BAD_POINT_FLAG) == BAD_POINT_FLAG)
					{
						//ERRF("\r\n ReferLast Y Left Scan, send point:%d, bad! skip! \n", ps);
						continue;
					}
					
					if(pr != g_u16LastRevPoint && ps != g_u16LastSendPoint)
						continue;

					//DEBF("\r\n %s, YL Get:%d. (pr:%d -> %d) (ps:%d -> %d) \n",__FUNCTION__,k,pr,g_u16LastRevPoint,ps,g_u16LastSendPoint);
					g_stDiverScanPara.revShift = pr;
					g_stDiverScanPara.sendShift = ps;
					
					return e;
				}
			}
		}
	}
	
	//ERRF("\r\n ReferLast P%d (%d,%d), total line Cnt:%d < nowLoop:%d, no fit line, skip! continue! \n", p, x, y,totalCnt,k);

	return 0xffff;
}

void CheckLeanScanMarixMask(uint16_t ad_value, uint8_t scanType, uint16_t p, uint16_t patternPtr, uint8_t patternBit)
{
	if(ad_value > g_stPatternFakeScore[patternPtr][patternBit])
		g_stPatternFakeScore[patternPtr][patternBit] = ad_value;

	if(ad_value >= LEAN_FAKE_POINT_AD_VALUE)//no mask
	{
		if( (g_u8FakePattern[patternPtr] & (0x80 >> patternBit)) != 0)
		{
			g_u8FakePattern[patternPtr] &= ~(0x80 >> patternBit);

			if(g_stAllFakePoint[g_u8CurArea][p].trueCnt > 0)
			{
				g_stAllFakePoint[g_u8CurArea][p].trueCnt --;
			
				if(g_stAllFakePoint[g_u8CurArea][p].trueCnt == 0)
					g_u8FakePointCnt ++;
			}
			else
			{
				ERRF("\r\n ERROR!! scan:0x%x, P%d, TrueCnt = 0, already fake!!   \n", scanType, p);
				ERRF("\r\n p:%d, y:%d, x:%d \n", p, patternPtr, patternBit);
			}
		}
		//DEBF("\r\n scan:0x%x, P%d (%d,%d) is FAKE, ad:%d, (%d,%d) eCur:%d \n", scanType, i, g_stAllFakePoint[i].x_start, g_stAllFakePoint[i].y_start, ad_value, debug_rev_offset, debug_send_offset,debug_ecur);
		//DEBF("\r\n scan:0x%x, P%d (%d,%d) is FAKE, ad:%d \n", scanType, i, g_stAllFakePoint[i].x_start, g_stAllFakePoint[i].y_start, ad_value);
	}
	else if(ad_value >= LEAN_PRE_FAKE_POINT_AD_VALUE)
	{
		//if((g_stPatternPreFakeCnt[patternPtr][patternBit]&POINT_STATUS_MASK) != PRE_FAKE_POINT)
		//	g_stPatternPreFakeCnt[patternPtr][patternBit] = PRE_FAKE_POINT;

		g_stPatternPreFakeCnt[patternPtr][patternBit] |= scanType;

		if(g_stPatternPreFakeCnt[patternPtr][patternBit] & POINT_PRE_STATUS_CNT_MASK == 0x0f)	
		{
			if( (g_u8FakePattern[patternPtr] & (0x80 >> patternBit)) != 0)
			{
				g_u8FakePattern[patternPtr] &= ~(0x80 >> patternBit);

				if(g_stAllFakePoint[g_u8CurArea][p].trueCnt > 0)
				{
					g_stAllFakePoint[g_u8CurArea][p].trueCnt --;
				
					if(g_stAllFakePoint[g_u8CurArea][p].trueCnt == 0)
						g_u8FakePointCnt ++;
				}
				else
				{
					ERRF("\r\n ERROR!! scan:0x%x, P%d, TrueCnt = 0, already fake!!   \n", scanType, p);
					ERRF("\r\n p:%d, y:%d, x:%d \n", p, patternPtr, patternBit);
				}
			}
		}
		
		//DEBF("\r\n scan:0x%x, P%d (%d,%d) is PRE_FAKE, ad:%d, (%d,%d) eCur:%d \n", scanType, i, g_stAllFakePoint[i].x_start, g_stAllFakePoint[i].y_start, ad_value, debug_rev_offset, debug_send_offset,debug_ecur);
		//DEBF("\r\n scan:0x%x, P%d (%d,%d) is PRE_FAKE, ad:%d \n", scanType, i, g_stAllFakePoint[i].x_start, g_stAllFakePoint[i].y_start, ad_value);
	}
}

void LeanScanQuickSamplePoint(LeanScanPointPara_t Data)
{
	uint32_t	t;

	//g_u16TotalDotCnt ++;
	
	if(g_u8QuickSampleFlag == 0)
	{
		g_stShiftLineCache = Data;
		QuickSamplePointSetParam(g_stShiftLineCache);
	}
	else
	{
		t = 0;
		while(CheckTimer3CurTime() != 1)
		{
			t ++;
			if(t >= 30)
			{
				//DEBF("\r\n %s--cnt:%d \n",__FUNCTION__,TIM_GetCounter(TIM3));
				DisableTimer3();
				break;
			}
		}
		
		CheckLeanScanMarixMask(QuickSamplePointGetAd(), g_stShiftLineCache.type, g_stShiftLineCache.p, g_stShiftLineCache.patternPtr, g_stShiftLineCache.patternBit);

		if(g_bFakeCheckOk == TRUE)
			return;
		
		g_stShiftLineCache = Data;
		
		QuickSamplePointSetParam(g_stShiftLineCache);
	}
}

void LeanScanWaitQuickSamplePointOver(void)
{
	//uint32_t	t;
	
	if(g_u8QuickSampleFlag != 0)
	{
	#if 0
		t = 0;
		while(CheckTimer3CurTime() != 1)
		{
			t ++;
			if(t >= 30)
			{
				//DEBF("\r\n %s(%d)--Counter:%d \n",__FUNCTION__,__LINE__,TIM_GetCounter(TIM3));
				DisableTimer3();
				break;
			}
		}

		CheckLeanScanMarixMask(g_stShiftLineCache.p, QuickSamplePointGetAd());
	#else
		DisableTimer3();
		g_u8QuickSampleFlag = 0;
	#endif
	}
}

void  BuildFakePointPattern(void)
{
	uint16_t i,j;
	uint8_t width;

	for(i = 0; i < g_u8MarixSize; i ++)
	{
		switch(g_stAllFakePoint[g_u8CurArea][i].x_width)
		{
			case 1:
				width = 0x80;	// 1000 0000
				break;
			case 2:
				width = 0xC0;	// 1100 0000
				break;
			case 3:
				width = 0xE0;	// 1110 0000
				break;
			case 4:
				width = 0xF0;	// 1111 0000
				break;
			case 5:
				width = 0xF8;	// 1111 1000
				break;
			case 6:
				width = 0xFC;	// 1111 1100
				break;	
			case 7:
				width = 0xFE;	// 1111 1110
				break;
			case 8:
				width = 0xFF;	// 1111 1111
				break;
			default:
				width = 0x00;
				break;
		}

		g_stAllFakePoint[g_u8CurArea][i].patternS = g_u16FakePatternCnt;

		for(j = 0; j < g_stAllFakePoint[g_u8CurArea][i].y_width; j ++)
		{
			//DEBF("\r\n 0-Build, P%d, Yindex:%d, X:0x%x \n",i,g_u16FakePatternCnt,width);
			
			g_u8FakePattern[g_u16FakePatternCnt] = width;
			g_u16FakePatternCnt ++;
		}

		g_stAllFakePoint[g_u8CurArea][i].patternSize = g_stAllFakePoint[g_u8CurArea][i].x_width * g_stAllFakePoint[g_u8CurArea][i].y_width;
		g_stAllFakePoint[g_u8CurArea][i].trueCnt = g_stAllFakePoint[g_u8CurArea][i].patternSize;

		//DEBF("\r\n P%d, (%d, %d) -> MaxTrueCnt:%d \n",i, g_stAllFakePoint[g_u8CurArea][i].x_width,g_stAllFakePoint[g_u8CurArea][i].y_width,g_stAllFakePoint[g_u8CurArea][i].TrueCnt);

		g_u16AllFakeDotCnt += g_stAllFakePoint[g_u8CurArea][i].patternSize;
	}
}

void  GetTruePointInPattern(uint8_t area)
{
	uint8_t tmp;
	uint16_t i,m,n;
	uint16_t patternPtr;
	uint16_t totalX,totalY;
	
	for(i = 0; i < g_u8MarixSize; i ++)
	{
		if(g_stAllFakePoint[area][i].trueCnt == 0) // fake point, skip
		{
			if(g_bOneLineFake == TRUE)// 1 line area, all point as true!
			{
				g_bReSearch = TRUE;
				ERRF("\r\n Line:%d, 1 line, Get point error happened, drop and re-search!! \n",__LINE__);
				return;
			}
			
			continue;
		}

		if((g_stAllFakePoint[area][i].trueCnt & POINT_STATUS_MASK) != FAKE_POINT)
		{
			totalX = 0;
			totalY = 0;
			patternPtr = g_stAllFakePoint[area][i].patternS;
				
			for(m = 0; m < g_stAllFakePoint[area][i].y_width; m ++)
			{
				tmp = 0x80;

				for(n = 0; n < g_stAllFakePoint[area][i].x_width; n ++)
				{
					if((g_u8FakePattern[patternPtr] & tmp) != 0)
					{
						totalX += n;
						totalY += m;
					}
					tmp >>= 1;
				}
				
				patternPtr ++;
			}

			g_stAllFakePoint[area][i].x_start = g_stAllFakePoint[area][i].x_start + (totalX / g_stAllFakePoint[area][i].trueCnt);
			g_stAllFakePoint[area][i].y_start = g_stAllFakePoint[area][i].y_start + (totalY / g_stAllFakePoint[area][i].trueCnt);
			//g_stAllFakePoint[g_u8CurArea][i].x_width = 1;
			//g_stAllFakePoint[g_u8CurArea][i].y_width = 1;

			//g_stCurTouchPoint[g_u8CurTouchPointCnt].status.u8Value = area;

			g_stCurTouchPoint[g_u8CurTouchPointCnt].trueCnt = g_stAllFakePoint[area][i].trueCnt;
			g_stCurTouchPoint[g_u8CurTouchPointCnt].ad_value = g_stAllFakePoint[area][i].ad_value;
			g_stCurTouchPoint[g_u8CurTouchPointCnt].x_position = g_stAllFakePoint[area][i].x_start;
			g_stCurTouchPoint[g_u8CurTouchPointCnt].y_position = g_stAllFakePoint[area][i].y_start;	
			DEBF("\r\n	0 Pattern fake(P%d) -> true(P%d), (x:%d, y:%d) \n",i,g_u8CurTouchPointCnt,g_stCurTouchPoint[g_u8CurTouchPointCnt].x_position,g_stCurTouchPoint[g_u8CurTouchPointCnt].y_position);
			g_u8CurTouchPointCnt ++;
			g_u8TruePointCnt ++;

			if(g_u8CurTouchPointCnt >= POINT_BUFFER_SIZE)
			{
				//DEBF("\r\n Line:%d, Get point cnt:%d, exit!! \n",__LINE__,g_u8CurTouchPointCnt);
				break;
			}
		}
		else
		{
			//g_stCurTouchPoint[g_u8CurTouchPointCnt].status.u8Value = area;
			g_stCurTouchPoint[g_u8CurTouchPointCnt].trueCnt = g_stAllFakePoint[area][i].trueCnt;
			g_stCurTouchPoint[g_u8CurTouchPointCnt].ad_value = g_stAllFakePoint[area][i].ad_value;
			g_stCurTouchPoint[g_u8CurTouchPointCnt].x_position = g_stAllFakePoint[area][i].x_start + (g_stAllFakePoint[area][i].x_width >> 1);
			g_stCurTouchPoint[g_u8CurTouchPointCnt].y_position = g_stAllFakePoint[area][i].y_start + (g_stAllFakePoint[area][i].y_width >> 1);	
			DEBF("\r\n	1 Pattern fake(P%d) -> true(P%d), (x:%d, y:%d) \n",i,g_u8CurTouchPointCnt,g_stCurTouchPoint[g_u8CurTouchPointCnt].x_position,g_stCurTouchPoint[g_u8CurTouchPointCnt].y_position);
			g_u8CurTouchPointCnt ++;
			g_u8TruePointCnt ++;

			if(g_u8CurTouchPointCnt >= POINT_BUFFER_SIZE)
			{
				//DEBF("\r\n Line:%d, Get point cnt:%d, exit!! \n",__LINE__,g_u8CurTouchPointCnt);
				break;
			}
		}
	}
}

void  _LeanScanIncSearchOtherFitPoint(uint8_t ScanDirection, uint8_t scantype, uint8_t p, uint8_t py, uint8_t px)
{
	uint8_t 	tmp;
	s8			i,m,n;
	uint16_t	index,x,y,k;
	LeanScanPointPara_t scanData;

	scanData.direc = ScanDirection;
	scanData.type = scantype;

	for(i = p; i < g_u8MarixSize; i ++)
	{
		if(g_stAllFakePoint[g_u8CurArea][i].trueCnt == 0)
		{
			py = 0;	px = 0;
			continue;
		}
		
		scanData.p = i;
		
		k = g_stAllFakePoint[g_u8CurArea][i].patternS;
		
		for(m = py; m < g_stAllFakePoint[g_u8CurArea][i].y_width; m ++)
		{
			scanData.patternPtr = k;
			
			for(n = px; n < g_stAllFakePoint[g_u8CurArea][i].x_width; n ++)
			{
				tmp = 0x80 >> n;
				
				if(((g_u8FakePattern[k] & tmp) != 0) && ((g_stDotMark[i][m] & tmp) == 0))
				{
					index = 0xffff;
					scanData.patternBit = n;
					
					x = g_stAllFakePoint[g_u8CurArea][i].x_start + n;
					y = g_stAllFakePoint[g_u8CurArea][i].y_start + m;

					index = GetShiftParaIndexReferLast(ScanDirection, scantype, x, y, i);
					
					if(index == 0xffff)
						continue;

					g_stDotMark[i][m] |= tmp;

					//g_u16GetDotFitCnt ++;

					scanData.revP = g_stDiverScanPara.revShift;
					scanData.sendP = g_stDiverScanPara.sendShift;
					
					//DEBF("\r\n Inc fit, p:%d, y:%d, x:%d \n", i, scanData.patternPtr, n);
					
					scanData.eCur = LeanScanAdjustEcur(ScanDirection, scantype, index, scanData.sendP);
					
					LeanScanQuickSamplePoint(scanData);
				}
			}
			k ++;
		}

		py = 0;	px = 0;
	}
}

void  LeanScanMarixIncElement(uint8_t ScanDirection, uint8_t scantype)
{
	uint8_t		tmp;
	s8			i,j,m,n;
	uint16_t	index,x,y,k;
	LeanScanPointPara_t scanData;

	for(i = 0; i < g_u8MarixSize; i ++)
	{
		for(j = 0; j < POINT_MAX_WIDTH; j ++)
		{
			g_stDotMark[i][j] = 0;
		}
	}
	
	scanData.direc = ScanDirection;
	scanData.type = scantype;

	for(i = 0; i < g_u8MarixSize; i ++)
	{
		if(g_stAllFakePoint[g_u8CurArea][i].trueCnt == 0) 
			continue;
		
		scanData.p = i;
		
		k = g_stAllFakePoint[g_u8CurArea][i].patternS;
		  
		for(m = 0; m < g_stAllFakePoint[g_u8CurArea][i].y_width; m ++)
		{
			scanData.patternPtr = k;
			
			tmp = 0x80;
			for(n = 0; n < g_stAllFakePoint[g_u8CurArea][i].x_width; n ++)
			{
				if(((g_u8FakePattern[k] & tmp) != 0) && ((g_stDotMark[i][m] & tmp) == 0))
				{
					index = 0xffff;
					scanData.patternBit = n;
				
					x = g_stAllFakePoint[g_u8CurArea][i].x_start + n;
					y = g_stAllFakePoint[g_u8CurArea][i].y_start + m;
					
					index = GetShiftParaIndex(ScanDirection, scantype, x, y, i);

					g_stDotMark[i][m] |= tmp; // mean:  1. index == 0xffff, skip it; 2. index != 0xffff, scan it

					if(index == 0xffff)
						continue;
					
					//g_u16ShiftDotCnt ++;
					
					scanData.revP = g_stDiverScanPara.revShift;
					scanData.sendP = g_stDiverScanPara.sendShift;
					
					scanData.eCur = LeanScanAdjustEcur(ScanDirection, scantype, index, scanData.sendP);
										
					LeanScanQuickSamplePoint(scanData);

					//DEBF("\r\n Inc, p:%d, y:%d, x:%d \n", i, scanData.patternPtr, n);
					_LeanScanIncSearchOtherFitPoint(ScanDirection, scantype, i, m, n+1);

					if(g_u8FakePointCnt >= g_u8MaxFake && g_u8MaxFake > 0)
					{
						//if(g_stAllFakePoint[g_u8CurArea][0].TrueCnt == 0 && g_stAllFakePoint[g_u8CurArea][3].TrueCnt == 0 ||
						//   g_stAllFakePoint[g_u8CurArea][1].TrueCnt == 0 && g_stAllFakePoint[g_u8CurArea][2].TrueCnt == 0)
						{
							g_bFakeCheckOk = TRUE;
							
							DEBF("\r\n FakePointCnt:%d >= MaxFake:%d, exit lean scan!! \n",g_u8FakePointCnt,g_u8MaxFake);
							return;
						}
					}
				}
				tmp >>= 1;
			}
			k ++;
		}
	}
}

void  _LeanScanDecSearchOtherFitPoint(uint8_t ScanDirection, uint8_t scantype, uint8_t co, uint8_t li,uint8_t py, uint8_t px)
{
	uint8_t 	tmp;
	s8			i,m,n,col,line;
	uint16_t	index,x,y,k;
	LeanScanPointPara_t scanData;

	scanData.direc = ScanDirection;
	scanData.type = scantype;

	for(col = co; col < g_u8XFakePointCnt; col ++)
	{
		for(line = li; line >= 0; line --)
		{
			i = line + g_u8YFakePointCnt * col;
			
			if(g_stAllFakePoint[g_u8CurArea][i].trueCnt == 0)
			{
				py = 0;	px = 0;
				continue;
			}
			
			scanData.p = i;
			
			k = g_stAllFakePoint[g_u8CurArea][i].patternS;
			
			for(m = py; m < g_stAllFakePoint[g_u8CurArea][i].y_width; m ++)
			{
				scanData.patternPtr = k;
				
				for(n = px; n < g_stAllFakePoint[g_u8CurArea][i].x_width; n ++)
				{
					tmp = 0x80 >> n;
					
					if(((g_u8FakePattern[k] & tmp) != 0) && ((g_stDotMark[i][m] & tmp) == 0))
					{
						index = 0xffff;
						scanData.patternBit = n;
						
						x = g_stAllFakePoint[g_u8CurArea][i].x_start + n;
						y = g_stAllFakePoint[g_u8CurArea][i].y_start + m;

						index = GetShiftParaIndexReferLast(ScanDirection, scantype, x, y, i);
						
						if(index == 0xffff)
							continue;

						g_stDotMark[i][m] |= tmp;
						//g_u16GetDotFitCnt ++;

						scanData.revP = g_stDiverScanPara.revShift;
						scanData.sendP = g_stDiverScanPara.sendShift;
						
						//DEBF("\r\n Dec fit, p:%d, y:%d, x:%d \n", i, scanData.patternPtr, n);
						
						scanData.eCur = LeanScanAdjustEcur(ScanDirection, scantype, index, scanData.sendP);
						
						LeanScanQuickSamplePoint(scanData);
					}
				}
				k ++;
			}

			py = 0;	px = 0;
		}

		li = g_u8YFakePointCnt - 1;
	}
}

void  LeanScanMarixDecElement(uint8_t ScanDirection, uint8_t scantype)
{
	uint8_t		tmp;
	s8			i,j,m,n,col,line;
	uint16_t	index,x,y,k;
	LeanScanPointPara_t scanData;

	for(i = 0; i < g_u8MarixSize; i ++)
	{
		for(j = 0; j < POINT_MAX_WIDTH; j ++)
		{
			g_stDotMark[i][j] = 0;
		}
	}
	
	scanData.direc = ScanDirection;
	scanData.type = scantype;

	for(col = 0; col < g_u8XFakePointCnt; col ++)
	{
		for(line = g_u8YFakePointCnt-1; line >= 0; line --)
		{
			i = line + g_u8YFakePointCnt * col;
				
			if(g_stAllFakePoint[g_u8CurArea][i].trueCnt == 0) 
				continue;
			
			scanData.p = i;
			
			k = g_stAllFakePoint[g_u8CurArea][i].patternS;
			  
			for(m = 0; m < g_stAllFakePoint[g_u8CurArea][i].y_width; m ++)
			{
				scanData.patternPtr = k;
				
				tmp = 0x80;
				for(n = 0; n < g_stAllFakePoint[g_u8CurArea][i].x_width; n ++)
				{
					if(((g_u8FakePattern[k] & tmp) != 0) && ((g_stDotMark[i][m] & tmp) == 0))
					{
						index = 0xffff;
						scanData.patternBit = n;
					
						x = g_stAllFakePoint[g_u8CurArea][i].x_start + n;
						y = g_stAllFakePoint[g_u8CurArea][i].y_start + m;
						
						index = GetShiftParaIndex(ScanDirection, scantype, x, y, i);

						g_stDotMark[i][m] |= tmp; // mean:  1. index == 0xffff, skip it; 2. index != 0xffff, scan it

						if(index == 0xffff)
							continue;
						
						//g_u16ShiftDotCnt ++;
						
						scanData.revP = g_stDiverScanPara.revShift;
						scanData.sendP = g_stDiverScanPara.sendShift;
						
						scanData.eCur = LeanScanAdjustEcur(ScanDirection, scantype, index, scanData.sendP);
						
						LeanScanQuickSamplePoint(scanData);
						
						//DEBF("\r\n Dec, p:%d, y:%d, x:%d \n", i, scanData.patternPtr, n);

						_LeanScanDecSearchOtherFitPoint(ScanDirection, scantype, col, line, m, n+1);

						if(g_u8FakePointCnt >= g_u8MaxFake && g_u8MaxFake > 0)
						{
							//if(g_stAllFakePoint[g_u8CurArea][0].TrueCnt == 0 && g_stAllFakePoint[g_u8CurArea][3].TrueCnt == 0 ||
							//   g_stAllFakePoint[g_u8CurArea][1].TrueCnt == 0 && g_stAllFakePoint[g_u8CurArea][2].TrueCnt == 0)
							{
								g_bFakeCheckOk = TRUE;
								
								DEBF("\r\n FakePointCnt:%d >= MaxFake:%d, exit lean scan!! \n",g_u8FakePointCnt,g_u8MaxFake);
								return;
							}
						}
					}
					tmp >>= 1;
				}
				k ++;
			}
		}
	}
}

void CountPointAdValue(void)
{
	uint8_t 	i,m,n;
	uint16_t	tmp,fakeScore,patternPtr;

	g_u8ValidPointCnt = 0;

	for(i = 0; i < g_u8MarixSize; i ++)
	{
		fakeScore = 0;
		patternPtr = g_stAllFakePoint[g_u8CurArea][i].patternS;
		for(m = 0; m < g_stAllFakePoint[g_u8CurArea][i].y_width; m ++)
		{
			tmp = 0x80;

			for(n = 0; n < g_stAllFakePoint[g_u8CurArea][i].x_width; n ++)
			{
				if((g_u8FakePattern[patternPtr] & tmp) != 0)
				{
					g_u16AdScore[g_u8ValidPointCnt] += g_stPatternFakeScore[patternPtr][n];
				}
				else
				{
					fakeScore += g_stPatternFakeScore[patternPtr][n];
				}
				
				tmp >>= 1;
			}
			
			patternPtr ++;
		}
		
		if(g_stAllFakePoint[g_u8CurArea][i].trueCnt > 0)
		{
			g_u8TrueCntScore[g_u8ValidPointCnt] = g_stAllFakePoint[g_u8CurArea][i].trueCnt;
			g_u16AdScore[g_u8ValidPointCnt] /= g_stAllFakePoint[g_u8CurArea][i].trueCnt;
			g_stAllFakePoint[g_u8CurArea][i].ad_value = g_u16AdScore[g_u8ValidPointCnt];
			
			g_u8ValidPointCnt ++;
		}
		else
		{
			g_stAllFakePoint[g_u8CurArea][i].ad_value = fakeScore / (g_stAllFakePoint[g_u8CurArea][i].y_width * g_stAllFakePoint[g_u8CurArea][i].x_width);
		}
	}
}


void OrderPointTrueCntAndAdValueScore(void)
{
	uint8_t 	i,j,trueM,adM;
	uint16_t	tmp;
		
	//order  true score bigger -> small, and order  ad score small -> bigger 
	for(i = 0; i < g_u8ValidPointCnt - 1; i ++)
	{
		adM = i;
		trueM = i;
		
		for(j = i + 1; j < g_u8ValidPointCnt; j ++)
		{
			if(g_u16AdScore[adM] > g_u16AdScore[j])
				adM = j;

			if(g_u8TrueCntScore[trueM] < g_u8TrueCntScore[j])
				trueM = j;
		}

		if(adM != i)
		{
			tmp = g_u16AdScore[i];
			g_u16AdScore[i] = g_u16AdScore[adM];
			g_u16AdScore[adM] = tmp;
		}
		
		if(trueM != i)
		{
			tmp = g_u8TrueCntScore[i];
			g_u8TrueCntScore[i] = g_u8TrueCntScore[trueM];
			g_u8TrueCntScore[trueM] = tmp;
		}
	}
}

void _PositionFilterSubCheck(uint8_t i, uint8_t j, uint8_t *pNaddId, uint8_t *pNdropId)
{
	if(g_stAllFakePoint[g_u8CurArea][i].trueCnt > 0 && g_stAllFakePoint[g_u8CurArea][j].trueCnt > 0)
	{
		if(g_stAllFakePoint[g_u8CurArea][j].trueCnt != g_stAllFakePoint[g_u8CurArea][i].trueCnt)
		{
			if(g_stAllFakePoint[g_u8CurArea][j].trueCnt > g_stAllFakePoint[g_u8CurArea][i].trueCnt)
			{
				if(*pNdropId == 0xff)
					*pNdropId = i;
				else
				{
					*pNdropId = j;
					DEBF("\r\n	PointFilter-Lv2, trueCnt drop P%d, TrueCnt:%d, ad:%d \n",i,g_stAllFakePoint[g_u8CurArea][i].trueCnt,g_stAllFakePoint[g_u8CurArea][i].ad_value);
					g_stAllFakePoint[g_u8CurArea][i].trueCnt = 0;
					g_u8FakePointCnt ++;
				}
			}
			else
			{
				if(*pNdropId == 0xff)
					*pNdropId = j;
				else
				{
					DEBF("\r\n	PointFilter-Lv2, trueCnt drop P%d, TrueCnt:%d, ad:%d \n",j,g_stAllFakePoint[g_u8CurArea][j].trueCnt,g_stAllFakePoint[g_u8CurArea][j].ad_value);
					g_stAllFakePoint[g_u8CurArea][j].trueCnt = 0;
					g_u8FakePointCnt ++;
				}
			}
		}
		else
		{
			if(g_stAllFakePoint[g_u8CurArea][j].ad_value < g_stAllFakePoint[g_u8CurArea][i].ad_value)
			{
				if(*pNdropId == 0xff)
					*pNdropId = i;
				else
				{
					*pNdropId = j;
					DEBF("\r\n	PointFilter-Lv2, ad drop P%d, TrueCnt:%d, ad:%d \n",i,g_stAllFakePoint[g_u8CurArea][i].trueCnt,g_stAllFakePoint[g_u8CurArea][i].ad_value);
					g_stAllFakePoint[g_u8CurArea][i].trueCnt = 0;
					g_u8FakePointCnt ++;
				}
			}
			else
			{
				if(*pNdropId == 0xff)
					*pNdropId = j;
				else
				{
					DEBF("\r\n	PointFilter-Lv2, ad drop P%d, TrueCnt:%d, ad:%d \n",j,g_stAllFakePoint[g_u8CurArea][j].trueCnt,g_stAllFakePoint[g_u8CurArea][j].ad_value);
					g_stAllFakePoint[g_u8CurArea][j].trueCnt = 0;
					g_u8FakePointCnt ++;
				}
			}
		}
	}
	else if(*pNdropId == 0xff && *pNaddId != 0xf0 && g_stAllFakePoint[g_u8CurArea][i].trueCnt == 0 && g_stAllFakePoint[g_u8CurArea][j].trueCnt == 0)
	{
		if(g_stAllFakePoint[g_u8CurArea][j].ad_value > g_stAllFakePoint[g_u8CurArea][i].ad_value)
		{
			*pNaddId = i;
		}
		else
		{
			*pNaddId = j;
		}
	}
	else// no need add point
	{
		*pNaddId = 0xf0;
	}
}


void _MultiPositionFilterPoint(uint8_t direction, bool bMore)
{
	//uint8_t	rechk;
	uint8_t	i,j,col,line;
	uint8_t	nDropCnt,nDropData[6];
	uint8_t miniTrue,reservePoint,dropPoint,decCnt;
	uint8_t NaddId,NdropId,sameId;
	uint16_t tmp;


	//rechk = 1;
	bMore = TRUE;

	nDropCnt = 0;

	if(direction == X_DIRECTION)
	{
		for(col = 0; col < g_u8XFakePointCnt; col ++)
		{
			NaddId = 0xff;
			NdropId = 0xff;
			for(line = 0; line < g_u8YFakePointCnt-1; line ++)
			{
				if(NdropId == 0xff)
					i = g_u8YFakePointCnt*col + line;
				else
					i = NdropId;
				
				j = g_u8YFakePointCnt*col + line + 1;

				_PositionFilterSubCheck(i, j, &NaddId, &NdropId);
			}

			if(NdropId != 0xff)
			{
				if(bMore == TRUE)
				{
					nDropData[nDropCnt] = NdropId;
					nDropCnt ++;
				}
				else
				{
					DEBF("\r\n	PointFilter-Lv1, drop P%d, TrueCnt:%d, ad:%d  \n",NdropId,g_stAllFakePoint[g_u8CurArea][NdropId].trueCnt,g_stAllFakePoint[g_u8CurArea][NdropId].ad_value);
					g_stAllFakePoint[g_u8CurArea][NdropId].trueCnt = 0;
					g_u8FakePointCnt ++;
				}
			}
			else if(NaddId != 0xff && NaddId != 0xf0)
			{
				DEBF("\r\n	PointFilter-Lv1, add P%d, TrueCnt:%d, ad:%d \n",NaddId,g_stAllFakePoint[g_u8CurArea][NaddId].trueCnt,g_stAllFakePoint[g_u8CurArea][NaddId].ad_value);
				g_stAllFakePoint[g_u8CurArea][NaddId].trueCnt = FAKE_POINT | g_stAllFakePoint[g_u8CurArea][NaddId].patternSize;
				g_u8FakePointCnt --;
			}
		}
		
	}
	else
	{
		for(line = 0; line < g_u8YFakePointCnt; line ++)
		{
			NdropId = 0xff;
			for(col = 0; col < g_u8XFakePointCnt-1; col ++)
			{
				if(NdropId == 0xff)
					i = g_u8YFakePointCnt*col + line;
				else
					i = NdropId;
				
				j = g_u8YFakePointCnt*(col+1) + line;
				
				_PositionFilterSubCheck(i, j, &NaddId, &NdropId);
			}
			
			if(NdropId != 0xff)
			{
				if(bMore == TRUE)
				{
					nDropData[nDropCnt] = NdropId;
					nDropCnt ++;
				}
				else
				{
					DEBF("\r\n	PointFilter-Lv1, drop P%d, TrueCnt:%d, ad:%d  \n",NdropId,g_stAllFakePoint[g_u8CurArea][NdropId].trueCnt,g_stAllFakePoint[g_u8CurArea][NdropId].ad_value);
					g_stAllFakePoint[g_u8CurArea][NdropId].trueCnt = 0;
					g_u8FakePointCnt ++;
				}
			}
			else if(NaddId != 0xff && NaddId != 0xf0)
			{
				DEBF("\r\n	PointFilter-Lv1, add P%d, TrueCnt:%d, ad:%d \n",NaddId,g_stAllFakePoint[g_u8CurArea][NaddId].trueCnt,g_stAllFakePoint[g_u8CurArea][NaddId].ad_value);
				g_stAllFakePoint[g_u8CurArea][NaddId].trueCnt = FAKE_POINT | g_stAllFakePoint[g_u8CurArea][NaddId].patternSize;
				g_u8FakePointCnt --;
			}
		}
	}

	if(bMore == TRUE)
	{
		miniTrue = (g_u8XFakePointCnt > g_u8YFakePointCnt) ? g_u8XFakePointCnt : g_u8YFakePointCnt;
		reservePoint = MAX_POINT_CNT - miniTrue;

		if(nDropCnt > reservePoint)
		{
			decCnt = 0;
			dropPoint = nDropCnt - reservePoint;
			
			i = 0;
			do
			{
				NdropId = i;
				sameId = NdropId;
				tmp = g_stAllFakePoint[g_u8CurArea][nDropData[i]].trueCnt;

				for(j = i + 1; j < nDropCnt; j ++)
				{
					if(nDropData[j] != 0xff)
					{
						if(g_stAllFakePoint[g_u8CurArea][nDropData[j]].trueCnt < tmp)
						{
							NdropId = j;
							tmp = g_stAllFakePoint[g_u8CurArea][nDropData[j]].trueCnt;
						}
						else if(g_stAllFakePoint[g_u8CurArea][nDropData[j]].trueCnt == tmp)
						{
							sameId = j;
						}
					}
				}

				if(sameId != NdropId)
					break;
				
				decCnt ++;
				
				DEBF("\r\n	PointFilter-Lv3, reserve loop, trueCnt drop, P%d, TrueCnt:%d, ad:%d  \n",nDropData[NdropId], g_stAllFakePoint[g_u8CurArea][nDropData[NdropId]].trueCnt, g_stAllFakePoint[g_u8CurArea][nDropData[NdropId]].ad_value);
				g_stAllFakePoint[g_u8CurArea][nDropData[NdropId]].trueCnt = 0;
				g_u8FakePointCnt ++;
				nDropData[NdropId] = 0xff;

				if(decCnt >= dropPoint)
					return;

				i ++;
			}while(i < nDropCnt - 1);
			
			i = 0;
			do
			{
				NdropId = i;
				sameId = NdropId;
				tmp = g_stAllFakePoint[g_u8CurArea][nDropData[i]].ad_value;

				for(j = i + 1; j < nDropCnt; j ++)
				{
					if(nDropData[j] != 0xff)
					{
						if(g_stAllFakePoint[g_u8CurArea][nDropData[j]].ad_value > tmp)
						{
							NdropId = j;
							tmp = g_stAllFakePoint[g_u8CurArea][nDropData[j]].ad_value;
						}
						else if(g_stAllFakePoint[g_u8CurArea][nDropData[j]].ad_value == tmp)
						{
							sameId = j;
						}
					}
				}

				if(sameId != NdropId)
					break;
				
				decCnt ++;
				
				DEBF("\r\n	PointFilter-Lv4, reserve loop, ad drop, P%d, TrueCnt:%d, ad:%d  \n",nDropData[NdropId], g_stAllFakePoint[g_u8CurArea][nDropData[NdropId]].trueCnt, g_stAllFakePoint[g_u8CurArea][nDropData[NdropId]].ad_value);
				g_stAllFakePoint[g_u8CurArea][nDropData[NdropId]].trueCnt = 0;
				g_u8FakePointCnt ++;
				nDropData[NdropId] = 0xff;

				if(decCnt >= dropPoint)
					return;

				i ++;
			}while(i < nDropCnt - 1);
		}
	}
}

void UsePositionFilterPoint(void)
{
	bool	bMore;
	uint8_t direction;

	direction = X_DIRECTION;
	
	switch(g_u8MarixSize)
	{
		case 8: // 4 x 2
			if(g_u8YFakePointCnt == 4)
			direction = Y_DIRECTION;
		case 4: // 2 x 2	
			_MultiPositionFilterPoint(direction, FALSE);
			break;
			
		case 6: // 3 x 2
			if(g_u8YFakePointCnt == 3)
			direction = Y_DIRECTION;
		case 9: // 3 x 3	
			if(g_u8LastTouchPointCnt == 4)
				bMore = TRUE;
			else
				bMore = FALSE;
			_MultiPositionFilterPoint(direction, bMore);
			break;
			
		case 12: // 4 x 3 
			if(g_u8YFakePointCnt == 4)
			direction = Y_DIRECTION;
		case 16: // 4 x 4
			_MultiPositionFilterPoint(direction, FALSE);
			break;
			
		default:
			break;
	}
}

void  UseScoreFilterPoint(void)
{
	uint16_t i,j;
	
	for(i = 0; i < g_u8MarixSize; i ++)
	{
		if(g_stAllFakePoint[g_u8CurArea][i].trueCnt != 0) 
		{
			for(j = 0; j < g_u8MarixSize - g_u8MaxFake; j ++)
			{
				if(g_stAllFakePoint[g_u8CurArea][i].trueCnt >= g_u8TrueCntScore[j])
					break;
			}

			if((j > 0) && (j >= g_u8MarixSize - g_u8MaxFake))
			{
				//DEBF("\r\n	TrueCnt-ScoreFilter, P%d, TrueCnt:%d is small, filter drop!! \n",i,g_stAllFakePoint[g_u8CurArea][i].trueCnt);
				g_stAllFakePoint[g_u8CurArea][i].trueCnt = 0;
				g_u8FakePointCnt ++;
			}
		}
	}

	if(g_u8FakePointCnt >= g_u8MaxFake)
		return;

	for(i = 0; i < g_u8MarixSize; i ++)
	{
		if(g_stAllFakePoint[g_u8CurArea][i].trueCnt != 0)
		{
			for(j = 0; j < g_u8MarixSize - g_u8MaxFake; j ++)
			{
				if(g_stAllFakePoint[g_u8CurArea][i].ad_value <= g_u16AdScore[j])
					break;
			}

			if((j > 0) && (j >= g_u8MarixSize - g_u8MaxFake))
			{
				//DEBF("\r\n	Ad-ScoreFilter, P%d, AD_Score:%d is bigger, filter drop!! \n",i,g_stAllFakePoint[g_u8CurArea][i].ad_value);
				g_stAllFakePoint[g_u8CurArea][i].trueCnt = 0;
				g_u8FakePointCnt ++;
			}
		}
	}
}

uint8_t ScanMarixToFilterFakePoint(uint8_t area)
{
	uint8_t 	i,maxloop;
	uint8_t 	miniPoint;

	//g_u16TotalDotCnt = 0;
	//g_u16ShiftDotCnt = 0;
	//g_u16GetDotFitCnt = 0;
	
	g_u8CurArea = area;
	g_bOneLineFake = FALSE;

	g_u8XCrossCnt = 0;
	g_u8YCrossCnt = 0;
	g_u8XR_CrossCnt = 0;
	g_u8XL_CrossCnt = 0;
	g_u8YR_CrossCnt = 0;
	g_u8YL_CrossCnt = 0;
	g_u16FakePatternCnt = 0;
	g_u8TruePointCnt = 0;
	g_u8FakePointCnt = 0;
	g_bFakeCheckOk = FALSE;
	g_bQuickFakeCheck = FALSE;
	g_u16AllFakeDotCnt = 0;

	g_u8MarixSize = g_u8XFakePointCnt * g_u8YFakePointCnt;
	g_u8MaxTrue = (g_u8XFakePointCnt > g_u8YFakePointCnt) ? g_u8XFakePointCnt : g_u8YFakePointCnt;

	if(g_bOneFakeArea == FALSE)
	{
		switch(g_u8MarixSize)
		{
			case 1:
			case 2:	
			case 3:
				maxloop = 3;
				break;
				
			case 4:
			default:	
				maxloop = 5;
				break;
		}

		if(g_u8ScreenAreaNmb == 2)// 1 * 2
		{
			miniPoint = 1;
			g_u8MaxFake = g_u8MarixSize - miniPoint;
		}
		else
		{
			miniPoint = 0;
			g_u8MaxFake = g_u8MarixSize;
		}
	}
	else
	{
		switch(g_u8MarixSize)
		{
			case 1:
			case 2:
			case 3:
				maxloop = 2;
				break;
			case 4:
				maxloop = 3;
				break;

			default:	
				maxloop = 5;
				break;
		}
		
		miniPoint = g_u8MaxTrue;
		g_u8MaxFake = g_u8MarixSize - miniPoint;
	}

	if((g_u8XTotalAreaCnt == 1 && g_u8XFakePointCnt == 1) || (g_u8YTotalAreaCnt == 1 && g_u8YFakePointCnt == 1))//only 1 line point
	{
		g_bOneLineFake = TRUE;
	}

	BuildFakePointPattern();

	for(i = 0; i < maxloop; i ++)
	{
		LeanScanMarixIncElement(X_DIRECTION, XR_LEAN);

		if(g_bFakeCheckOk == TRUE)
			break;

		LeanScanMarixDecElement(X_DIRECTION, XL_LEAN);
		
		if(g_bFakeCheckOk == TRUE)
			break;

		LeanScanMarixDecElement(Y_DIRECTION, YR_LEAN);

		if(g_bFakeCheckOk == TRUE)
			break;
		
		LeanScanMarixIncElement(Y_DIRECTION, YL_LEAN);

		if(g_bFakeCheckOk == TRUE)
			break;
	}
	
	LeanScanWaitQuickSamplePointOver();
	
	//DEBF("\r\n area:%d, loop:%d, Marix: %d*%d = %d, MaxFake:%d, get fake:%d. \n",area, i, g_u8XFakePointCnt, g_u8YFakePointCnt, g_u8MarixSize, g_u8MaxFake, g_u8FakePointCnt);

 	//DEBF("\r\n TotalDotCnt:%d, ShiftDotCnt:%d, GetFitCnt:%d \n",g_u16TotalDotCnt,g_u16ShiftDotCnt,g_u16GetDotFitCnt);

	CountPointAdValue();

	if(g_bOneFakeArea == TRUE)
	{	
		OrderPointTrueCntAndAdValueScore();
		//DEBF("\r\n filter step 1, g_u8FakePointCnt:%d, g_u8MaxFake:%d \n",g_u8FakePointCnt,g_u8MaxFake);
		UsePositionFilterPoint();

		if(g_u8FakePointCnt < g_u8MaxFake)
		{
			//DEBF("\r\n filter step 2, g_u8FakePointCnt:%d, g_u8MaxFake:%d \n",g_u8FakePointCnt,g_u8MaxFake);
			UseScoreFilterPoint();
		}
	}
	
	GetTruePointInPattern(g_u8CurArea);
	
	for(i = 0; i < g_u8MarixSize; i ++)
	{
		//DEBF("\r\n fake(P%d) :  TrueCnt: %d -> %d, (x:%d, y:%d), w(%d, %d) \n", i, g_stAllFakePoint[g_u8CurArea][i].x_width*g_stAllFakePoint[g_u8CurArea][i].y_width, g_stAllFakePoint[g_u8CurArea][i].trueCnt, 
		//	g_stAllFakePoint[g_u8CurArea][i].x_start + (g_stAllFakePoint[g_u8CurArea][i].x_width >> 1), g_stAllFakePoint[g_u8CurArea][i].y_start + (g_stAllFakePoint[g_u8CurArea][i].y_width >> 1),
		//	g_stAllFakePoint[g_u8CurArea][i].x_width, g_stAllFakePoint[g_u8CurArea][i].y_width);
		
		//DEBF("\r\n fake(P%d) :  TrueCnt: %d -> %d,  ad:%d \n", i, g_stAllFakePoint[g_u8CurArea][i].x_width*g_stAllFakePoint[g_u8CurArea][i].y_width, 
		//g_stAllFakePoint[g_u8CurArea][i].trueCnt, g_stAllFakePoint[g_u8CurArea][i].ad_value);
	}
	
	//DEBF("\r\n g_u8CurTouchPointCnt:%d \n",g_u8CurTouchPointCnt);
	
	return g_u8TruePointCnt;
}

bool Trace_CheckGroupValid(const TraceLink_t *ptrLink)
{
	#ifdef USE_TRACE_CROSS_CHECK_FUNCTION
	uint8_t i,j;
	uint16_t x1,y1,x2,y2,x3,y3,x4,y4;
	const TraceLink_t *ptr;
	
	for(i = 0; i < g_u8TraceCnt - 1; i ++)
	{
		ptr = ptrLink + i;
		
		x1 = g_stLastTouchPoint[g_stTraceNode[ptr->p].PNode_ID].x_position;
		y1 = g_stLastTouchPoint[g_stTraceNode[ptr->p].PNode_ID].y_position;
		
		x2 = g_stCurTouchPoint[g_stTraceNode[ptr->p].link[ptr->s].SNode_ID].x_position;
		y2 = g_stCurTouchPoint[g_stTraceNode[ptr->p].link[ptr->s].SNode_ID].y_position;

		for(j = i + 1; j < g_u8TraceCnt; j ++)
		{
			ptr = ptrLink + j;
			
			x3 = g_stLastTouchPoint[g_stTraceNode[ptr->p].PNode_ID].x_position;
			y3 = g_stLastTouchPoint[g_stTraceNode[ptr->p].PNode_ID].y_position;
			
			x4 = g_stCurTouchPoint[g_stTraceNode[ptr->p].link[ptr->s].SNode_ID].x_position;
			y4 = g_stCurTouchPoint[g_stTraceNode[ptr->p].link[ptr->s].SNode_ID].y_position;

			if(IsLineSegmentCross(x1,y1,x2,y2,x3,y3,x4,y4) == TRUE)
			{
				return FALSE;
			}
		}
	}
	#endif

	return TRUE;
}

void Trace_ChooseBestLinkGroup(void)
{
	uint16_t	groupId,tmpData;
	uint8_t 	i,j,k,orderSumCnt,isValid;	
	uint8_t		noCrossflag[CACHE_TRACE_GROUP_SIZE];
	uint16_t	orderSum[CACHE_TRACE_GROUP_SIZE],orderSumId[CACHE_TRACE_GROUP_SIZE];
	const TraceLink_t *ptrLink;

	groupId = 0xffff;
	orderSumCnt = 0;

	#if 0
	//DEBF("\r\n	Sum:  \n");
	for(i = 0; i < g_u16TraceGroupNmb; i ++)
	{
		DEBF("\r\n	group:%d, Sum:%d   \n", g_stLinkDsumSortGroup[i].Groupid, g_stLinkDsumSortGroup[i].Dsum);
	}
	
	//DEBF("\r\n	Dis:  \n");
	for(i = 0; i < g_u16TraceGroupNmb; i ++)
	{
		DEBF("\r\n	group:%d, Dis:%d   \n", g_stLinkDdisSortGroup[i].Groupid, g_stLinkDdisSortGroup[i].Ddis);
	}
	#endif
		
	for(i = 0; i < g_u16TraceGroupNmb; i ++)
	{
		for(j = 0; j < g_u16TraceGroupNmb; j ++)
		{
			if(g_stLinkDsumSortGroup[i].Groupid == g_stLinkDdisSortGroup[j].Groupid)
			{
				orderSum[orderSumCnt] = i + j;
				orderSumId[orderSumCnt] = g_stLinkDsumSortGroup[i].Groupid;
				orderSumCnt ++;
			}
		}
	}
	
	//DEBF("\r\n ChooseBestLink : g_u16TraceGroupNmb:%d, orderSum Cnt:%d  \n",g_u16TraceGroupNmb, orderSumCnt);
			
	if(orderSumCnt == 0)
	{
		for(i = 0; i < g_u16TraceGroupNmb; i ++)
		{
			ptrLink = g_pLinkArrayPtr + (g_stLinkDsumSortGroup[i].Groupid * g_u8TraceCnt);
			isValid = Trace_CheckGroupValid(ptrLink);
			//DEBF("\r\n check group%d, no cross, valid:%d  \n",orderSumId[i],isValid);
			if(isValid == 1)
			{
				groupId = g_stLinkDsumSortGroup[i].Groupid;
				DEBF("\r\n Can't get best, use mini no cross sum group:%d as best. \n",groupId);
				break;
			}
		}

		if(groupId == 0xffff)
		{
			groupId = g_stLinkDsumSortGroup[0].Groupid;
			DEBF("\r\n Can't get best, all group cross, choose mini sum group:%d as best. \n",groupId);
		}
	}
	else
	{
		for(i = 0; i < orderSumCnt; i ++)
		{
			ptrLink = g_pLinkArrayPtr + (orderSumId[i] * g_u8TraceCnt);
			isValid = Trace_CheckGroupValid(ptrLink);
			//DEBF("\r\n check group%d, no cross, valid:%d  \n",orderSumId[i],isValid);
			noCrossflag[i] = isValid;
		}

		//order  small -> bigger
		for(i = 0; i < orderSumCnt - 1; i ++)
		{
			k = i;
			
			for(j = i + 1; j < orderSumCnt; j ++)
			{
				if(orderSum[k] > orderSum[j])
					k = j;
			}
	
			if(k != i)
			{
				tmpData = orderSum[i];
				orderSum[i] = orderSum[k];
				orderSum[k] = tmpData;
				
				tmpData = noCrossflag[i];
				noCrossflag[i] = noCrossflag[k];
				noCrossflag[k] = tmpData;
				
				tmpData = orderSumId[i];
				orderSumId[i] = orderSumId[k];
				orderSumId[k] = tmpData;
			}
		}

		
		for(i = 0; i < orderSumCnt; i ++)
		{			
			if(noCrossflag[i] == 1)
			{
				groupId = orderSumId[i];
				//DEBF("\r\n choose group:%d, Sum order:%d \n",groupId, i);
				break;
			}
		}

		if(groupId == 0xffff)
		{
			groupId = orderSumId[0];
			DEBF("\r\n TraceGroupNmb:%d, all group cross, choose group:%d (Sum order:0) \n",g_u16TraceGroupNmb,groupId);
		}
	}

	if(groupId != 0xffff)
	{
		ptrLink = g_pLinkArrayPtr + (groupId * g_u8TraceCnt);

		for(i = 0; i < g_u8TraceCnt; i ++)
		{
			g_stTraceNode[ptrLink->p].P_Link_Flag = LINKED;
			g_stTraceNode[ptrLink->p].link[ptrLink->s].S_Link_Flag = LINKED;

			//DEBF("\r\n choose : get trace:%d, (P%d - S%d), dis:%d \n", i, ptrLink->p, ptrLink->s, g_stTraceNode[ptrLink->p].link[ptrLink->s].Distance);

			ptrLink ++;
		}
	}
	else
	{
		DEBF("\r\n can't choose group !! \n");
	}
}

void Trace_SelectBestPointLink(void)
{
	uint8_t i;
  //uint8_t getBest;
	
	if(g_u16TraceGroupNmb > 0 && g_bOneFakeArea == TRUE && g_u8CurTouchPointCnt == g_u8LastTouchPointCnt)
	{
		Trace_ChooseBestLinkGroup();
	}
	else
	{
		//getBest = 0;
		DEBF("\r\n try get best link !! \n");
		for(i = 0; i < g_u8PreLinkCnt; i ++)
		{
			if(g_stPreLink[i].link == PRE_LINKED)
			{
				//getBest = 1;
				g_stTraceNode[g_stPreLink[i].p].P_Link_Flag = LINKED;
				g_stTraceNode[g_stPreLink[i].p].link[g_stPreLink[i].s].S_Link_Flag = LINKED;
				
				DEBF("\r\n	Select link : get trace:%d, (P%d - S%d), dis:%d \n", i, g_stPreLink[i].p, g_stPreLink[i].s, g_stTraceNode[g_stPreLink[i].p].link[g_stPreLink[i].s].Distance);
			}
		}	
	}
}

void _SetPreLinkStatus(void)
{
	uint8_t i,maxLoop,p,s;

	g_s8ReChkPreLinkCnt = 0;
	g_s8ReChkPreUnlinkCnt = 0;
	maxLoop = g_u8PreLinkCnt > g_u8PreUnlinkCnt ? g_u8PreLinkCnt : g_u8PreUnlinkCnt;

#if 0
	if(g_u8TraceDire == 0)
		DEBF("\r\nParent: Last. ");
	else
		DEBF("\r\nParent: Current. ");
#endif

	for(i = 0; i < maxLoop; i ++)
	{
		if(g_stPreLink[i].link == PRE_LINKED)
		{
			p = g_stPreLink[i].p;
			s = g_stPreLink[i].s;
			g_stTraceNode[p].P_Link_Flag = PRE_LINKED;
			g_stTraceNode[p].link[s].S_Link_Flag = PRE_LINKED;
			
			//DEBF("\r\n  Pre-Link:%d, P:%d - S:%d, dis:%d ", g_s8ReChkPreLinkCnt, p, s, g_stTraceNode[p].link[s].Distance);
			g_s8ReChkPreLinkCnt ++;
		}

		if(g_stPreUnlink[i].link == PRE_UNLINKED)
		{
			p = g_stPreUnlink[i].p;
			s = g_stPreUnlink[i].s;
			g_stTraceNode[p].link[s].S_Link_Flag = PRE_UNLINKED;
			
			//DEBF("\r\n  Pre-UnLink:%d, P:%d - S:%d, dis:%d ", g_s8ReChkPreUnlinkCnt, p, s, g_stTraceNode[p].link[s].Distance);
			g_s8ReChkPreUnlinkCnt ++;
		}
	}
}

void _ReCheckPreLink(void)
{
	uint8_t i,j,k,PunlinkCnt,oldP,newP;

	for(i = 0; i < g_u8PreLinkCnt; i ++)
	{
		if(g_stPreLink[i].link != PRE_LINKED)
			continue;
		
		k = i;
		for(j = 0; j < g_u8PreLinkCnt; j ++)
		{
			if(g_stPreLink[j].link != PRE_LINKED || j == i)
				continue;
			
			if(g_stPreLink[k].p == g_stPreLink[j].p || g_stPreLink[k].s == g_stPreLink[j].s)
			{
				//DEBF("\r\n Pre-Link much! try to cut some!! \n");
				
				if(g_stPreLink[j].ds == g_stPreLink[k].ds)
				{
					g_stPreLink[k].link = UNLINK;
					g_stPreLink[j].link = UNLINK;
					//DEBF("\r\n cut:(P:%d - S:%d),(P:%d - S:%d) ds:%d \n",g_stPreLink[k].p,g_stPreLink[k].s,g_stPreLink[j].p,g_stPreLink[j].s,g_stPreLink[j].ds);
				}
				else if(g_stPreLink[j].ds > g_stPreLink[k].ds)
				{
					g_stPreLink[j].link = UNLINK;
					//DEBF("\r\n cut:(P:%d - S:%d) ds:%d, keep:(P:%d - S:%d) ds:%d \n",g_stPreLink[j].p,g_stPreLink[j].s,g_stPreLink[j].ds,g_stPreLink[k].p,g_stPreLink[k].s,g_stPreLink[k].ds);
				}
				else
				{
					g_stPreLink[k].link = UNLINK;
					//DEBF("\r\n cut:(P:%d - S:%d) ds:%d, keep:(P:%d - S:%d) ds:%d \n",g_stPreLink[k].p,g_stPreLink[k].s,g_stPreLink[k].ds,g_stPreLink[j].p,g_stPreLink[j].s,g_stPreLink[j].ds);
					k = j;
				}
			}
		}
	}

	#if 0 /*zhaorui 130723 +*/ 
	for(i = 0; i < g_u8PreUnlinkCnt; i ++)
	{
		if(g_stPreUnlink[i].link != PRE_UNLINKED)
			continue;
		
		PunlinkCnt = 0;

		for(j = 0; j < g_u8PreUnlinkCnt; j ++)
		{
			if(g_stPreUnlink[j].link != PRE_UNLINKED)
				continue;
			
			if(g_stPreUnlink[i].p == g_stPreUnlink[j].p)
			{
				PunlinkCnt ++;
			}
		}
		
		if(PunlinkCnt >= g_u8TraceCnt)
		{
			DEBF("\r\n Pre-UnLink much! try to cut some!! \n");

			k = i;
			
			for(j = 0; j < g_u8PreUnlinkCnt; j ++)
			{
				if(g_stPreUnlink[j].link != PRE_UNLINKED || j == i)
					continue;
				
				if(g_stPreUnlink[k].p == g_stPreUnlink[j].p)
				{
					if(g_stPreUnlink[j].ds == g_stPreUnlink[k].ds)
					{
						if(g_stPreUnlink[k].ds < g_u8UnLinkDs)
						{
							PunlinkCnt --;
							
							g_stPreUnlink[k].link = UNLINK;
							g_stPreUnlink[j].link = UNLINK;
							DEBF("\r\n cut:(P:%d - S:%d),(P:%d - S:%d) ds:%d \n",g_stPreUnlink[k].p,g_stPreUnlink[k].s,g_stPreUnlink[j].p,g_stPreUnlink[j].s,g_stPreUnlink[j].ds);
						}
					}
					else if(g_stPreUnlink[j].ds < g_stPreUnlink[k].ds)
					{
						if(g_stPreUnlink[j].ds < g_u8UnLinkDs)
						{
							PunlinkCnt --;
							
							g_stPreUnlink[j].link = UNLINK;
							DEBF("\r\n cut:(P:%d - S:%d) ds:%d, keep:(P:%d - S:%d) ds:%d \n",g_stPreUnlink[j].p,g_stPreUnlink[j].s,g_stPreUnlink[j].ds,g_stPreUnlink[k].p,g_stPreUnlink[k].s,g_stPreUnlink[k].ds);
						}
					}
					else
					{
						if(g_stPreUnlink[k].ds < g_u8UnLinkDs)
						{
							PunlinkCnt --;
							
							g_stPreUnlink[k].link = UNLINK;
							DEBF("\r\n cut:(P:%d - S:%d) ds:%d, keep:(P:%d - S:%d) ds:%d \n",g_stPreUnlink[k].p,g_stPreUnlink[k].s,g_stPreUnlink[k].ds,g_stPreUnlink[j].p,g_stPreUnlink[j].s,g_stPreUnlink[j].ds);
							k = j;
						}
					}
				}
			}			
		}
	}
	#endif

	#if 1  /*zhaorui 130723 +*/ 
	for(i = 0; i < g_u8PreUnlinkCnt; i ++)
	{
		if(g_stPreUnlink[i].link != PRE_UNLINKED)
			continue;
		
		if(g_u8TraceDire == OLD_POINT_IS_PARENT)
		{
			oldP = g_stPreUnlink[i].p;
			newP = g_stPreUnlink[i].s;
		}
		else
		{
			oldP = g_stPreUnlink[i].s;
			newP = g_stPreUnlink[i].p;
		}

		if(g_stCurTouchPoint[newP].attri != NEW)
		{
			PunlinkCnt = 0;
			
			for(j = 0; j < g_u8PreUnlinkCnt; j ++)
			{
				if(g_stPreUnlink[j].link != PRE_UNLINKED)
					continue;

				if(g_u8TraceDire == OLD_POINT_IS_PARENT)
				{
					if(newP == g_stPreUnlink[j].s)
						PunlinkCnt ++;
				}
				else
				{
					if(newP == g_stPreUnlink[j].p)
						PunlinkCnt ++;
				}
			}

			if(PunlinkCnt >= g_u8LastTouchPointCnt)
			{
				//DEBF("\r\n _ReCheckPreLink current point:%d, Pre-UnLink much! mean new get!! \n", newP);
				g_stCurTouchPoint[newP].attri = NEW;
			}
		}


		if(g_stLastTouchPoint[oldP].attri != REMOVE)
		{
			PunlinkCnt = 0;
					
			for(j = 0; j < g_u8PreUnlinkCnt; j ++)
			{
				if(g_stPreUnlink[j].link != PRE_UNLINKED)
					continue;

				if(g_u8TraceDire == OLD_POINT_IS_PARENT)
				{
					if(oldP == g_stPreUnlink[j].p)
						PunlinkCnt ++;
				}
				else
				{
					if(oldP == g_stPreUnlink[j].s)
						PunlinkCnt ++;
				}
			}

			if(PunlinkCnt >= g_u8CurTouchPointCnt)
			{
				//DEBF("\r\n _ReCheckPreLink last point:%d, Pre-UnLink much! mean remove!! \n", oldP);
				g_stLastTouchPoint[oldP].attri = REMOVE;
			}
		}
	}
	#endif  /*  #if 0 -*/
}

void _CheckPreLink(uint8_t p, uint8_t s, uint32_t ds)
{
	if(ds <= PRE_LINK_DS)
	{		
		g_stPreLink[g_u8PreLinkCnt].link = PRE_LINKED;
		g_stPreLink[g_u8PreLinkCnt].p = p;
		g_stPreLink[g_u8PreLinkCnt].s = s;
		g_stPreLink[g_u8PreLinkCnt].ds = ds;
		
		g_u8PreLinkCnt ++;
	}
	else if(ds >= g_u8PreUnLinkDs)
	{
		g_stPreUnlink[g_u8PreUnlinkCnt].link = PRE_UNLINKED;
		g_stPreUnlink[g_u8PreUnlinkCnt].p = p;
		g_stPreUnlink[g_u8PreUnlinkCnt].s = s;
		g_stPreUnlink[g_u8PreUnlinkCnt].ds = ds;
		
		g_u8PreUnlinkCnt ++;
	}
}

void Trace_StoreSmallLinkDis(uint16_t groupId, uint32_t dis, uint32_t sum)
{
	int i,k,j;
	TraceDispersion_t tmpStDdis;
	TraceDistanceSum_t tmpStDsum;
	
	if(g_u16TraceGroupNmb > 0)
	{
		k = g_u16TraceGroupNmb;
		j = g_u16TraceGroupNmb;
		
		for(i = g_u16TraceGroupNmb - 1; i >= 0; i --)
		{
			if(g_stLinkDdisSortGroup[i].Ddis > dis)
				k = i;

			if(g_stLinkDsumSortGroup[i].Dsum > sum)
				j = i;
		}

		if(k < g_u16TraceGroupNmb)
		{
			tmpStDdis = g_stLinkDdisSortGroup[g_u16TraceGroupNmb - 1];
			
			for(i = g_u16TraceGroupNmb - 1; i >= k + 1; i --)
			{
				g_stLinkDdisSortGroup[i] = g_stLinkDdisSortGroup[i-1];
			}

			g_stLinkDdisSortGroup[k].Ddis = dis;
			g_stLinkDdisSortGroup[k].Groupid = groupId;

			if(g_u16TraceGroupNmb < CACHE_TRACE_GROUP_SIZE)
			{
				g_stLinkDdisSortGroup[g_u16TraceGroupNmb] = tmpStDdis;
			}
		}
		else
		{
			if(g_u16TraceGroupNmb < CACHE_TRACE_GROUP_SIZE)
			{
				g_stLinkDdisSortGroup[g_u16TraceGroupNmb].Ddis = dis;
				g_stLinkDdisSortGroup[g_u16TraceGroupNmb].Groupid = groupId;
			}
		}
		
		if(j < g_u16TraceGroupNmb)
		{
			tmpStDsum = g_stLinkDsumSortGroup[g_u16TraceGroupNmb - 1];
			
			for(i = g_u16TraceGroupNmb - 1; i >= j + 1; i --)
			{
				g_stLinkDsumSortGroup[i] = g_stLinkDsumSortGroup[i-1];
			}

			g_stLinkDsumSortGroup[j].Dsum = sum;
			g_stLinkDsumSortGroup[j].Groupid = groupId;

			if(g_u16TraceGroupNmb < CACHE_TRACE_GROUP_SIZE)
			{
				g_stLinkDsumSortGroup[g_u16TraceGroupNmb] = tmpStDsum;
			}
		}
		else
		{
			if(g_u16TraceGroupNmb < CACHE_TRACE_GROUP_SIZE)
			{
				g_stLinkDsumSortGroup[g_u16TraceGroupNmb].Dsum = sum;
				g_stLinkDsumSortGroup[g_u16TraceGroupNmb].Groupid = groupId;
			}
		}
	}
	else
	{
		g_stLinkDdisSortGroup[g_u16TraceGroupNmb].Groupid = groupId;
		g_stLinkDdisSortGroup[g_u16TraceGroupNmb].Ddis = dis;
		
		g_stLinkDsumSortGroup[g_u16TraceGroupNmb].Groupid = groupId;
		g_stLinkDsumSortGroup[g_u16TraceGroupNmb].Dsum = sum;
	}

	if(g_u16TraceGroupNmb < CACHE_TRACE_GROUP_SIZE)
	{
		g_u16TraceGroupNmb ++;
	}
}

uint8_t Trace_OrderLinksDdisAndDsum(void)
{
	uint8_t		p,s,maxP,maxS;
	uint8_t 	preLinkCnt,preUnLinkCnt;
	uint16_t 	i,groupId;
	uint32_t	tmpDis,tmpSum;
	uint32_t	groupDisCache[POINT_BUFFER_SIZE];
	const TraceLink_t *pLinkptr;

	maxP = g_u8TraceCnt;
	maxS = g_u8SonPoint;

	DEBF("\r\n OrderLinks, maxP:%d, maxS:%d \n",maxP,maxS);

	if(maxP > MAX_POINT_CNT || maxS > USB_SEND_POINT_CNT)
		return 0xff;
	
	switch(maxP)
	{
		case 6:
			g_u16LinkmaxGroup = 720;
			g_pLinkArrayPtr = &g_stTraceGroupArray6x6[0][0];
			break;
		
		case 5:
			if(maxS == 5)
			{
				g_u16LinkmaxGroup = 120;
				g_pLinkArrayPtr = &g_stTraceGroupArray5x5[0][0];
			}
			else
			{
				g_u16LinkmaxGroup = 720;
				g_pLinkArrayPtr = &g_stTraceGroupArray5x6[0][0];
			}
			break;

		case 4:
			if(maxS == 4)
			{
				g_u16LinkmaxGroup = 24;
				g_pLinkArrayPtr = &g_stTraceGroupArray4x4[0][0];
			}
			else if(maxS == 5)
			{
				g_u16LinkmaxGroup = 120;
				g_pLinkArrayPtr = &g_stTraceGroupArray4x5[0][0];
			}
			else
			{
				g_u16LinkmaxGroup = 360;
				g_pLinkArrayPtr = &g_stTraceGroupArray4x6[0][0];
			}
			break;

		case 3:
			if(maxS == 3)
			{
				g_u16LinkmaxGroup = 6;
				g_pLinkArrayPtr = &g_stTraceGroupArray3x3[0][0];
			}
			else if(maxS == 4)
			{
				g_u16LinkmaxGroup = 24;
				g_pLinkArrayPtr = &g_stTraceGroupArray3x4[0][0];
			}
			else if(maxS == 5)
			{
				g_u16LinkmaxGroup = 60;
				g_pLinkArrayPtr = &g_stTraceGroupArray3x5[0][0];
			}
			else
			{
				g_u16LinkmaxGroup = 120;
				g_pLinkArrayPtr = &g_stTraceGroupArray3x6[0][0];
			}
			break;

		case 2:
			if(maxS == 2)
			{
				g_u16LinkmaxGroup = 2;
				g_pLinkArrayPtr = &g_stTraceGroupArray2x2[0][0];
			}
			else if(maxS == 3)
			{
				g_u16LinkmaxGroup = 6;
				g_pLinkArrayPtr = &g_stTraceGroupArray2x3[0][0];
			}
			else if(maxS == 4)
			{
				g_u16LinkmaxGroup = 12;
				g_pLinkArrayPtr = &g_stTraceGroupArray2x4[0][0];
			}
			else if(maxS == 5)
			{
				g_u16LinkmaxGroup = 20;
				g_pLinkArrayPtr = &g_stTraceGroupArray2x5[0][0];
			}
			else
			{
				g_u16LinkmaxGroup = 30;
				g_pLinkArrayPtr = &g_stTraceGroupArray2x6[0][0];
			}
			break;
		
		case 1:
			g_u16LinkmaxGroup = maxS;
			g_pLinkArrayPtr = &g_stTraceGroupArray1x6[0][0];
			break;

		default:
			g_u16LinkmaxGroup = maxS;
			g_pLinkArrayPtr = &g_stTraceGroupArray1x6[0][0];
			break;
	}

	for(groupId = 0; groupId < g_u16LinkmaxGroup; groupId ++)
	{
		pLinkptr = g_pLinkArrayPtr + (maxP*groupId);

		p = 0;
		s = 0;
		tmpDis = 0;
		tmpSum = 0;
		preLinkCnt = 0;
		preUnLinkCnt = 0;
		
		for(i = 0; i < maxP; i ++)
		{
			p = pLinkptr->p;
			s = pLinkptr->s;
			
			if(g_stTraceNode[p].link[s].S_Link_Flag == PRE_UNLINKED)
			{
				preUnLinkCnt ++;
				break;
			}
			
			if(g_stTraceNode[p].link[s].S_Link_Flag == PRE_LINKED)
				preLinkCnt ++;

			groupDisCache[i] = g_stTraceNode[p].link[s].Distance;

			tmpSum += groupDisCache[i];
			pLinkptr ++;
		}

		if(preLinkCnt == g_s8ReChkPreLinkCnt && preUnLinkCnt == 0)
		{
#if 0
			DEBF("\r\n Trace:%d ",groupId);
			pLinkptr = g_pLinkArrayPtr + (maxP*groupId);
			for(i = 0; i < maxP; i ++)
			{
				p = pLinkptr->p;
				s = pLinkptr->s;
				pLinkptr ++;
				
				DEBF("{%d,%d}, ", p, s);
			}
#endif
			tmpDis = _CalOneGroupTraceDispersion(&groupDisCache[0], maxP);

			Trace_StoreSmallLinkDis(groupId, tmpDis, tmpSum);
		}
		else
		{
			//DEBF("\r\n Trace:%d, skip!!  linkCnt:%d, unLinkCnt:%d \n",groupId, preLinkCnt, preUnLinkCnt);
		}
	}

	return 0;
}

uint32_t _GetPointDs(uint8_t old, uint8_t cur)
{
	uint8_t i,j;

	for(i = 0; i < POINT_BUFFER_SIZE; i ++)
	{
		if(g_stTraceDS[i].PNode_ID == old)
		{
			for(j = 0; j < g_u8CurTouchPointCnt; j ++)
			{
				if(g_stTraceDS[i].link[j].SNode_ID == cur)
					return g_stTraceDS[i].link[j].Distance;
			}
		}
	}

	ERRF("\r\n oldP:%d - curP:%d, no save this trace distance !! ", old, cur);
	
	return 0xffff;
}

void Trace_CalPointLinkDis(void)
{
	uint8_t i,j,p,s;

	//DEBF("\r\n Trace_CalPointLinkDis, LastTracePointCnt:%d, CurTracePointCnt:%d ", g_u8LastTracePointCnt, g_u8CurTracePointCnt);
	
	p = 0;
	if (g_u8LastTracePointCnt <= g_u8CurTracePointCnt)
	{
		g_u8TraceDire = OLD_POINT_IS_PARENT;
		g_u8TraceCnt = g_u8LastTracePointCnt;
		g_u8SonPoint = g_u8CurTracePointCnt;
			
		for(i = 0; i < POINT_BUFFER_SIZE; i ++)
		{
			if(g_stLastTouchPoint[i].status.u8Value != INVALID_STATUS && g_stLastTouchPoint[i].attri == NORMAL)
			{
				g_stTraceNode[p].PNode_ID = i;
				g_stTraceNode[p].P_Link_Flag = UNLINK;

				s = 0;
				for(j = 0; j < g_u8CurTouchPointCnt; j ++)
				{
					if(g_stCurTouchPoint[j].attri == NORMAL)
					{
						g_stTraceNode[p].link[s].SNode_ID = j;
						g_stTraceNode[p].link[s].S_Link_Flag = UNLINK;
						g_stTraceNode[p].link[s].Distance = _GetPointDs(i, j);
						_CheckPreLink(p, s, g_stTraceNode[p].link[s].Distance);

						s ++;
					}
				}
				p ++;
			}
		}
	}
	else
	{
		g_u8TraceDire = NEW_POINT_IS_PARENT;
		g_u8TraceCnt = g_u8CurTracePointCnt;
		g_u8SonPoint = g_u8LastTracePointCnt;	

		for(j = 0; j < g_u8CurTouchPointCnt; j ++)
		{
			if(g_stCurTouchPoint[j].attri == NORMAL)
			{
				g_stTraceNode[p].PNode_ID = j;
				g_stTraceNode[p].P_Link_Flag = UNLINK;

				s = 0;
				for(i = 0; i < POINT_BUFFER_SIZE; i ++)
				{
					if(g_stLastTouchPoint[i].status.u8Value != INVALID_STATUS && g_stLastTouchPoint[i].attri == NORMAL)
					{
						g_stTraceNode[p].link[s].SNode_ID = i;
						g_stTraceNode[p].link[s].S_Link_Flag = UNLINK;
						g_stTraceNode[p].link[s].Distance = _GetPointDs(i, j);
						_CheckPreLink(p, s, g_stTraceNode[p].link[s].Distance);
						s ++;
					}
				}
				p ++;
			}
		}
	}
	
	_ReCheckPreLink();
	_SetPreLinkStatus();
}

void Trace_CheckPointAttri(void)
{
	uint8_t i,j,p,s,k,OverPreLinkCnt;
	uint8_t oldlink[POINT_BUFFER_SIZE],curlink[POINT_BUFFER_SIZE],oldUnlink[POINT_BUFFER_SIZE],curUnlink[POINT_BUFFER_SIZE],OverChklink[POINT_BUFFER_SIZE];

	k = 0;
	OverPreLinkCnt = 0;
	g_u8CurTracePointCnt = 0;
	g_u8LastTracePointCnt = 0;

	for(i = 0; i < POINT_BUFFER_SIZE; i ++)
	{
		oldlink[i] = 0;
		curlink[i] = 0;
		oldUnlink[i] = 0;
		curUnlink[i] = 0;
		OverChklink[i] = 0;
	}

	p = 0;
	for(i = 0; i < POINT_BUFFER_SIZE; i ++)
	{
		if( (g_bLinkUp == TRUE && g_stLastTouchPoint[i].status.u8Value != INVALID_STATUS) || 
			(g_bLinkUp == FALSE && g_stLastTouchPoint[i].status.u8Value != INVALID_STATUS && g_stLastTouchPoint[i].status.u8Value != UP_STATUS))
		{
			g_stTraceDS[p].PNode_ID = i;
			g_stTraceDS[p].P_Link_Flag = UNKNOW_LINK;
			
			for(s = 0; s < g_u8CurTouchPointCnt; s ++)
			{
				g_stTraceDS[p].link[s].SNode_ID = s;
				g_stTraceDS[p].link[s].S_Link_Flag = UNKNOW_LINK;
				g_stTraceDS[p].link[s].Distance = CalPointDistance8Bit(g_stLastTouchPoint[i].x_position,g_stLastTouchPoint[i].y_position,g_stCurTouchPoint[s].x_position,g_stCurTouchPoint[s].y_position);
				//DEBF("\r\n old%d - cur%d  DS:%d ", i, s, g_stTraceDS[p].link[s].Distance);
				
				if(g_stTraceDS[p].link[s].Distance >= g_u8PreUnLinkDs)
				{
					oldUnlink[p] ++;
					curUnlink[s] ++;
					g_stTraceDS[p].link[s].S_Link_Flag = PRE_UNLINKED;
					//DEBF("\r\n  Pre-UnLink: P:%d - S:%d, dis:%d, oldUnlinkCnt:%d,curUnlinkCnt:%d ", p, s, g_stTraceDS[p].link[s].Distance,oldUnlink[p],curUnlink[s]);
				}
				else if(g_stTraceDS[p].link[s].Distance <= PRE_LINK_DS)
				{
					oldlink[p] ++;
					curlink[s] ++;
					g_stTraceDS[p].P_Link_Flag = PRE_LINKED;
					g_stTraceDS[p].link[s].S_Link_Flag = PRE_LINKED;
					//DEBF("\r\n  Pre-Link: P:%d - S:%d, dis:%d, oldlinkCnt:%d,curlinkCnt:%d ", p, s, g_stTraceDS[p].link[s].Distance,oldlink[p],curlink[s]);
				}
				
				if(g_stTraceDS[p].link[s].Distance <= OVER_CHECK_PRE_LINK_DS)
				{
					OverPreLinkCnt ++;
					OverChklink[s] ++;
					//DEBF("\r\n  Over Check Pre-Link: P:%d - S:%d, dis:%d, oldlinkCnt:%d,curlinkCnt:%d ", p, s, g_stTraceDS[p].link[s].Distance,oldlink[p],curlink[s]);
				}
					
			}
			p ++;
		}
		else
		{
			g_stLastTouchPoint[i].attri = REMOVE;
		}
	}

	for(i = 0; i < p; i ++)//check old
	{
		if(oldlink[i] > 0)
		{
			g_u8LastTracePointCnt ++;
			continue;
		}
		else if(oldUnlink[i] > 0 && oldUnlink[i] < g_u8CurTouchPointCnt)
		{
			for(s = 0; s < g_u8CurTouchPointCnt; s ++)
			{
				if(g_stTraceDS[i].link[s].S_Link_Flag != PRE_UNLINKED)
				{
					for(j = 0; j < p; j ++)
					{
						if(i != j && g_stTraceDS[j].link[s].S_Link_Flag == PRE_LINKED)
							oldUnlink[i] ++;
					}
				}
			}
		}

		if(oldUnlink[i] >= g_u8CurTouchPointCnt)
		{
			#if 0  /*zhaorui 130819 +*/ 
			if(g_bOneFakeArea == TRUE && g_u8CurTouchPointCnt == g_u8LastTouchPointCnt)
			{
				g_u8LastTracePointCnt ++;
			}
			else
			#endif  /*  #if 0 -*/
			{
				DEBF("\r\n last P:%d unLink, remove!! \n", g_stTraceDS[i].PNode_ID);
				g_stLastTouchPoint[g_stTraceDS[i].PNode_ID].attri = REMOVE;
			}
		}
		else
		{
			g_u8LastTracePointCnt ++;
		}
	}
	
	for(s = 0; s < g_u8CurTouchPointCnt; s ++)//check current
	{
		if(curlink[s] > 0)
		{
			g_u8CurTracePointCnt ++;
			continue;
		}
		else if(curUnlink[s] > 0 && curUnlink[s] < g_u8LastTouchPointCnt)
		{
			for(i = 0; i < p; i ++)
			{
				if(g_stTraceDS[i].link[s].S_Link_Flag != PRE_UNLINKED && g_stTraceDS[i].P_Link_Flag == PRE_LINKED)
				{
					curUnlink[s] ++;
				}
			}
		}

		if(curUnlink[s] >= g_u8LastTouchPointCnt)
		{
			#if 0  /*zhaorui 130820 +*/ 
			#if 0  /*zhaorui 130819 +*/ 
			if(g_bOneFakeArea == TRUE && g_u8CurTouchPointCnt == g_u8LastTouchPointCnt)
			{
				g_u8CurTracePointCnt ++;
			}
			else
			#endif  /*  #if 0 -*/
			{
				DEBF("\r\n cur P:%d unLink, new get!! \n", s);
				g_stCurTouchPoint[s].attri = NEW;
			}
			#endif  /*  #if 0 -*/

			if(g_u8CurTouchPointCnt - k > MAX_POINT_CNT)
			{
				DEBF("\r\n line:%d, ignore unLink cur P:%d, CurPointCnt:%d over max!! \n", __LINE__, s, g_u8CurTouchPointCnt);
				k ++;
				g_stCurTouchPoint[s].attri = IGNORE;
			}
			else
			{
				DEBF("\r\n cur P:%d unLink, new get!! \n", s);
				g_stCurTouchPoint[s].attri = NEW;
			}
		}
		else
		{
			g_u8CurTracePointCnt ++;
		}
	}

	if(g_u8CurTracePointCnt > MAX_POINT_CNT)
	{
		if(OverPreLinkCnt >= MAX_POINT_CNT)
		{
			for(i = 0; i < g_u8CurTouchPointCnt; i ++)
			{
				if(g_stCurTouchPoint[i].attri == NORMAL)
				{
					if(OverChklink[i] == 0)
					{
						DEBF("\r\n line:%d, ignore cur P:%d, CurPointCnt:%d over max!! \n", __LINE__, i, g_u8CurTouchPointCnt);
						g_stCurTouchPoint[i].attri = IGNORE;
						g_u8CurTracePointCnt --;
					}
				}
			}

			if(g_u8CurTracePointCnt > MAX_POINT_CNT)
			{
				j = 0;
				for(i = 0; i < g_u8CurTouchPointCnt; i ++)
				{
					if(g_stCurTouchPoint[i].attri == NORMAL)
					{
						j ++;
						
						if(j > MAX_POINT_CNT)
						{
							DEBF("\r\n line:%d, ignore cur P:%d, OverChklink[i] > 0, CurPointCnt:%d over max!! \n", __LINE__, i, g_u8CurTouchPointCnt);
							g_stCurTouchPoint[i].attri = IGNORE;
							g_u8CurTracePointCnt --;
						}
					}
				}
			}
		}
		else
		{
			j = 0;
			for(i = 0; i < g_u8CurTouchPointCnt; i ++)
			{
				if(g_stCurTouchPoint[i].attri == NORMAL)
				{
					j ++;
					
					if((j > MAX_POINT_CNT - OverPreLinkCnt) && (OverChklink[i] == 0))
					{
						DEBF("\r\n line:%d, ignore cur P:%d, CurPointCnt:%d over max!! \n", __LINE__, i, g_u8CurTouchPointCnt);
						g_stCurTouchPoint[i].attri = IGNORE;
						g_u8CurTracePointCnt --;
					}
				}
			}
		}
	}
}


void TouchBeginSetLastTouch(void)
{
	uint8_t i = 0;
	
	g_u8LastTouchPointCnt = g_u8CurTouchPointCnt;
	
	for(i = 0; i < g_u8CurTouchPointCnt; i ++)
	{
		g_stLastTouchPoint[i].status.u8Value = PRESS_STATUS;
		g_stLastTouchPoint[i].x_position = g_stCurTouchPoint[i].x_position;
		g_stLastTouchPoint[i].y_position = g_stCurTouchPoint[i].y_position;
	}
}

void TouchOneMoveUpdateLastTouch(void)
{
	uint8_t 	i,update,linkcnt,MiniLinkIndex;
	uint32_t	dis[POINT_BUFFER_SIZE],mini;

	update = 0;
	linkcnt = 0;
	
	for(i = 0; i < POINT_BUFFER_SIZE; i ++)
	{
		if(g_stLastTouchPoint[i].status.u8Value != INVALID_STATUS)
		{
			dis[i] = CalPointDistance8Bit(g_stLastTouchPoint[i].x_position,g_stLastTouchPoint[i].y_position,g_stCurTouchPoint[0].x_position,g_stCurTouchPoint[0].y_position);
		}
		else
		{
			dis[i] = 0x1fffffff;
		}
	}

	mini = dis[0];
	MiniLinkIndex = 0;
	for(i = 0; i < POINT_BUFFER_SIZE; i ++)
	{
		if(dis[i] < mini)
		{
			mini = dis[i];
			MiniLinkIndex = i;
		}
	}

	if(dis[MiniLinkIndex] <= ONE_AREA_PRE_UNLINK_DS)
	{
		update = 1;
		linkcnt ++;
		g_stLastTouchPoint[MiniLinkIndex].status.u8Value = RUNNING_STATUS;
		g_stLastTouchPoint[MiniLinkIndex].x_position = g_stCurTouchPoint[0].x_position;
		g_stLastTouchPoint[MiniLinkIndex].y_position = g_stCurTouchPoint[0].y_position;

		//DEBF("\r\n line:%d, Link: last P%d - P%d ", __LINE__, MiniLinkIndex, 0);
	}
	

	for(i = 0; i < POINT_BUFFER_SIZE; i ++)
	{
		if(update == 0 && g_stLastTouchPoint[i].status.u8Value == INVALID_STATUS)
		{
			update = 1;
			MiniLinkIndex = i;
			g_stLastTouchPoint[i].status.u8Value = PRESS_STATUS;
			g_stLastTouchPoint[i].x_position = g_stCurTouchPoint[0].x_position;
			g_stLastTouchPoint[i].y_position = g_stCurTouchPoint[0].y_position;
			//DEBF("\r\n line:%d, Link: last P%d - P%d, press ", __LINE__, i, 0);
		}
		else if(update == 1 && i == MiniLinkIndex)
		{
			continue;
		}
		else if(g_stLastTouchPoint[i].status.u8Value != INVALID_STATUS)
		{
			if(g_stLastTouchPoint[i].status.u8Value == UP_STATUS || g_stLastTouchPoint[i].status.u8Value == PRESS_STATUS)
			{
				g_stLastTouchPoint[i].status.u8Value = INVALID_STATUS;
				if(g_u8LastTouchPointCnt > linkcnt)
					g_u8LastTouchPointCnt --;

				g_stLastTouchPoint[i].x_position = 0;
				g_stLastTouchPoint[i].y_position = 0;
				//DEBF("\r\n line:%d, Last:%d invalid, g_u8LastTouchPointCnt dec :%d ", __LINE__,i,g_u8LastTouchPointCnt);
			}	
			else
			{
				g_stLastTouchPoint[i].status.u8Value = UP_STATUS;
				//DEBF("\r\n line:%d, Last:%d up ", __LINE__,i);
			}
		}
	}
}

void TouchMultiMoveUpdateLastTouch(void)
{
	uint8_t i,j,update,linkcnt;
	uint8_t p,s;
	uint8_t lastTouchLink[POINT_BUFFER_SIZE],curTouchLink[POINT_BUFFER_SIZE];

	linkcnt = 0;

	for(i = 0; i < POINT_BUFFER_SIZE; i ++)
	{
		lastTouchLink[i] = UNLINK;
		curTouchLink[i] = UNLINK;
	}

	p = 0;
	if(g_u8TraceDire == OLD_POINT_IS_PARENT)
	{
		for(i = 0; i < POINT_BUFFER_SIZE; i ++)// g_stLastTouchPoint is parent level
		{
			if(g_stLastTouchPoint[i].attri == NORMAL && g_stLastTouchPoint[i].status.u8Value != INVALID_STATUS)
			{
				if(g_stTraceNode[p].P_Link_Flag == LINKED)
				{					
					s = 0;
					for(j = 0; j < g_u8CurTouchPointCnt; j ++)
					{
						if(g_stCurTouchPoint[j].attri == NORMAL)
						{
							if(g_stTraceNode[p].link[s].S_Link_Flag == LINKED)
							{
								linkcnt ++;
								
								DEBF("\r\n line:%d, Link: last P%d - P%d ", __LINE__, g_stTraceNode[p].PNode_ID, g_stTraceNode[p].link[s].SNode_ID);
								
								g_stLastTouchPoint[g_stTraceNode[p].PNode_ID].status.u8Value = RUNNING_STATUS;
								g_stLastTouchPoint[g_stTraceNode[p].PNode_ID].x_position = g_stCurTouchPoint[g_stTraceNode[p].link[s].SNode_ID].x_position;
								g_stLastTouchPoint[g_stTraceNode[p].PNode_ID].y_position = g_stCurTouchPoint[g_stTraceNode[p].link[s].SNode_ID].y_position;
								
								lastTouchLink[g_stTraceNode[p].PNode_ID] = LINKED;
								curTouchLink[g_stTraceNode[p].link[s].SNode_ID] = LINKED;
								
								break;
							}
							s ++;
						}
					}
				}
				p ++;
			}
		}
	}
	else 
	{	
		for(j = 0; j < g_u8CurTouchPointCnt; j ++)// g_stCurTouchPoint is parent level
		{
			if(g_stCurTouchPoint[j].attri == NORMAL)
			{
				if(g_stTraceNode[p].P_Link_Flag == LINKED)
				{
					s = 0;
					for(i = 0; i < POINT_BUFFER_SIZE; i ++)
					{
						if(g_stLastTouchPoint[i].attri == NORMAL && g_stLastTouchPoint[i].status.u8Value != INVALID_STATUS)
						{
							if(g_stTraceNode[p].link[s].S_Link_Flag == LINKED)
							{
								linkcnt ++;
								
								DEBF("\r\n line:%d, Link: cur P%d - P%d ", __LINE__, g_stTraceNode[p].PNode_ID, g_stTraceNode[p].link[s].SNode_ID);
								
								g_stLastTouchPoint[g_stTraceNode[p].link[s].SNode_ID].status.u8Value = RUNNING_STATUS;
								g_stLastTouchPoint[g_stTraceNode[p].link[s].SNode_ID].x_position = g_stCurTouchPoint[g_stTraceNode[p].PNode_ID].x_position;
								g_stLastTouchPoint[g_stTraceNode[p].link[s].SNode_ID].y_position = g_stCurTouchPoint[g_stTraceNode[p].PNode_ID].y_position;
								
								lastTouchLink[g_stTraceNode[p].link[s].SNode_ID] = LINKED;
								curTouchLink[g_stTraceNode[p].PNode_ID] = LINKED;

								break;
							}
							s ++;
						}
					}
				}
				p ++;
			}
		}
	}

	//check del old point
	for(i = 0; i < POINT_BUFFER_SIZE; i ++)
	{
		if(lastTouchLink[i] == UNLINK && g_stLastTouchPoint[i].attri == REMOVE && g_stLastTouchPoint[i].status.u8Value != INVALID_STATUS)
		{
			if(g_stLastTouchPoint[i].status.u8Value == UP_STATUS || g_stLastTouchPoint[i].status.u8Value == PRESS_STATUS)
			{
				g_stLastTouchPoint[i].status.u8Value = INVALID_STATUS;
				if(g_u8LastTouchPointCnt > linkcnt)
					g_u8LastTouchPointCnt --;

				g_stLastTouchPoint[i].x_position = 0;
				g_stLastTouchPoint[i].y_position = 0;
				DEBF("\r\n line:%d, Last:%d invalid, g_u8LastTouchPointCnt dec :%d ", __LINE__,i,g_u8LastTouchPointCnt);
			}	
			else
			{
				g_stLastTouchPoint[i].status.u8Value = UP_STATUS;
				DEBF("\r\n line:%d, Last:%d up ", __LINE__,i);
			}
			lastTouchLink[i] = LINKED; // avoid new get point use this id
			//DEBF("\r\n 0 del unlink touch id:%d, set status to :%d", i, g_stLastTouchPoint[i].status.u8Value);
		}
		else if(lastTouchLink[i] == UNLINK && g_stLastTouchPoint[i].status.u8Value != INVALID_STATUS)
		{
			if(g_stLastTouchPoint[i].status.u8Value == UP_STATUS || g_stLastTouchPoint[i].status.u8Value == PRESS_STATUS)
			{
				g_stLastTouchPoint[i].status.u8Value = INVALID_STATUS;
				if(g_u8LastTouchPointCnt > linkcnt)
					g_u8LastTouchPointCnt --;
				
				g_stLastTouchPoint[i].x_position = 0;
				g_stLastTouchPoint[i].y_position = 0;
				DEBF("\r\n line:%d, Last:%d invalid, g_u8LastTouchPointCnt dec :%d ", __LINE__,i,g_u8LastTouchPointCnt);
			}	
			else
			{
				g_stLastTouchPoint[i].status.u8Value = UP_STATUS;
				DEBF("\r\n line:%d, Last:%d up ", __LINE__,i);
			}
			lastTouchLink[i] = LINKED; // avoid new get point use this id
			//DEBF("\r\n 1 del unlink touch id:%d, set status to :%d", i, g_stLastTouchPoint[i].status.u8Value);
		}
	}
		
	//check add new point
	for(j = 0; j < g_u8CurTouchPointCnt; j ++)
	{
		if(curTouchLink[j] == UNLINK && g_stCurTouchPoint[j].attri == NEW)
		{
			update = 0;
			for(i = 0; i < POINT_BUFFER_SIZE; i ++)
			{
				if(lastTouchLink[i] == UNLINK && g_stLastTouchPoint[i].status.u8Value == INVALID_STATUS)
				{
					update = 1;
					g_stLastTouchPoint[i].status.u8Value = PRESS_STATUS;
					g_stLastTouchPoint[i].x_position = g_stCurTouchPoint[j].x_position;
					g_stLastTouchPoint[i].y_position = g_stCurTouchPoint[j].y_position;
					
					DEBF("\r\n line:%d, add unlink point:P%d, as last: P%d, status:2 ",__LINE__,j,i);

					lastTouchLink[i] = LINKED;
					curTouchLink[j] = LINKED;
					if(g_u8LastTouchPointCnt < POINT_BUFFER_SIZE)
						g_u8LastTouchPointCnt ++;
					//DEBF("\r\n line:%d, cur:%d -> %d press, g_u8LastTouchPointCnt inc :%d ", __LINE__,j,i,g_u8LastTouchPointCnt);
					break;
				}
			}

			if(update == 0)
			{
				for(i = 0; i < POINT_BUFFER_SIZE; i ++)
				{
					if(lastTouchLink[i] == UNLINK && (g_stLastTouchPoint[i].status.u8Value == UP_STATUS || g_stLastTouchPoint[i].status.u8Value == PRESS_STATUS))
					{
						update = 1;
						g_stLastTouchPoint[i].status.u8Value = PRESS_STATUS;
						g_stLastTouchPoint[i].x_position = g_stCurTouchPoint[j].x_position;
						g_stLastTouchPoint[i].y_position = g_stCurTouchPoint[j].y_position;
						
						DEBF("\r\n line:%d, add unlink point:P%d, as last: P%d, status:2 ",__LINE__,j,i);

						lastTouchLink[i] = LINKED;
						curTouchLink[j] = LINKED;
						if(g_u8LastTouchPointCnt < POINT_BUFFER_SIZE)
							g_u8LastTouchPointCnt ++;
						//DEBF("\r\n line:%d, cur:%d -> %d press, g_u8LastTouchPointCnt inc :%d ", __LINE__,j,i,g_u8LastTouchPointCnt);
						break;
					}
				}
			}
		}
		else if(curTouchLink[j] == UNLINK && g_stCurTouchPoint[j].attri != IGNORE)
		{
			update = 0;
			
			for(i = 0; i < POINT_BUFFER_SIZE; i ++)
			{
				if(lastTouchLink[i] == UNLINK && g_stLastTouchPoint[i].status.u8Value == INVALID_STATUS)
				{
					update = 1;
					g_stLastTouchPoint[i].status.u8Value = PRESS_STATUS;
					g_stLastTouchPoint[i].x_position = g_stCurTouchPoint[j].x_position;
					g_stLastTouchPoint[i].y_position = g_stCurTouchPoint[j].y_position;
					
					DEBF("\r\n line:%d, add unlink point:P%d, as last: P%d, status:2 ",__LINE__,j,i);

					lastTouchLink[i] = LINKED;
					curTouchLink[j] = LINKED;

					if(g_u8LastTouchPointCnt < POINT_BUFFER_SIZE)
						g_u8LastTouchPointCnt ++;
					//DEBF("\r\n line:%d, cur:%d -> %d press, g_u8LastTouchPointCnt inc :%d ", __LINE__,j,i,g_u8LastTouchPointCnt);
					break;
				}
			}
			
			if(update == 0)
			{
				for(i = 0; i < POINT_BUFFER_SIZE; i ++)
				{
					if(lastTouchLink[i] == UNLINK && (g_stLastTouchPoint[i].status.u8Value == UP_STATUS || g_stLastTouchPoint[i].status.u8Value == PRESS_STATUS))
					{
						update = 1;
						g_stLastTouchPoint[i].status.u8Value = PRESS_STATUS;
						g_stLastTouchPoint[i].x_position = g_stCurTouchPoint[j].x_position;
						g_stLastTouchPoint[i].y_position = g_stCurTouchPoint[j].y_position;
						
						DEBF("\r\n line:%d, add unlink point:P%d, as last: P%d, status:2 ",__LINE__,j,i);

						lastTouchLink[i] = LINKED;
						curTouchLink[j] = LINKED;

						if(g_u8LastTouchPointCnt < POINT_BUFFER_SIZE)
							g_u8LastTouchPointCnt ++;
						
						//DEBF("\r\n line:%d, cur:%d -> %d press, g_u8LastTouchPointCnt inc :%d ", __LINE__,j,i,g_u8LastTouchPointCnt);
						break;
					}
				}
			}
		}
	}

	if(g_u8LastTouchPointCnt < linkcnt)
		g_u8LastTouchPointCnt = linkcnt;
}

void  TruePointMuchPickBest(void)
{
	uint16_t tmp;
	uint16_t i,j,m,n,orgiPointCnt,nowCnt;

	orgiPointCnt = g_u8CurTouchPointCnt;

	for(i = 0; i < g_u8CurTouchPointCnt; i ++)
	{
		g_u8TrueCntScore[i] = 0;
		g_u16AdScore[i] = 0;
	}

	for(i = 0; i < g_u8CurTouchPointCnt; i ++)
	{
		g_u8TrueCntScore[i] = g_stCurTouchPoint[i].trueCnt;
	}

	DEBF("\r\n CurTouchPointCnt:%d, Pick Best Point, step 1 \n", g_u8CurTouchPointCnt);

	//order true cnt score,  bigger -> small
	for(i = 0; i < g_u8CurTouchPointCnt - 1; i ++)
	{
		m = i;
		
		for(n = i + 1; n < g_u8CurTouchPointCnt; n ++)
		{
			if(g_u8TrueCntScore[m] < g_u8TrueCntScore[n])
				m = n;
		}

		if(m != i)
		{
			tmp = g_u8TrueCntScore[i];
			g_u8TrueCntScore[i] = g_u8TrueCntScore[m];
			g_u8TrueCntScore[m] = tmp;
		}
	}

	m = g_u8CurTouchPointCnt;
	
	for(i = 0; i < m; i ++)
	{
		for(j = 0; j < MAX_POINT_CNT; j ++)
		{
			if(g_stCurTouchPoint[i].trueCnt >= g_u8TrueCntScore[j])
				break;
		}

		if(j >= MAX_POINT_CNT)
		{
			DEBF("\r\n	TruePointMuchPickBest, P%d, TrueCnt:%d is small, drop!! \n",i,g_stCurTouchPoint[i].trueCnt);
			g_stCurTouchPoint[i].attri = IGNORE;
			g_u8CurTouchPointCnt --;
		}
	}

	if(g_u8CurTouchPointCnt > MAX_POINT_CNT)
	{
		DEBF("\r\n CurTouchPointCnt:%d, Pick Best Point, step 2 \n", g_u8CurTouchPointCnt);

		nowCnt = 0;

		for(i = 0; i < orgiPointCnt; i ++)
		{
			if(g_stCurTouchPoint[i].attri != IGNORE)
			{
				g_u16AdScore[nowCnt] = g_stCurTouchPoint[i].ad_value;
				nowCnt ++;
			}
		}
		
		//order  ad score, small -> bigger 
		for(i = 0; i < nowCnt - 1; i ++)
		{
			m = i;
			
			for(n = i + 1; n < nowCnt; n ++)
			{
				if(g_u16AdScore[m] > g_u16AdScore[n])
					m = n;
			}
	
			if(m != i)
			{
				tmp = g_u16AdScore[i];
				g_u16AdScore[i] = g_u16AdScore[m];
				g_u16AdScore[m] = tmp;
			}
		}

		m = g_u8CurTouchPointCnt;
		
		for(i = 0; i < m; i ++)
		{
			for(j = 0; j < MAX_POINT_CNT; j ++)
			{
				if(g_stCurTouchPoint[i].ad_value <= g_u16AdScore[j])
					break;
			}
	
			if(j >= MAX_POINT_CNT)
			{
				DEBF("\r\n	TruePointMuchPickBest, P%d, Score:%d is bigger, drop!! \n",i,g_stCurTouchPoint[i].ad_value);
				g_stCurTouchPoint[i].attri = IGNORE;
				g_u8CurTouchPointCnt --;
			}
		}
	}

	if(g_u8CurTouchPointCnt > MAX_POINT_CNT)
	{
		g_u8CurTouchPointCnt = MAX_POINT_CNT;
		ERRF("\r\n CurTouchPointCnt:%d, Pick Best Point fail!! line:%d\n", g_u8CurTouchPointCnt, __LINE__);
	}
	
	DEBF("\r\n CurTouchPointCnt:%d, Order residue Point, step 3 \n", g_u8CurTouchPointCnt);
	
	for(i = 0; i < orgiPointCnt - 1; i ++)
	{
		if(g_stCurTouchPoint[i].attri == IGNORE)
		{
			for(j = i + 1; j < orgiPointCnt; j ++)
			{
				if(g_stCurTouchPoint[j].attri != IGNORE)
				{
					g_stCurTouchPoint[i] = g_stCurTouchPoint[j];
					g_stCurTouchPoint[j].attri = IGNORE;
					break;
				}
			}
		}
	}
}

void UpdateTouchPoint(void)
{
	//uint8_t ret;
	
	
#if 1 /* vino marked*/ 
	if(g_u8LastTouchPointCnt == 0)//touch begin
	{
		if(g_u8CurTouchPointCnt >= 1)
		{
			TouchBeginSetLastTouch();
		}
	}
	else //touch running 
	{
		if(g_u8CurTouchPointCnt == 1 && g_u8LastTouchPointCnt == 1)
		{
			TouchOneMoveUpdateLastTouch();
		}
	}
#endif /* vino marked */
	FillBufferAndSendData();
	USB_MultiModeSendString();
}

 FakePointData_t testPointX[11] = 
{
	{50,50},
	{60,60},
	{70,70},
	{80,80},
	{90,90},
	{100,100},
	{90,90},
	{80,80},
	{70,70},
	{60,60},
	{50,50},
};

 FakePointData_t testPointY[11] =
{
	{50,50},
	{60,60},
	{70,70},
	{80,80},
	{90,90},
	{100,100},
	{90,90},
	{80,80},
	{70,70},
	{60,60},
	{50,50},
};
uint8_t Xcnt,Ycnt;

void testUART(void)
{
	USB_SIL_Write(EP1_IN, (uint8_t *)g_u8UsbSendBuffer, MULTI_MODE_REPORT_COUNT);  
	SetEPTxValid(ENDP1);
	//putchar('A');
	return;
#if  0 /* chenyan marked */
	Xcnt = 0;
	Ycnt = 0;
	memset(g_u8UsbSendBuffer,0,MULTI_MODE_REPORT_COUNT);
	g_u8UsbSendBuffer[0]=0x02;//ID
	
	g_u8UsbSendBuffer[1]=PRESS_STATUS;//State
	g_u8UsbSendBuffer[2]=0x00;//Current Point ID 0
	g_u8UsbSendBuffer[3]=0x50;//Low 8 X
	g_u8UsbSendBuffer[4]=0x00;//High 8 X
	g_u8UsbSendBuffer[5]=0x50;//Low 8 Y
	g_u8UsbSendBuffer[6]=0x00;//High 8 Y
	
	g_u8UsbSendBuffer[7]=0x04;//State
	g_u8UsbSendBuffer[8]=0x01;//Current Point ID 0
	g_u8UsbSendBuffer[9]=0x00;//Low 8 X
	g_u8UsbSendBuffer[10]=0x00;//High 8 X
	g_u8UsbSendBuffer[11]=0x00;//Low 8 Y
	g_u8UsbSendBuffer[12]=0x00;//High 8 Y
	
	g_u8UsbSendBuffer[13]=0x04;//State
	g_u8UsbSendBuffer[14]=0x02;//Current Point ID 0
	g_u8UsbSendBuffer[15]=0x00;//Low 8 X
	g_u8UsbSendBuffer[16]=0x00;//High 8 X
	g_u8UsbSendBuffer[17]=0x00;//Low 8 Y
	g_u8UsbSendBuffer[18]=0x00;//High 8 Y
	
	g_u8UsbSendBuffer[19]=0x04;//State
	g_u8UsbSendBuffer[20]=0x03;//Current Point ID 0
	g_u8UsbSendBuffer[21]=0x00;//Low 8 X
	g_u8UsbSendBuffer[22]=0x00;//High 8 X
	g_u8UsbSendBuffer[23]=0x00;//Low 8 Y
	g_u8UsbSendBuffer[24]=0x00;//High 8 Y
	
	g_u8UsbSendBuffer[25]=0x04;//State
	g_u8UsbSendBuffer[26]=0x04;//Current Point ID 0
	g_u8UsbSendBuffer[27]=0x00;//Low 8 X
	g_u8UsbSendBuffer[28]=0x00;//High 8 X
	g_u8UsbSendBuffer[29]=0x00;//Low 8 Y
	g_u8UsbSendBuffer[30]=0x00;//High 8 Y
	
	g_u8UsbSendBuffer[31]=0x04;//State
	g_u8UsbSendBuffer[32]=0x05;//Current Point ID 0
	g_u8UsbSendBuffer[33]=0x00;//Low 8 X
	g_u8UsbSendBuffer[34]=0x00;//High 8 X
	g_u8UsbSendBuffer[35]=0x00;//Low 8 Y
	g_u8UsbSendBuffer[36]=0x00;//High 8 Y
	
	g_u8UsbSendBuffer[37]=0x1;//High 8 Y
	
	

	//while(1)
	{
	
		g_u8UsbSendBuffer[1]=04;//PRESS_STATUS;//State
		g_u8UsbSendBuffer[2]=0x00;//Current Point ID 0
		g_u8UsbSendBuffer[3]=0x00;//Low 8 X
		g_u8UsbSendBuffer[4]=0x20;//High 8 X
		g_u8UsbSendBuffer[5]=0x00;//Low 8 Y
		g_u8UsbSendBuffer[6]=0x20;//High 8 Y
		printf(" ## VINO ## 	%d	: %s\r \n",__LINE__,__FUNCTION__);
			USB_SIL_Write(EP1_IN, (uint8_t *)g_u8UsbSendBuffer, MULTI_MODE_REPORT_COUNT);  
			SetEPTxValid(ENDP1);
			DelayMS(50);
#if 1/* vino marked*/ 
		g_u8UsbSendBuffer[1]=07;//RUNNING_STATUS;//State
		g_u8UsbSendBuffer[2]=0x00;//Current Point ID 0
		g_u8UsbSendBuffer[3]=0x70;//Low 8 X
		g_u8UsbSendBuffer[4]=0x20;//High 8 X
		g_u8UsbSendBuffer[5]=0x70;//Low 8 Y
		g_u8UsbSendBuffer[6]=0x20;//High 8 Y
			USB_SIL_Write(EP1_IN, (uint8_t *)g_u8UsbSendBuffer, MULTI_MODE_REPORT_COUNT);  
			SetEPTxValid(ENDP1);
			DelayMS(30);

#endif /* vino marked */
		g_u8UsbSendBuffer[1]=04;//PRESS_STATUS;//State
		g_u8UsbSendBuffer[2]=0x00;//Current Point ID 0
		g_u8UsbSendBuffer[3]=0x90;//Low 8 X
		g_u8UsbSendBuffer[4]=0x20;//High 8 X
		g_u8UsbSendBuffer[5]=0x90;//Low 8 Y
		g_u8UsbSendBuffer[6]=0x20;//High 8 Y
			USB_SIL_Write(EP1_IN, (uint8_t *)g_u8UsbSendBuffer, MULTI_MODE_REPORT_COUNT);  
			SetEPTxValid(ENDP1);
DelayMS(30);		
		//DelayMS(30000);
		//DelayMS(30);

	}
#endif /* chenyan marked */
}



/**
  * @brief  Task TxRx.
  * @param  pdata
  * @retval None
  */
void T_TxRx(void)
{
	uint8_t		ret;
	uint8_t		i,u8running,u8reChk;
	uint32_t    waitting;//cnt;
		
	DelayMS(10);

	g_bInitCheck = TRUE;
 	g_bUsbSendOpen = TRUE;
	g_bUartSendOpen = FALSE;
	//g_bUartSendOpen = TRUE;
	g_bXGetBadPoint = FALSE;
	g_bYGetBadPoint = FALSE;
	g_u16XBadPointCnt = 0;
	g_u16YBadPointCnt = 0;
	
	g_u8TouchMode = MOUSE_MODE;
	g_u8SingleModeStatus = LEFT_KEY;
	g_u8UartSendCnt = 0;
	g_u8AdjustScreen = 0;
	//g_u8GpioDebug = 0;
	g_u8LastDirec = 0;
	g_u16LastRevPoint = 0;
	g_u16LastSendPoint = 0;

	g_u8AdjustStatus = ADJUST_NO;
	g_u8AdjustIdCnt = 0;
	g_u8AdjustErrorCode = 0;
	g_s16AdjustOffsetX = 0;
	g_s16AdjustOffsetY = 0;
	g_u8MaskFlag = 0;

	printf("\r\n 01   \r\n");
	//Init_StaticPointEcurAndResData();
	printf("\r\n 02   \r\n");
	//Init_BadPointCheck();
	printf("\r\n 03   \r\n");

	Init_TouchData();
	printf("\r\n 04   \r\n");
	Init_FakePointData();
	printf("\r\n 05   \r\n");
	Init_PointTraceData();
	printf("\r\n 06   \r\n");
	
	//	清除标志位，否则第1 位数据会丢失
#ifdef  USE_UART_CONTROL_POINT_SEND
	USART_ClearFlag(MCU_COM_UART,USART_FLAG_TC); 
#endif

	printf("\r\n Tag-4.0 SYSTEM START RUNNING !!! \r\n");

	u8reChk = 0;
	u8running = 0;
	waitting = 0;

printf(" ## VINO ##     %d  : %s \n",__LINE__,__FUNCTION__);
printf(" ## VINO ##     %d  : %s \n",__LINE__,__FUNCTION__);
	g_u8YTotalAreaCnt = 0;
	return;
	//returntestUART();

	while(1)
	{
		while(g_u8YTotalAreaCnt == 0 && u8running == 0)
		{
			waitting ++;
			Fast_Sample_X_Points();
			if(g_u8MaskFlag >= 2)
				break;
			//DelayUS(500);
			Fast_Sample_Y_Points();
			if(g_u8MaskFlag >= 2)
				break;
			//DelayMS(1);
			if(waitting == 10000)// 10000 ~ 180s 
			{
				waitting = 0;
				//DEBF("\r\n *****************************************  waitting time out!! \n");
				
				if(g_bXGetBadPoint == TRUE || g_bYGetBadPoint == TRUE)
				{
					DEBF("\r\n ************************ in bad check!! \n");
					Init_BadPointCheck();
					DEBF("\r\n ************************ exit bad check!! \n");
				}
			}
		}
		ret = 0;
		//g_u8GpioDebug = 0;
		waitting = 0;
		
		g_u8YTotalAreaCnt = 0;
		g_u8XTotalAreaCnt = 0;
		g_u8XTotalFakePointCnt = 0;
 		g_u8YTotalFakePointCnt = 0;

		#if 0
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		#endif
	
		Dynamic_Sample_Y_Points();

		#if 0
		cnt = 0; 
		cnt = TIM_GetCounter(TIM3);
		TIM_Cmd(TIM3, DISABLE);

 		DEBF("\r\n cnt:%d \n",cnt);
		#endif
 
		//if(g_u8YTotalAreaCnt == 0 && u8running == 0)
		//{
		//	DelayUS(500);
		//}


		#if 0
		TIM_SetCounter(TIM3, 0);
		TIM_Cmd(TIM3, ENABLE);
		#endif
		
		Dynamic_Sample_X_Points();
		
		#if 0
		cnt = 0; 
		cnt = TIM_GetCounter(TIM3);
		TIM_Cmd(TIM3, DISABLE);

 		DEBF("\r\n cnt:%d \n",cnt);		
		#endif
		
		
		if(g_u8YTotalAreaCnt > 0 && g_u8XTotalAreaCnt > 0)
		{
			g_bReSearch = FALSE;
				
			u8reChk = 0;
			
			
			g_u8ScreenAreaNmb = g_u8YTotalAreaCnt * g_u8XTotalAreaCnt;
			
			if(u8running == 0)//start loop
			{
				u8running = 1;
				ERRF("\r\n ############### Point scan start!! ################ \n");
			}
			
			DEBF("\r\n -----------	One time scan - total area:%d, x area:%d, y area:%d  ----------- \n",g_u8ScreenAreaNmb,g_u8XTotalAreaCnt,g_u8YTotalAreaCnt);

			//g_u8GpioDebug = 1;
				
			g_u8CurTouchPointCnt = 0;

			if(g_u8ScreenAreaNmb == 1)//single AREA
			{
				ret = 0;
				g_bOneFakeArea = TRUE;
				
				g_u8YFakePointCnt = 0;
				g_u8XFakePointCnt = 0;
				QuickReVerticalScanGetFakePoint(0);
				//DEBF("\r\n XFake:%d, YFake:%d \n",g_u8XFakePointCnt,g_u8YFakePointCnt);
				
				if(g_u8XFakePointCnt > 0 && g_u8YFakePointCnt > 0)
				{
					if(g_u8XFakePointCnt == 1 && g_u8YFakePointCnt == 1)
					{
						g_stCurTouchPoint[g_u8CurTouchPointCnt].x_position = (X_FakePosition[0].pointStart + X_FakePosition[0].pointEnd) >> 1;
						g_stCurTouchPoint[g_u8CurTouchPointCnt].y_position = (Y_FakePosition[0].pointStart + Y_FakePosition[0].pointEnd) >> 1;
						
						//DEBF("\r\n 1 point fake(0) -> true(%d), x:%d, y:%d \n",g_u8CurTouchPointCnt,g_stCurTouchPoint[g_u8CurTouchPointCnt].x_position,g_stCurTouchPoint[g_u8CurTouchPointCnt].y_position);
						g_u8CurTouchPointCnt ++;
					}
					else 
					{
						BuildFakePointMarix(0);
						ret = ScanMarixToFilterFakePoint(0);
					}
				}
				else if(g_u8XTotalFakePointCnt > 0 && g_u8YTotalFakePointCnt > 0)
				{
					if(g_u8XTotalFakePointCnt == 1 && g_u8YTotalFakePointCnt == 1)
					{
						g_stCurTouchPoint[g_u8CurTouchPointCnt].x_position = (X_TotalFakePosition[0].pointStart + X_TotalFakePosition[0].pointEnd) >> 1;
						g_stCurTouchPoint[g_u8CurTouchPointCnt].y_position = (Y_TotalFakePosition[0].pointStart + Y_TotalFakePosition[0].pointEnd) >> 1;
						
						//DEBF("\r\n TotalFake 1 point fake(0) -> true(%d), x:%d, y:%d \n",g_u8CurTouchPointCnt,g_stCurTouchPoint[g_u8CurTouchPointCnt].x_position,g_stCurTouchPoint[g_u8CurTouchPointCnt].y_position);
						g_u8CurTouchPointCnt ++;
					}
					else if(g_u8XTotalFakePointCnt == 1 || g_u8YTotalFakePointCnt == 1)
					{
						BuildFakePointMarix(0xff);
						Only1LineFakePointMarixGetPoint(0xff);
					}
				}
				else
				{
					#ifdef USE_RESEARCH_FUNCTION
					g_bReSearch = TRUE;
					ERRF("\r\n Line:%d, Get point error happened, drop and re-search!! \n",__LINE__);
					#endif
				}
				
				Init_FakePointData();
				Init_GlobalFakePointData();
			}
			else// multi AREA
			{
				g_bOneFakeArea = FALSE;
				
				for(i = 0; i < g_u8ScreenAreaNmb; i ++)
				{
					ret = 0;
					g_u8TrueArea[i] = 0xff;
					g_u8YFakePointCnt = 0;
					g_u8XFakePointCnt = 0;

					if(g_u8CurTouchPointCnt >= POINT_BUFFER_SIZE)
						break;
					
					QuickReVerticalScanGetFakePoint(i);

					//DEBF("\r\n <scan area:%d>, XFake:%d, YFake:%d \n",i,g_u8XFakePointCnt,g_u8YFakePointCnt);

					if(g_u8XFakePointCnt != 0 && g_u8YFakePointCnt != 0)
					{
						BuildFakePointMarix(i);
						ret = ScanMarixToFilterFakePoint(i);

						if(g_bReSearch != TRUE)
						{
							if(ret != 0)
							{
								g_u8TrueArea[i] = TRUE;
							}
							else
							{
								g_u8TrueArea[i] = FALSE;
							}
						}
					}
					else
					{
						g_u8TrueArea[i] = FALSE;
						g_bReSearch = TRUE;
					}

					Init_FakePointData();

					if(g_bReSearch == TRUE)
					{
						ERRF("\r\n Line:%d, Get point error happened, drop and re-search!! \n",__LINE__);
						break;
					}
				}

				Init_GlobalFakePointData();
			}
			
			if(g_u8CurTouchPointCnt >= 1 && g_bReSearch != TRUE)
			{
				if(g_u8CurTouchPointCnt > MAX_POINT_CNT)
				{
					TruePointMuchPickBest();
				}
				
				UpdateTouchPoint();
			}
			else if(g_bReSearch == TRUE)
			{
				ERRF("\r\n ############### Point scan Error, re-scan !! ################ \n"); 
			}

			Init_PointTraceData();
		}
		else if(u8running == 1)
		{
			u8reChk ++;
			
			DEBF("\r\n -----------	One time scan - x area:%d, y area:%d  ----------- \n",g_u8XTotalAreaCnt,g_u8YTotalAreaCnt);
			
			if(u8reChk < 5 && (g_u8XTotalAreaCnt > 0 || g_u8YTotalAreaCnt > 0))
			{
				//DelayUS(100);
				ERRF("\r\n ############### Point scan lost, retry:%d !! ################ \n", u8reChk);
				continue;
			}
			
			u8reChk = 0;
			ERRF("\r\n ############### Point scan end !! ################ \n");
			if(g_u8LastTouchPointCnt > 0)
			{
				SendTouchEndToTerminal();
			}
			u8running = 0;
			Init_TouchData();
		}
		
		//if(g_u8XTotalAreaCnt == 0 && u8running == 0)
		//{
		//	DelayMS(1);
		//}
		//DelayMS(10);
		#ifdef USE_FINE_TUNE_SCREEN_FUNCTION
		adjustScreenFun();
		#endif

		
	}
}


/* End of File ****************************************************************/
