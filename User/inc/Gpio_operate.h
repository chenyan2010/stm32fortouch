#ifndef GPIO_OPERATE_H
#define GPIO_OPERATE_H
#include <stdio.h>
#include "stm32f10x.h"
#include "gpio.h"


/*typedef*/
typedef enum
{
	CHANNEL_YPBPR,
	CHANNEL_AV1,
	CHANNEL_AV2,
	CHANNEL_AV3,
	CHANNEL_VGA1,
	CHANNEL_VGA2,
	CHANNEL_VGA3,
	CHANNEL_HDMI0,
	CHANNEL_HDMI1,
	CHANNEL_HDMI2,
	CHANNEL_HDMI3,
	CHANNEL_HDMI4,
	CHANNEL_HDMI5,
	CHANNEL_HDMI6,
	CHANNEL_NONE
}CHANNEL_NUM_TYPE;
extern void enable_signaltoSOC(CHANNEL_NUM_TYPE channel_num);

extern bool check_signalinput_status(CHANNEL_NUM_TYPE channel_num);

#endif

