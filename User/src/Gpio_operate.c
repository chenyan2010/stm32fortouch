/*-------------------------------------------------------------------------
 file Gpio_operate.c
 GPIO operator.
-------------------------------------------------------------------------*/

#define GPIO_OPERATE_C

#include "Gpio_operate.h"

///////////////////////////////////////////////////////////////////////////////
//<FuncName>: disable_signalforcheck.
//<Description>: disbale signal for check.
//<Param[IN]>: CHANNEL_NUM_TYPE channel_num.
//<Returns>: void.
//<Author>: ChenYan.
//<Date>: 2014/10/16.
///////////////////////////////////////////////////////////////////////////////
#if 0
void disable_signalforcheck(CHANNEL_NUM_TYPE channel_num)
{
	if(channel_num < CHANNEL_YPBPR || channel_num > CHANNEL_HDMI6)
	{
		printf("param channel_num error!\n");
		return;
	}
	
       if(channel_num >= CHANNEL_HDMI0)
       	{
   		Close_HDMIChk(0);
       	}
       /*
       else if(channel_num >= CHANNEL_VGA1)
       	{
       		switch(channel_num)
		{
		       //pre VGA,VGA1 use the same port.
			case CHANNEL_VGA1://VGA1
				Close_VGAChk(1);
				break;
				
			case CHANNEL_VGA2://VGA2
				Close_VGAChk(2);
				break;
				
			case CHANNEL_VGA3://pre VGA
				Close_VGAChk(1);
				break;				
			default:
				break;
		}
       	}
	else if(channel_num >= CHANNEL_AV1)
	{
		switch(channel_num)
		{
			case CHANNEL_AV1://AV1
				Close_CVBSChk(1);
				break;
				
			case CHANNEL_AV2://SVIDEO,AV2,AV3
				Close_CVBSChk(2);
				break;
				
			case CHANNEL_AV3://AV3
				Close_CVBSChk(3);
                                        Close_CVBSChk(2);
				break;
				
			default:
				break;
		}
	}
	else
		Close_YPbPrChk();//YPBPR
		*/
}
#endif

///////////////////////////////////////////////////////////////////////////////
//<FuncName>: enable_signaltoSOC.
//<Description>: enable signal to SOC.
//<Param[IN]>: CHANNEL_NUM_TYPE channel_num.
//<Returns>: void.
//<Author>: ChenYan.
//<Date>: 2014/10/16.
///////////////////////////////////////////////////////////////////////////////
void enable_signaltoSOC(CHANNEL_NUM_TYPE channel_num)
{
	//disable_signalforcheck(channel_num);
}


///////////////////////////////////////////////////////////////////////////////
//<FuncName>: check_signalinput_status.
//<Description>: check the signal input status.
//<Param[IN]>: CHANNEL_NUM_TYPE channel_num.
//<Returns>: bool status(TRUE or FALSE)
//<Author>: ChenYan.
//<Date>: 2014/10/16.
///////////////////////////////////////////////////////////////////////////////
bool check_signalinput_status(CHANNEL_NUM_TYPE channel_num)
{
       bool signal_input_status = FALSE;
	if(channel_num < CHANNEL_YPBPR || channel_num > CHANNEL_HDMI6)
	{
		printf("param channel_num error!\n");// high the accord printf error.
		return FALSE;
	}


    switch(channel_num)
    {
            //HDMI check.
            case CHANNEL_HDMI0://DVI   (standby)
			     //signal_input_status = (GPIO_ReadInputDataBit(GPIOC, HDMI4_CHK_IN) == (uint8_t)RESET);
				break;
            case CHANNEL_HDMI1://HDMI1
			     signal_input_status = (GPIO_ReadInputDataBit(GPIOB, HDMI1_CHK_IN) == 0) ? TRUE : FALSE;
				break;
            case CHANNEL_HDMI2://HDMI2
			     signal_input_status = (GPIO_ReadInputDataBit(GPIOB, HDMI2_CHK_IN) == 0) ? TRUE : FALSE;
				break;
            case CHANNEL_HDMI4://HDMI4
			     signal_input_status = (GPIO_ReadInputDataBit(GPIOB, HDMI4_CHK_IN) == 0) ? TRUE : FALSE;
				break;
            case CHANNEL_HDMI5://HDMI5
			     signal_input_status = (GPIO_ReadInputDataBit(GPIOB, HDMI5_CHK_IN) == 0) ? TRUE : FALSE;
				break;
            case CHANNEL_HDMI6://pre HDMI
			     signal_input_status = (GPIO_ReadInputDataBit(GPIOB, HDMI6_CHK_IN) == 0) ? TRUE : FALSE;
				break;
                
            //VGA check.
			case CHANNEL_VGA1://VGA1
			     signal_input_status = (GPIO_ReadInputDataBit(GPIOB, VGA1_CHK_IN) == 0) ? TRUE : FALSE;
				break;
				
			case CHANNEL_VGA2://VGA2
			    signal_input_status = (GPIO_ReadInputDataBit(GPIOB, VGA2_CHK_IN) == 0) ? TRUE : FALSE;	
				break;
				
			case CHANNEL_VGA3://VGA3.
			    signal_input_status = (GPIO_ReadInputDataBit(GPIOB, VGA3_CHK_IN) == 0) ? TRUE : FALSE;
				break;		

            //AV check.
			case CHANNEL_AV1://AV1
                signal_input_status = (GPIO_ReadInputDataBit(GPIOB, AV1_CHECK_IN) != 0) ? TRUE : FALSE;
				break;
				
			case CHANNEL_AV2://AV2  
                signal_input_status = (GPIO_ReadInputDataBit(GPIOB, AV2_CHECK_IN) != 0) ? TRUE : FALSE;
				break;
				
			case CHANNEL_AV3://SVIDEO
                //signal_input_status = (GPIO_ReadInputDataBit(GPIOB, SVIDEO_CHK_IN) != (uint8_t)RESET);
				break;
				
			case CHANNEL_YPBPR://YPBPR    (STANDBY)
                signal_input_status = (GPIO_ReadInputDataBit(GPIOC, YPBPR_CHK_IN) != 0) ? TRUE : FALSE;
				break;
		    default:
				break;
	}

	return signal_input_status;//if signal input status is TRUE tell us has a signal input.

}

