#ifndef    	_USER_CONFIG__
#define  	 	_USER_CONFIG__

#include "stm32f10x.h"
void ConfigurationProgram(void);
int Flash_Read(uint32_t iAddress, uint8_t *buf, uint32_t iNbrToRead) ;
//uint8_t AcrInfoData(uint8_t *infoData,uint8_t *Data,uint8_t length);
//uint8_t AsciiToDec(uint8_t *p_AsciiData);
#endif

