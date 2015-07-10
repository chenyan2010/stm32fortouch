#include "user_config.h"
#include "stm32f10x_flash.h"
#include "stdio.h"
#include "uart.h"
char swversion[32]={":SWVERSION=aaaa.baaa.cccc.dddd\r\n"};
char hwversion[22]={":HWVERSION=aaaa.bbbb\r\n"};
char hwmodel[20]={":HWMODEL=aaaa.bbbb\r\n"};
char ouid[22]={":OUID=0000.0000.0000\r\n"};
uint32_t UserInfoAddr= 0x8003000;
static uint32_t	InfoAddr;

/*-----------------------------------------------------------------------------------------------*/

/**************************************************************************************************
** 函数名称 : AsciiToDec
** 功能描述 : 将一个ASCII字符码转换为数值
** 入口参数 : <p_AsciiData>[in] 被转换的ASCII数据地址
** 出口参数 : 无
** 返 回 值 : 转换后的数值
** 其他说明 : 无
***************************************************************************************************/
uint8_t AsciiToDec(uint8_t *p_AsciiData)
{
	if(p_AsciiData!=NULL)
	{
		if((*p_AsciiData>='0')&&(*p_AsciiData<='9'))
		{
			return ((*p_AsciiData)-'0');
		}
		else if((*p_AsciiData>='A')&&(*p_AsciiData<='F'))
		{
			return	((*p_AsciiData)-'A'+10);
		}
		else if((*p_AsciiData>='a')&&(*p_AsciiData<='f'))
		{
			return	((*p_AsciiData)-'a'+10);
		}
		else
		{
			return(*p_AsciiData);
		}
	}
	else
	{
		return ERROR;
	}	
}





/**************************************************************************************************
** 函数名称 : AcrInfoData
** 功能描述 : 根据传入设备信息以及用户信息解析为写flash时需要的格式数据
** 入口参数 : 设备信息以及用户数据起始数据地址
** 出口参数 : 数据信息
** 返 回 值 : 无其他异常返回0
** 其他说明 : 无
***************************************************************************************************/
uint8_t AcrInfoData(uint8_t *infoData,uint8_t *Data,uint8_t length)
{
	uint8_t ErrorCode=0;
	uint8_t i=0;
	for(i=0;i<length;i++)
	{
		Data[i]=(AsciiToDec(infoData+2*i)<<4)+(AsciiToDec(infoData+2*i+1));
	}	
	return ErrorCode;
}



/**
  * @brief  将数据烧写到指定地址的Flash中 。
  * @param  Address Flash起始地址。
  * @param  Data 数据存储区起始地址。
  * @param  DataNum 数据字节数。
  * @retval 数据烧写状态。
  */
FLASH_Status ProgramDatatoFlash(uint32_t Address,uint16_t *Data,uint32_t DataNum) 
{
	FLASH_Status FLASHStatus=FLASH_COMPLETE;
	uint32_t i;
	uint16_t *p_Data=Data;
	FLASH_Unlock();
	for(i=0;i<(DataNum>>1);i++){
		FLASHStatus=FLASH_ProgramHalfWord(Address, *p_Data);
		if(FLASHStatus!=FLASH_COMPLETE){
			return	FLASHStatus;	
		}
		Address+=2;
		p_Data++;
	}
	return	FLASHStatus;
	
}

/**
  * @brief  读取Flash指定地址的数据
  */
int Flash_Read(uint32_t iAddress, uint8_t *buf, uint32_t iNbrToRead) 
{
    int i = 0;
    while(i < iNbrToRead) 
    {
        *(buf + i) = *(__IO uint32_t*) iAddress++;
        i++;
    }
    return i;
}
uint8_t UserData[16];
void program(char* name,uint8_t length)
{
	
	//uint8_t j;
	
	uint32_t	DST;
	uint8_t Read_Flash[16]; 
	DST = UserInfoAddr+InfoAddr;
	//printf("addr %x\r\n",DST);
	AcrInfoData((uint8_t *)name,UserData,length);
	ProgramDatatoFlash(DST,(uint16_t*)UserData,length);
	Flash_Read(DST,Read_Flash,length);
	//for(j = 0;j<length;j++)
	//printf("%x ",Read_Flash[j]);
	//printf("\r\n");
	InfoAddr+=0x10;
}
/**
  *	@}
  */
/**
  * @brief  擦出指定扇区区间的Flash数据 。
  * @param  StartPage 起始扇区
  * @param  Noen
  * @retval Noen
  */
void ErasePage(uint32_t iAddress)
{
	FLASH_Unlock();
	FLASH_ErasePage(iAddress);	
}

void ConfigurationProgram()
{
	ErasePage(UserInfoAddr);
	program(swversion,16);
	program(hwversion,10);	
	program(hwmodel,10);
	program(ouid,10);

}




