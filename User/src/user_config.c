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
** �������� : AsciiToDec
** �������� : ��һ��ASCII�ַ���ת��Ϊ��ֵ
** ��ڲ��� : <p_AsciiData>[in] ��ת����ASCII���ݵ�ַ
** ���ڲ��� : ��
** �� �� ֵ : ת�������ֵ
** ����˵�� : ��
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
** �������� : AcrInfoData
** �������� : ���ݴ����豸��Ϣ�Լ��û���Ϣ����Ϊдflashʱ��Ҫ�ĸ�ʽ����
** ��ڲ��� : �豸��Ϣ�Լ��û�������ʼ���ݵ�ַ
** ���ڲ��� : ������Ϣ
** �� �� ֵ : �������쳣����0
** ����˵�� : ��
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
  * @brief  ��������д��ָ����ַ��Flash�� ��
  * @param  Address Flash��ʼ��ַ��
  * @param  Data ���ݴ洢����ʼ��ַ��
  * @param  DataNum �����ֽ�����
  * @retval ������д״̬��
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
  * @brief  ��ȡFlashָ����ַ������
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
  * @brief  ����ָ�����������Flash���� ��
  * @param  StartPage ��ʼ����
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




