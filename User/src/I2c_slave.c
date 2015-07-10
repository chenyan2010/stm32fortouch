#include "I2c_slave.h"
#include "stm32f10x_i2c.h"
#include "gpio.h"

uint8_t i2c_regaddr = 0;
bitfield8 tagI2C;
uint8_t i2c_index = 0;//i2c buf index.
#define I2C_DATA_SIZE 8
uint8_t i2c_data[I2C_DATA_SIZE] = {0x0};
uint16_t u16RetArray[]={0x4350, 0x4c55, 0x5649, 0x4e4f, 0x5300};// for what ?
extern uint16_t u16send_data;
uint8_t recv_datArray[RECV_DATA_SIZE] = {0};
uint8_t g_u8for818send_data = 0xFF;//for customer uart command.
extern uint8_t isUartRecving;
uint8_t isPoweroff = 0;//indicate 818 whether power off.

extern int SendChar (int ch);

void I2C1_EV_IRQHandler(void)
{
		uint8_t i = 0;
	uint32_t Event=I2C_GetLastEvent(I2C1);
	if(Event==I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED)
	{
			i2c_index = 0;//clear index.
		tagI2C.Bit0 = 1;//bit0 is tagi2c for smart signal switch func.
		i2c_data[i2c_index]  = I2C_ReceiveData(I2C1);
	}
	/*slave received the date byte from master*/
	/*EV2:RxNE=1,?DR???????*/
	else if(Event == (I2C_EVENT_SLAVE_BYTE_RECEIVED))
	{
		i2c_data[i2c_index ++]  = I2C_ReceiveData(I2C1);
		i2c_regaddr = i2c_data[0] & 0xFF;//receive register address.

		if(i2c_index > 3)//three bytes. 1 byte regaddr + 3 bytes data.
		{
				for(i = 0;i < RECV_DATA_SIZE; i++)
				{
					recv_datArray[i]=i2c_data[i+1];
				}         
			i2c_index = 0;//clear index.
			tagI2C.Bit1= 1;//receive end.//bit1 is reveive end for smart signal switch func.
		}

		switch(i2c_regaddr)
		{
		    case REGADDR_UART_CMD1:
                i2c_index = 0;
                if(isPoweroff)
                {
                    isPoweroff = 0;//818 is power on.
                    i2c_data[0] = 0xff;
                }
                else
                {
                    i2c_data[0] = g_u8for818send_data;//one byte.
                }
                if(g_u8for818send_data != 0xff && isUartRecving == 0)
                {
                    g_u8for818send_data = 0xff;//clear the send data.
                }
                break;
            case REGADDR_UART_CMD2:
                if(i2c_index > 1)//write one byte.
                {
                    i2c_index = 0;
                    if(i2c_data[1] == 0x01)
                    {
                        isPoweroff = 1;//818 is power off.
                    }
                }
                break;
			case REGADDR_ENCRYPTION_ID1:
			case REGADDR_ENCRYPTION_ID2:
			case REGADDR_ROUTINE1:
			case REGADDR_ROUTINE2:
			case REGADDR_CONFIRM:
				i2c_index = 0;//clear the index.
				i2c_data[0] = (u16RetArray[i2c_regaddr-REGADDR_ENCRYPTION_ID1]>> 8) & 0xFF;
				i2c_data[1] = u16RetArray[i2c_regaddr-REGADDR_ENCRYPTION_ID1] & 0xFF;
				break;
		}
	}

	/*SLAVE TRANSMITTER*/
	/*Data transmited from slave to master*/
	/*EV3: TxE=1,?DR?????DR*/
	//slave transmit mode. 
	else if(Event == (I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED) || Event == (I2C_EVENT_SLAVE_BYTE_TRANSMITTED) )
	{
		I2C_SendData(I2C1,i2c_data[i2c_index ++]);
	}
	/*Acknowledge failure*/
	/*EV3-2:AF=1,AF is cleared by writing '0' in AF bit of SR2 register*/
	else if(Event == (I2C_EVENT_SLAVE_ACK_FAILURE))
	{
			I2C_ClearFlag(I2C1,I2C_FLAG_AF);
			i2c_index = 0;
	}
	/*Slave receivered STOP byte from master*/
	/*EV4: STOPF=1,?SR1??????CR2??*/
	else if(Event == (I2C_EVENT_SLAVE_STOP_DETECTED))
	{
			I2C_GetFlagStatus(I2C1,I2C_FLAG_STOPF);
			I2C_GenerateSTOP(I2C1,DISABLE);//clear stop.
			i2c_index = 0;
	}
}

void I2C1_ER_IRQHandler(void)
{

		__IO uint32_t SR1Register =0;

		/* Read the I2C1 status register */
		SR1Register = I2C1->SR1;
		/* If AF = 1 */
		if ((SR1Register & 0x0400) == 0x0400)
		{
				I2C1->SR1 &= 0xFBFF;
				SR1Register = 0;
		}
		/* If ARLO = 1 */
		if ((SR1Register & 0x0200) == 0x0200)
		{
				I2C1->SR1 &= 0xFBFF;
				SR1Register = 0;
		}
		/* If BERR = 1 */
		if ((SR1Register & 0x0100) == 0x0100)
		{
				I2C1->SR1 &= 0xFEFF;
				SR1Register = 0;
		}

		/* If OVR = 1 */

		if ((SR1Register & 0x0800) == 0x0800)
		{
				I2C1->SR1 &= 0xF7FF;
				SR1Register = 0;
		}
}


