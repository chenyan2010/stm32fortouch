#ifndef __I2C_SLAVE_H
#define __I2C_SLAVE_H
#include "stm32f10x.h"
#include "stm32f10x_it.h"

typedef struct {
        unsigned char Bit0:1;
        unsigned char Bit1:1;
        unsigned char Bit2:1;
        unsigned char Bit3:1;
        unsigned char Bit4:1;
        unsigned char Bit5:1;
        unsigned char Bit6:1;
        unsigned char Bit7:1;
}bitfield8;

#define RECV_DATA_SIZE  3

/********************** I2C configuration variables *****************************/  
	/* Define I2C Address mode ---------------------------------------------------*/
	#define I2C_slave_7Bits_Address

	/* Define Slave Address  -----------------------------------------------------*/
	#ifdef I2C_slave_10Bits_Address
		#define SLAVE_ADDRESS  0x3F0
	#endif 

	#ifdef I2C_slave_7Bits_Address
		#define SLAVE_ADDRESS  (0x52 << 1)    //chenyan 20141015 modified.
	#endif

#endif

