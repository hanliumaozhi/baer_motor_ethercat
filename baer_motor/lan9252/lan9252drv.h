
#ifndef __LAN9252DRV_H
#define __LAN9252DRV_H

#include <stdio.h>
#include <string.h>
#include "stm32h7xx_hal.h"

#define CUST_BYTE_NUM_OUT	64
#define CUST_BYTE_NUM_IN	92
#define TOT_BYTE_NUM_ROUND_OUT	64
#define TOT_BYTE_NUM_ROUND_IN	92

                                                        
#define SEC_BYTE_NUM_IN  (CUST_BYTE_NUM_IN - 64)   // number of bytes of the second transfer
  
#if ((SEC_BYTE_NUM_IN & 0x03) != 0x00)            // number of bytes of the second transfer
	#define SEC_BYTE_NUM_ROUND_IN  ((SEC_BYTE_NUM_IN | 0x03) + 1)  
#else                                             // rounded to 4 (long)
	#define SEC_BYTE_NUM_ROUND_IN  SEC_BYTE_NUM_IN  //
#endif                                            //

#define SEC_LONG_NUM_IN  SEC_BYTE_NUM_ROUND_IN/4  // number of long of the second transfer

#define FST_BYTE_NUM_IN  64                       // number of bytes of the first transfer     
#define FST_BYTE_NUM_ROUND_IN  64                 // number of bytes of the first transfer
                                                  // rounded to 4 (long)
#define FST_LONG_NUM_IN  28                       // number of long of the second transfer

#define SPI_TIMEOUT_MAX				0x1000
#define CMD_LENGTH					((uint16_t)0x0004)

#define COMM_SPI_READ				0x03
#define COMM_SPI_WRITE				0x02

#define ESC_WRITE 		   			0x80
#define ESC_READ 		   			0xC0
#define ECAT_CSR_BUSY     			0x80

#define AL_CONTROL              	0x0120      			// AL control
#define AL_STATUS               	0x0130      			// AL status
#define AL_STATUS_CODE          	0x0134      			// AL status code
#define AL_EVENT                	0x0220      			// AL event request
#define AL_EVENT_MASK           	0x0204      			// AL event interrupt mask

#define IRQ_CFG                 0x0054      // interrupt configuration
#define INT_EN                  0x005C      // interrupt enable

#define PRAM_ABORT        			0x40000000
#define PRAM_BUSY         			0x80
#define PRAM_AVAIL        			0x01
#define READY             			0x08
#define DUMMY_BYTE					0xFF

#define BYTE_TEST               	0x0064      			// byte order test register
#define HW_CFG                  	0x0074      			// hardware configuration register
#define RESET_CTL               	0x01F8      			// reset register
#define ECAT_CSR_DATA           	0x0300      			// EtherCAT CSR Interface Data Register
#define ECAT_CSR_CMD            	0x0304      			// EtherCAT CSR Interface Command Register
#define ECAT_PRAM_RD_ADDR_LEN   	0x0308      			// EtherCAT Process RAM Read Address and Length Register
#define ECAT_PRAM_RD_CMD        	0x030C      			// EtherCAT Process RAM Read Command Register
#define ECAT_PRAM_WR_ADDR_LEN   	0x0310      			// EtherCAT Process RAM Write Address and Length Register
#define ECAT_PRAM_WR_CMD        	0x0314      			// EtherCAT Process RAM Write Command Register
#define WDOG_STATUS             	0x0440      			// watch dog status
#define ECAT_LOCAL_TIME				0x0910

#define DIGITAL_RST       			0x00000001

#define ESM_INIT                	0x01          			// state machine control
#define ESM_PREOP               	0x02          			// (state request)
#define ESM_BOOT                	0x03          			//
#define ESM_SAFEOP              	0x04          			// safe-operational
#define ESM_OP                  	0x08          			// operational

#define Tout 						2000

typedef union {
    uint16_t  Word;
    uint8_t   Byte[2];
} UWORD;

typedef union {
    uint32_t   Long;
    uint16_t  Word[2];
    uint8_t   Byte[4];
} ULONG;


typedef union												//---- output buffer ----
{
	uint8_t  Byte[TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		uint64_t    hs;
		uint64_t    motor_1;
		uint64_t    motor_2;
		uint64_t    motor_3;
		uint64_t    motor_4;
		uint64_t    motor_5;
		uint64_t    motor_6;
		uint32_t    test_word;
		uint16_t    control_word;
		uint16_t    motor_enable;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte[TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		uint64_t    hs;
		uint64_t    motor_1;
		uint64_t    motor_2;
		uint64_t    motor_3;
		uint64_t    motor_4;
		uint64_t    motor_5;
		uint64_t    motor_6;
		uint64_t    test_word_byte_8;
		uint64_t    hs_record;
		uint32_t    can1_error_log;
		uint32_t    can2_error_log;
		uint32_t    test_word_byte_4;
		uint16_t    rec_error_can2;
		uint16_t    rec_error_can1;
		uint16_t    motor_status;
		uint16_t    fsm;
	}Cust;
} PROCBUFFER_IN;

typedef struct {
	UART_HandleTypeDef* uart;
	SPI_HandleTypeDef* 	spi;
	PROCBUFFER_IN* 		bIn;
	PROCBUFFER_OUT* 	bOut;
	uint8_t				deviceInitiated;
	uint32_t			deviceTime;
} spiCTX;


unsigned long SPIReadRegisterDirect(spiCTX* ctx, unsigned short Address, unsigned char Len);
void SPIWriteRegisterDirect (spiCTX* ctx, unsigned short Address, unsigned long DataOut);
unsigned long SPIReadRegisterIndirect (spiCTX* ctx, unsigned short Address, unsigned char Len);
void SPIWriteRegisterIndirect (spiCTX* ctx, unsigned long DataOut, unsigned short Address, unsigned char Len);
void SPIWriteProcRamFifo(spiCTX* ctx);
void SPIReadProcRamFifo(spiCTX* ctx);

#endif
