#include "lan9252drv.h"
#include "main.h"

unsigned long SPIReadRegisterDirect(spiCTX* ctx, unsigned short Address, unsigned char Len) {
	ULONG Command, Result;
	UWORD Addr;
	char uartbuffer[64] = { 0 };

	Addr.Word = Address;
	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_RESET);

	Command.Byte[0] = COMM_SPI_READ;
	Command.Byte[1] = Addr.Byte[1];
	Command.Byte[2] = Addr.Byte[0];

	if (HAL_SPI_Transmit(ctx->spi, Command.Byte, 3, SPI_TIMEOUT_MAX) != HAL_OK) {
		//memset(uartbuffer, 0, 64);
				//sprintf(uartbuffer, "ERROR# HAL_SPI_Transmit\r\n\r\n");
	}

	if (HAL_SPI_Receive(ctx->spi, Result.Byte, Len, SPI_TIMEOUT_MAX) != HAL_OK) {
		//memset(uartbuffer, 0, 64);
		//sprintf(uartbuffer, "ERROR# HAL_SPI_Receive\r\n\r\n");
	}

	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_SET);

	return Result.Long;
}

void SPIWriteRegisterDirect(spiCTX* ctx, unsigned short Address, unsigned long DataOut) {
	ULONG Data;
	UWORD Addr;
	uint8_t Buffer[8] = { 0 };
	char uartbuffer[64] = { 0 };

	Addr.Word = Address;
	Data.Long = DataOut;

	Buffer[0] = COMM_SPI_WRITE;
	Buffer[1] = Addr.Byte[1];
	Buffer[2] = Addr.Byte[0];
	Buffer[3] = Data.Byte[0];
	Buffer[4] = Data.Byte[1];
	Buffer[5] = Data.Byte[2];
	Buffer[6] = Data.Byte[3];

	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(ctx->spi, Buffer, 7, SPI_TIMEOUT_MAX) != HAL_OK) {
		memset(uartbuffer, 0, 64);
		sprintf(uartbuffer, "ERROR# HAL_SPI_Transmit\r\n\r\n");
	}

	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_SET);
}

unsigned long SPIReadRegisterIndirect(spiCTX* ctx, unsigned short Address, unsigned char Len) {
	ULONG TempLong;
	UWORD Addr;

	Addr.Word = Address;

	/*
	  CSR_BUSY	|	R_nW	|	RESERVED	|	CSR_SIZE	|	CSR_ADDR
	  31			|	30		|	29:19		|	18:16		|	15:0
	 */
	TempLong.Byte[0] = Addr.Byte[0]; // address of the register
	TempLong.Byte[1] = Addr.Byte[1]; // to read, LsByte first
	TempLong.Byte[2] = Len; // number of bytes to read
	TempLong.Byte[3] = ESC_READ; // ESC read
	SPIWriteRegisterDirect(ctx, ECAT_CSR_CMD, TempLong.Long);

	// wait for command execution
	do {
		TempLong.Long = SPIReadRegisterDirect(ctx, ECAT_CSR_CMD, 4);
	} while (TempLong.Byte[3] & ECAT_CSR_BUSY);

	TempLong.Long = SPIReadRegisterDirect(ctx, ECAT_CSR_DATA, Len); // read the requested register

	return TempLong.Long;
}

void SPIWriteRegisterIndirect(spiCTX* ctx, unsigned long DataOut, unsigned short Address, unsigned char Len) {
	ULONG TempLong;
	UWORD Addr;

	Addr.Word = Address;

	SPIWriteRegisterDirect(ctx, ECAT_CSR_DATA, DataOut); // write the data

	TempLong.Byte[0] = Addr.Byte[0]; // address of the register
	TempLong.Byte[1] = Addr.Byte[1]; // to write, LsByte first
	TempLong.Byte[2] = Len; // number of bytes to write
	TempLong.Byte[3] = ESC_WRITE; // ESC write

	SPIWriteRegisterDirect(ctx, ECAT_CSR_CMD, TempLong.Long); // write the command

	do {
		TempLong.Long = SPIReadRegisterDirect(ctx, ECAT_CSR_CMD, 4);
	} while (TempLong.Byte[3] & ECAT_CSR_BUSY);
}

//#define FST_BYTE_NUM_ROUND_IN TOT_BYTE_NUM_IN
/*
 * From master
 */
void SPIWriteProcRamFifo(spiCTX* ctx) {
	ULONG TempLong;
	unsigned char i;
	uint8_t Buffer[32] = { 0 };
	char uartbuffer[64] = { 0 };

	// abort any possible pending transfer
	SPIWriteRegisterDirect(ctx, ECAT_PRAM_WR_CMD, PRAM_ABORT);
	/*
	 * Send address and length
	  PRAM_WRITE_LEN	|	PRAM_WRITE_ADDR
	  31:16			|	15:0
	*/
	SPIWriteRegisterDirect(ctx, ECAT_PRAM_WR_ADDR_LEN, (0x00001200 | (((uint32_t)TOT_BYTE_NUM_ROUND_IN) << 16)));
	/*
	 *
	  PRAM_WRITE_BUSY	|	PRAM_WRITE_ABORT	|	RESERVED	|	PRAM_WRITE_AVAIL_CNT	|	RESERVED	| 	PRAM_WRITE_AVAIL
	  31				|	30					|	29:13		|	12:8					|	7:1 		| 	0
	*/
	SPIWriteRegisterDirect(ctx, ECAT_PRAM_WR_CMD, 0x80000000); // start command (set PRAM_WRITE_BUSY)

	do {
		TempLong.Long = SPIReadRegisterDirect(ctx, ECAT_PRAM_WR_CMD, 2);
	} while (TempLong.Byte[1] < (FST_BYTE_NUM_ROUND_IN / 4));

	Buffer[0] = COMM_SPI_WRITE;
	Buffer[1] = 0x00; // address of the write fifo
	Buffer[2] = 0x20; // MsByte first (ECAT_PRAM_WR_DATA 020h-03Ch [ETHERCAT PROCESS RAM WRITE DATA FIFO])

	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(ctx->spi, Buffer, 3, SPI_TIMEOUT_MAX) != HAL_OK) {
	}
	if (HAL_SPI_Transmit(ctx->spi, ctx->bIn->Byte, FST_BYTE_NUM_ROUND_IN, SPI_TIMEOUT_MAX) != HAL_OK) {
	}
	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_SET);
	
	// second 
	do {                                                                  
		TempLong.Long = SPIReadRegisterDirect(ctx, ECAT_PRAM_WR_CMD, 2);
	} while (TempLong.Byte[1] < (SEC_BYTE_NUM_ROUND_IN / 4));
	
	Buffer[0] = COMM_SPI_WRITE;
	Buffer[1] = 0x00; // address of the write fifo
	Buffer[2] = 0x20; // MsByte first (ECAT_PRAM_WR_DATA 020h-03Ch [ETHERCAT PROCESS RAM WRITE DATA FIFO])

	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(ctx->spi, Buffer, 3, SPI_TIMEOUT_MAX) != HAL_OK) {
	}
	if (HAL_SPI_Transmit(ctx->spi, &ctx->bIn->Byte[64], SEC_BYTE_NUM_ROUND_IN, SPI_TIMEOUT_MAX) != HAL_OK) {
	}
	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_SET);
}

#define FST_BYTE_NUM_ROUND_OUT TOT_BYTE_NUM_ROUND_OUT
/*
 * To master
 */
void SPIReadProcRamFifo(spiCTX* ctx) {
	ULONG TempLong;
	uint8_t Buffer[32] = { 0 };

	// abort any possible pending transfer
	SPIWriteRegisterDirect(ctx, ECAT_PRAM_RD_CMD, PRAM_ABORT);
	SPIWriteRegisterDirect(ctx, ECAT_PRAM_RD_ADDR_LEN, (0x00001000 | (((uint32_t)TOT_BYTE_NUM_ROUND_OUT) << 16)));
	// start command
	SPIWriteRegisterDirect(ctx, ECAT_PRAM_RD_CMD, 0x80000000);

	do {
		TempLong.Long = SPIReadRegisterDirect(ctx, ECAT_PRAM_RD_CMD, 2);
	} while (TempLong.Byte[1] != (FST_BYTE_NUM_ROUND_OUT / 4));

	Buffer[0] = COMM_SPI_READ;
	Buffer[1] = 0x00; // address of the read FIFO
	Buffer[2] = 0x00; // FIFO MsByte first
	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_RESET);

	// write command
	if (HAL_SPI_Transmit(ctx->spi, Buffer, 3, SPI_TIMEOUT_MAX) != HAL_OK) {
		// HAL_UART_Transmit(&huart3, (uint8_t *)"ERROR# HAL_SPI_Transmit\r\n", 25, HAL_MAX_DELAY);
	}

	if (HAL_SPI_Receive(ctx->spi, ctx->bOut->Byte, FST_BYTE_NUM_ROUND_OUT, SPI_TIMEOUT_MAX) != HAL_OK) {
		// HAL_UART_Transmit(&huart3, (uint8_t *)"ERROR# HAL_SPI_Receive\r\n", 24, HAL_MAX_DELAY);
	}

	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_SET);
	
	// second 
	do {                                                                  
		TempLong.Long = SPIReadRegisterDirect(ctx, ECAT_PRAM_WR_CMD, 2);
	} while (TempLong.Byte[1] < (SEC_BYTE_NUM_ROUND_OUT / 4));
	
	Buffer[0] = COMM_SPI_READ;
	Buffer[1] = 0x00; // address of the write fifo
	Buffer[2] = 0x00; // MsByte first (ECAT_PRAM_WR_DATA 020h-03Ch [ETHERCAT PROCESS RAM WRITE DATA FIFO])

	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(ctx->spi, Buffer, 3, SPI_TIMEOUT_MAX) != HAL_OK) {
	}
	if (HAL_SPI_Transmit(ctx->spi, &ctx->bIn->Byte[64], SEC_BYTE_NUM_ROUND_OUT, SPI_TIMEOUT_MAX) != HAL_OK) {
	}
	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_SET);
}
