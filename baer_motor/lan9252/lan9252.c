#include "lan9252.h"
#include "stdbool.h"

void init9252(spiCTX* ctx) {
	char buffer[64] = { 0 };
	ULONG TempLong;

	SPIWriteRegisterDirect(ctx, RESET_CTL, DIGITAL_RST);

	unsigned short i = 0;
	do {
		TempLong.Long = SPIReadRegisterDirect(ctx, RESET_CTL, 4);
		sprintf(buffer, "SPI# (RESET_CTL) 0x%08x \r\n", TempLong.Long);
		HAL_UART_Transmit(ctx->uart, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
		i++;
	} while (((TempLong.Byte[0] & 0x01) != 0x00) && (i != Tout));
	
	if (i == Tout) {
		ctx->deviceInitiated = 0x0;
		return;
	}

	i = 0;
	do {
		TempLong.Long = SPIReadRegisterDirect(ctx, BYTE_TEST, 4);
		sprintf(buffer, "SPI# (BYTE_TEST) 0x%08x \r\n", TempLong.Long);
		HAL_UART_Transmit(ctx->uart, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
		i++;
	} while ((TempLong.Long != 0x87654321) && (i != Tout));

	if (i == Tout) {
		ctx->deviceInitiated = 0x0;
		return;
	}

	i = 0;
	do {
		TempLong.Long = SPIReadRegisterDirect(ctx, HW_CFG, 4);
		sprintf(buffer, "SPI# (HW_CFG) 0x%08x \r\n", TempLong.Long);
		HAL_UART_Transmit(ctx->uart, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
		i++;
	} while (((TempLong.Byte[3] & READY) == 0) && (i != Tout));

	if (i == Tout) {
		ctx->deviceInitiated = 0x0;
		return;
	}

    //SPIWriteRegisterIndirect(ctx, 0x00000004, AL_EVENT_MASK, 4);
	//sprintf(buffer, "DC_SYNC\r\n");
	
    /*SPIWriteRegisterIndirect(ctx, 0x00000111, AL_EVENT_MASK, 4);
	sprintf(buffer, "SM_SYNC\r\n");
	HAL_UART_Transmit(ctx->uart, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
	
	SPIWriteRegisterDirect(ctx, IRQ_CFG, 0x00000111); // set LAN9252 interrupt pin driver  
                                                          // as push-pull active high
                                                          // (On the EasyCAT shield board the IRQ pin
                                                          // is inverted by a mosfet, so Arduino                                                        
                                                          // receives an active low signal)
	
	sprintf(buffer, "SS\r\n");
	HAL_UART_Transmit(ctx->uart, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);
                                                                        
	SPIWriteRegisterDirect(ctx, INT_EN, 0x00000001); // enable LAN9252 interrupt    */  

	ctx->deviceInitiated = 0x1;
}

unsigned char main_task(spiCTX* ctx)
{
	bool WatchDog = true;
	bool Operational = false; 
	unsigned char i;
	ULONG TempLong; 
	unsigned char Status;  
 
  
	TempLong.Long = SPIReadRegisterIndirect(ctx, WDOG_STATUS, 1); // read watchdog status
	if ((TempLong.Byte[0] & 0x01) == 0x01)                    //
	WatchDog = false; // set/reset the corrisponding flag
	else                                                      //
	  WatchDog = true; //

    
	TempLong.Long = SPIReadRegisterIndirect(ctx, AL_STATUS, 1); // read the EtherCAT State Machine status
	Status = TempLong.Byte[0] & 0x0F; //
	if (Status == ESM_OP)                                     // to see if we are in operational state
	Operational = true; //
	else                                                      // set/reset the corrisponding flag
	  Operational = false; //    


	                                                          //--- process data transfert ----------
	                                                          //                                                        
	if (WatchDog | !Operational)                              // if watchdog is active or we are 
	{
		// not in operational state, reset 
		for (i = 0; i < TOT_BYTE_NUM_ROUND_OUT; i++)                   // the output buffer
		ctx->bOut->Byte[i] = 0; //

		/*                                                          // debug
		    if (!Operational)                                       //
		      printf("Not operational\n");                    		//
		 if (WatchDog)                                           //    
		   printf("WatchDog\n");                           		//  
		   */                                                          //
	}
	else                                                      
	{                                                         
		SPIReadProcRamFifo(ctx); // otherwise transfer process data from 
	}                                                         // the EtherCAT core to the output buffer  
                 
	SPIWriteProcRamFifo(ctx); // we always transfer process data from
	                                                          // the input buffer to the EtherCAT core  

	if (WatchDog)                                             // return the status of the State Machine      
	{
		// and of the watchdog
		Status |= 0x80; //
	}                                                         //
	return Status;                                             //   
}