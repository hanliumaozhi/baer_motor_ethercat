#ifndef __LAN9252_H
#define __LAN9252_H

#include "stm32h7xx_hal.h"
#include "lan9252drv.h"

void init9252(spiCTX* ctx);

unsigned char main_task(spiCTX* ctx);

#endif
