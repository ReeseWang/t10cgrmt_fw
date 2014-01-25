#ifndef __NRF2_H
#define __NRF2_H

#include "stm32f10x.h"
#include "nRF24L01P.h"

void NRF2_Init(void);
uint8_t NRF2_SendPayload(NRF_PayloadDataType* NRF_Payload);
uint8_t NRF2_IRQ_Handler(NRF_PayloadDataType* ReceivedPayload);
uint8_t NRF2_ReadReg(uint8_t addr);
#endif
