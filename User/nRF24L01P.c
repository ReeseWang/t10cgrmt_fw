/******************************************************************************
 * @file			nRF24L01P.c
 * @author		Skywalker
 * @brief			STM32 user peripheral library for RF module nRF24L01P
 * @version		V1.1
 * @date			19. Nov 2013
 *
 * @par
 * Include this header in your project source file(s) and set correct include
 * path.
 ******************************************************************************/

/* Includes ----------------------------------------------------------------- */
#include "nRF24L01P.h"
#include <string.h>

/* Private definition ------------------------------------------------------- */
#define NRF_CS_HIGH(nrf)				GPIO_SetBits((nrf)->CS_GPIO_PORT, (nrf)->CS_GPIO_PIN)
#define NRF_CS_LOW(nrf)					GPIO_ResetBits((nrf)->CS_GPIO_PORT, (nrf)->CS_GPIO_PIN)

#define NRF_CE_HIGH(nrf)				GPIO_SetBits((nrf)->CE_GPIO_PORT, (nrf)->CE_GPIO_PIN)
#define NRF_CE_LOW(nrf)					GPIO_ResetBits((nrf)->CE_GPIO_PORT, (nrf)->CE_GPIO_PIN)

/* Global variables --------------------------------------------------------- */

void NRF_Delay(uint32_t t)
{
	while(t--)
		__NOP();
}

/**
  * @brief  GPIO & AFIO configuration
  * @note   Pointer to nRF24L01P_Object
  * @param  None
  * @retval None
  */
void NRF_LowLevel_Config(nRF24L01P_Object* nrf)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	
	/* Enable GPIO & AFIO Clocks */
	RCC_APB2PeriphClockCmd(nrf->SPI_GPIO_CLK | nrf->IRQ_GPIO_CLK | \
												RCC_APB2Periph_AFIO | nrf->CE_GPIO_CLK | \
												nrf->CS_GPIO_CLK, ENABLE);
	
	/* Configurate SPI GPIOs */
	GPIO_InitStructure.GPIO_Pin = nrf->SPI_SCK_PIN | nrf->SPI_MISO_PIN | nrf->SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(nrf->SPI_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configurate CS GPIO Pin */
	GPIO_InitStructure.GPIO_Pin = nrf->CS_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(nrf->CS_GPIO_PORT, &GPIO_InitStructure);
	
	NRF_CS_HIGH(nrf);
	
	/* Configurate CE GPIO Pin */
	GPIO_InitStructure.GPIO_Pin = nrf->CE_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(nrf->CE_GPIO_PORT, &GPIO_InitStructure);
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	/* Configurate IRQ GPIO Pin */
	GPIO_InitStructure.GPIO_Pin = nrf->IRQ_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(nrf->IRQ_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  External interrupt channel configuration for IRQ pin
	* @note   None
  * @param  Pointer to nRF24L01P_Object
  * @retval None
  */
void NRF_EXTI_Config(nRF24L01P_Object* nrf)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_StructInit(&EXTI_InitStructure);
	
	/* EXTI Line Config */
	GPIO_EXTILineConfig(nrf->IRQ_GPIO_PORT_SOURCE, nrf->IRQ_GPIO_PIN_SOURCE); 
	
  EXTI_InitStructure.EXTI_Line = nrf->EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 	
}

/**
  * @brief  Nested Vectored Interrupt Controller configuration for IRQ pin
  * @note   None
  * @param  Pointer to nRF24L01P_Object
  * @retval None
  */

void NRF_NVIC_Config(nRF24L01P_Object* nrf)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = nrf->EXTI_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = nrf->IRQChannelPreemptionPriority;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = nrf->IRQChannelSubPriority;	
	NVIC_Init(&NVIC_InitStruct);
}

/**
  * @brief  SPI peripheral configuration
  * @note   None
  * @param  Pointer to nRF24L01P_Object
  * @retval None
  */
void NRF_SPI_Config(nRF24L01P_Object* nrf)
{
	SPI_InitTypeDef SPI_InitStructure;
	SPI_StructInit(&SPI_InitStructure);

	/* Set nRF24L01 SPI Idle */
	NRF_CS_HIGH(nrf);
	
	/* Enable SPI clock */
	nrf->SPI_CLK_ENABLE_FUNCTION(nrf->SPI_PERIPH_CLK, ENABLE);
	
	/* Disable SPI */
	SPI_Cmd(nrf->SPIx, DISABLE);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = nrf->SPI_BaudRatePrescaler;	//Parameter check will be performed in SPI_Init()
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(nrf->SPIx, &SPI_InitStructure);
	
	/* Enable SPI */
	SPI_Cmd(nrf->SPIx, ENABLE);
}

/**
  * @brief  Basical SPI operation
  * @note   Private funtion. 
	*					nRF24L01 will automatically shift data into
	* 				MISO line while receiving bytes from MOSI line,
	* 				so no exclusive read opration is needed.
  * @param  Pointer to nRF24L01P_Object
  * @param  1-byte data to send.
  * @retval STATUS register in nRF24L01P.
  */
uint8_t NRF_SPI_SendByte(nRF24L01P_Object* nrf, uint8_t data)
{	
	/* Wait until last SPI transmit completes */
	while(SPI_I2S_GetFlagStatus(nrf->SPIx, SPI_I2S_FLAG_TXE) == RESET);
	
	/* Send one byte through SPI */
	SPI_I2S_SendData(nrf->SPIx, (uint16_t)data);
	
	/* Wait until SPI receives data */
	while(SPI_I2S_GetFlagStatus(nrf->SPIx, SPI_I2S_FLAG_RXNE) == RESET);
	
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(nrf->SPIx);
}

/**
  * @brief  Write an multi-byte register using NRF_WRITE_REG command
  * @note   
  * @param  Pointer to nRF24L01P_Object
  * @param  Register address, can be a value of @ref SPI(nRF24L01) register addresses.
	* @param	Buffer pointer. 
	* @param	Number of byte(s) to write.
  * @retval STATUS register in nRF24L01P.
  */
uint8_t NRF_WriteMultiByteReg(nRF24L01P_Object* nrf, uint8_t addr, uint8_t *pBuf, uint8_t numbyte)
{
	uint8_t status;
	uint8_t bytecnt;
	
	/* Reset CS pin to initiate an SPI transmission */
	NRF_CS_LOW(nrf);
	
	/* Send NRF_WRITE_REG command with register address */
	assert_param(IS_NRF_REGISTER(addr));
	status = NRF_SPI_SendByte(nrf, NRF_WRITE_REG | addr);
	
	/* Send remaining bytes */
	for(bytecnt = 0; bytecnt < numbyte; bytecnt++)
	{
		NRF_SPI_SendByte(nrf, *pBuf++);
	}
	
	/* Set CS pin to complete transmission */
	NRF_CS_HIGH(nrf);
	
	/* Return STATUS register */
	return status;
}

/**
  * @brief  Write a register using NRF_WRITE_REG command
  * @note   
	* @param  Pointer to nRF24L01P_Object
  * @param  Register address, can be a value of @ref SPI(nRF24L01) register addresses.
	* @param	Data. 
  * @retval STATUS register in nRF24L01P.
  */

uint8_t NRF_WriteReg(nRF24L01P_Object* nrf, uint8_t addr, uint8_t pData)
{
	uint8_t status;
	
	/* Reset CS pin to initiate an SPI transmission */
	NRF_CS_LOW(nrf);
	
	/* Send NRF_WRITE_REG command with register address */
	assert_param(IS_NRF_REGISTER(addr));
	status = NRF_SPI_SendByte(nrf, NRF_WRITE_REG | addr);
	
	/* Send remaining bytes */
	NRF_SPI_SendByte(nrf, pData);
	
	/* Set CS pin to complete transmission */
	NRF_CS_HIGH(nrf);
	
	/* Return STATUS register */
	return status;
}

/**
  * @brief  Read a multi-byte register using NRF_READ_REG command
  * @note   
	* @param  Pointer to nRF24L01P_Object
  * @param  Register address, can be a value of @ref SPI(nRF24L01) register addresses.
	* @param	Buffer pointer. 
	* @param	Number of byte(s) to read.
  * @retval STATUS register in nRF24L01P.
  */
uint8_t NRF_ReadMultiByteReg(nRF24L01P_Object* nrf, uint8_t addr, uint8_t *pBuf, uint8_t numbyte)
{
	uint8_t status;
	uint8_t bytecnt;
	
	/* Reset CS pin to initiate an SPI transmission */
	NRF_CS_LOW(nrf);
	
	/* Send NRF_READ_REG command with register address */
	assert_param(IS_NRF_REGISTER(addr));
	status = NRF_SPI_SendByte(nrf, NRF_READ_REG | addr);
	
	/* Read remaining bytes */
	for(bytecnt = 0; bytecnt < numbyte; bytecnt++)
	{
		pBuf[bytecnt] = NRF_SPI_SendByte(nrf, NRF_READ_REG);
	}
	
	/* Set CS pin to complete transmission */
	NRF_CS_HIGH(nrf);
	
	/* Return STATUS register */
	return status;
}

/**
  * @brief  Read a multi-byte register using NRF_READ_REG command
  * @note   
	* @param  Pointer to nRF24L01P_Object
  * @param  Register address, can be a value of @ref SPI(nRF24L01) register addresses.
	* @param	Buffer pointer. 
	* @param	Number of byte(s) to read.
  * @retval Requested register content.
  */
uint8_t NRF_ReadReg(nRF24L01P_Object* nrf, uint8_t addr)
{
	uint8_t data;
	
	/* Reset CS pin to initiate an SPI transmission */
	NRF_CS_LOW(nrf);
	
	/* Send NRF_READ_REG command with register address */
	assert_param(IS_NRF_REGISTER(addr));
	NRF_SPI_SendByte(nrf, NRF_READ_REG | addr);
	
	/* Read byte*/
	data = NRF_SPI_SendByte(nrf, NRF_NOP);

	/* Set CS pin to complete transmission */
	NRF_CS_HIGH(nrf);
	
	/* Return register content */
	return data;
}

uint8_t NRF_PowerUp(nRF24L01P_Object* nrf)
{
	uint8_t config;
	config = NRF_ReadReg(nrf, NRF_CONFIG) | NRF_PWR_UP;
	return NRF_WriteReg(nrf, NRF_CONFIG, config);
}

uint8_t NRF_PowerDown(nRF24L01P_Object* nrf)
{
	uint8_t config;
	config = NRF_ReadReg(nrf, NRF_CONFIG) & (~NRF_PWR_UP);
	return NRF_WriteReg(nrf, NRF_CONFIG, config);
}

/**
  * @brief  Clear RX cache
  * @note   
	* @param  Pointer to nRF24L01P_Object
  * @retval none
  */
void NRF_Flush_RX(nRF24L01P_Object* nrf)
{
	NRF_CS_LOW(nrf);
	NRF_SPI_SendByte(nrf, NRF_FLUSH_RX);
	NRF_CS_HIGH(nrf);
}

/**
  * @brief  Clear TX cache
  * @note   
	* @param  Pointer to nRF24L01P_Object
  * @retval none
  */
void NRF_Flush_TX(nRF24L01P_Object* nrf)
{
	NRF_CS_LOW(nrf);
	NRF_SPI_SendByte(nrf, NRF_FLUSH_TX);
	NRF_CS_HIGH(nrf);
}

/**
  * @brief  Set TX address
  * @note   Automatically decide number of bytes to send.
	* @param  Pointer to nRF24L01P_Object
  * @param  Address buffer pointer.
  * @retval STATUS register in nRF24L01P.
  */
uint8_t NRF_SetTXAddr(nRF24L01P_Object* nrf, uint8_t* Addr)
{
	memcpy(nrf->TX_Address, Addr, nrf->AddressFieldWidth);
	return NRF_WriteMultiByteReg(nrf, NRF_TX_ADDR, Addr, nrf->AddressFieldWidth);
}

/**
  * @brief  Set RX address
  * @note   Automatically decide number of bytes to send.
	* @param  Pointer to nRF24L01P_Object
  * @param  Datapipe, 0 ~ 5 integer
	* @param	Address buffer pointer.
  * @retval STATUS register in nRF24L01P.
  */
uint8_t NRF_SetRXAddr(nRF24L01P_Object* nrf, uint8_t Datapipe,	uint8_t* Addr)
{
	assert_param((Datapipe >= 0) && (Datapipe <= 5));
	switch(Datapipe){
		case 0:
			memcpy(nrf->RX_Address_P0, Addr, nrf->AddressFieldWidth);
			return NRF_WriteMultiByteReg(nrf, NRF_RX_ADDR_P0, Addr, nrf->AddressFieldWidth);
		case 1:
			memcpy(nrf->RX_Address_P1, Addr, nrf->AddressFieldWidth);
			return NRF_WriteMultiByteReg(nrf, NRF_RX_ADDR_P1, Addr, nrf->AddressFieldWidth);
		case 2:
			nrf->RX_Address_P2 = *Addr;
			return NRF_WriteReg(nrf, NRF_RX_ADDR_P2, *Addr);
		case 3:
			nrf->RX_Address_P3 = *Addr;
			return NRF_WriteReg(nrf, NRF_RX_ADDR_P3, *Addr);
		case 4:
			nrf->RX_Address_P4 = *Addr;
			return NRF_WriteReg(nrf, NRF_RX_ADDR_P4, *Addr);
		case 5:
			nrf->RX_Address_P5 = *Addr;
			return NRF_WriteReg(nrf, NRF_RX_ADDR_P5, *Addr);
		default:
			return 0;
	}
}

/**
  * @brief  Enable of disable interrupt masks.
  * @note   
	* @param  Pointer to nRF24L01P_Object
  * @param  Interrupt mask, can be combination of @ref NRF_InterruptMask.
	* @param	NewState. ENABLE of DISABLE 
  * @retval STATUS register in nRF24L01P.
  */
uint8_t NRF_INT_MASK_Cmd(nRF24L01P_Object* nrf, uint8_t NRF_InterruptMask,	FunctionalState NewState)
{
	uint8_t config; //Content of NRF_CONFIG register
	
	/* Parameter check */
	assert_param(IS_NRF_INT_MASK(NRF_InterruptMask));
	
	/* Read original NRF_CONFIG register */
	config = NRF_ReadReg(nrf, NRF_CONFIG);
	
	/* Compute the new value */
	if(NewState == ENABLE)
		config = config | NRF_InterruptMask;
	else
		config = config & ~NRF_InterruptMask;
	
	/* Write new value into NRF_CONFIG register */
	return NRF_WriteReg(nrf, NRF_CONFIG, config);	 
}

/**
  * @brief  Fills each NRF_InitStructure member with its default value.
  * @note   
  * @param  Pointer of NRF_InitTypeDef type.
  * @retval none.
  */
void NRF_ObjectInit(nRF24L01P_Object* nrf)
{
	uint8_t i;
	
	nrf->InterruptMask = NRF_INT_MASK_NONE;
	nrf->IntegratedCRC = NRF_CRC_ENABLE;
	nrf->CRCEncodingScheme = NRF_CRC_ENCODING_1BYTE;
	nrf->MasterMode = NRF_MASTER_MODE_TX;
	nrf->AutoACKEnabledPipe = NRF_DATAPIPE_ALL;
	nrf->RXAddressEnabledPipe = NRF_DATAPIPE_0 | NRF_DATAPIPE_1;
	nrf->AddressFieldWidth = 5;
	nrf->AutoRetransmitDelay = NRF_ARD_250;
	nrf->AutoRetransmitCount = 3;
	nrf->RFChannel = 2;
	nrf->ContinuousCarrierTransmit = NRF_CCT_DISABLE;
	nrf->PLL_Lock = NRF_PLL_LOCK_DISABLE;
	nrf->AirDataRate = NRF_RF_DR_2M;
	nrf->RF_Power = NRF_RF_PWR_0;
	nrf->DynamicPayloadEnabledPipe = NRF_DATAPIPE_NONE;
	nrf->DynamicPayloadLength = NRF_DPL_DISABLE;
	nrf->PayloadWithACK = NRF_ACK_PAY_DISABLE;
	nrf->DynamicACK = NRF_DYN_ACK_DISABLE;
	
	for(i = 0; i < 5; i++)
	{
		nrf->RX_Address_P0[i] = 0xE7;
		nrf->RX_Address_P1[i] = 0xC2;
		nrf->TX_Address[i] = 0xE7;
	}
	nrf->RX_Address_P2 = 0xC3;
	nrf->RX_Address_P3 = 0xC4;
	nrf->RX_Address_P4 = 0xC5;
	nrf->RX_Address_P5 = 0xC6;
}

/**
  * @brief  Initialize nRF24L01P registers
  * @note   Pointer to nRF24L01P_Object
  * @param  None
  * @retval None
  */
uint8_t NRF_Device_Config(nRF24L01P_Object* nrf)
{
	uint8_t bytecnt;
	uint8_t bufferA[7];	//for register NRF_CONFIG to NRF_RF_SETUP
	uint8_t bufferB[2]; //for NRF_DYNPD and NRF_FEATURE
	
	/* Parameter check */
	assert_param(IS_NRF_INT_MASK(nrf->InterruptMask) && \
							 IS_NRF_CRC(nrf->IntegratedCRC) && \
							 IS_NRF_CRC_ENCODING(nrf->CRCEncodingScheme) && \
							 IS_NRF_MASTER_MODE(nrf->MasterMode) && \
							 IS_NRF_DATAPIPE(nrf->AutoACKEnabledPipe) && \
							 IS_NRF_DATAPIPE(nrf->RXAddressEnabledPipe) && \
							 IS_NRF_AFW(nrf->AddressFieldWidth) && \
							 IS_NRF_ARD(nrf->AutoRetransmitDelay) && \
							 IS_NRF_ARC(nrf->AutoRetransmitCount) && \
							 IS_NRF_RF_CHN(nrf->RFChannel) && \
							 IS_NRF_CCT(nrf->ContinuousCarrierTransmit) && \
							 IS_NRF_PLL_LOCK(nrf->PLL_Lock) && \
							 IS_NRF_RF_DR(nrf->AirDataRate) && \
							 IS_NRF_RF_PWR(nrf->RF_Power) && \
							 IS_NRF_DATAPIPE(nrf->DynamicPayloadEnabledPipe) && \
							 IS_NRF_DPL(nrf->DynamicPayloadLength) && \
							 IS_NRF_ACK_PAY(nrf->PayloadWithACK) && \
							 IS_NRF_DYN_ACK(nrf->DynamicACK)); 
							 // Ensure desired data range
//	assert_param(((NRF_InitStructure->NRF_DynamicPayloadEnabledPipe) & \
								~(NRF_InitStructure->NRF_AutoACKEnabledPipe)) || \
								(!(NRF_InitStructure->NRF_DynamicPayloadLength) && \
								(NRF_InitStructure->NRF_DynamicPayloadEnabledPipe))); 
								// Prevent conflict between configurations:
								// Can not enable dynamic payload when AutoACK or NRF_DPL is disabled
	
	/* Assemble contents to send */
	bufferA[0] = nrf->InterruptMask | \
							 nrf->IntegratedCRC | \
							 nrf->CRCEncodingScheme | \
							 NRF_PWR_UP | \
							 nrf->MasterMode;
	bufferA[1] = nrf->AutoACKEnabledPipe;
	bufferA[2] = nrf->RXAddressEnabledPipe;
	bufferA[3] = nrf->AddressFieldWidth - 2;
	bufferA[4] = nrf->AutoRetransmitDelay | \
							 nrf->AutoRetransmitCount;
	bufferA[5] = nrf->RFChannel;
	bufferA[6] = nrf->ContinuousCarrierTransmit | \
							 nrf->PLL_Lock | \
							 nrf->AirDataRate | \
							 nrf->RF_Power;
	bufferB[0] = nrf->DynamicPayloadEnabledPipe;
	bufferB[1] = nrf->DynamicPayloadLength | \
							 nrf->PayloadWithACK | \
							 nrf->DynamicACK;
							 
	NRF_CE_LOW(nrf);
	/* Send register contents through SPI */
	for(bytecnt = 0; bytecnt < 7; bytecnt++)
	{
		NRF_WriteReg(nrf, NRF_CONFIG + bytecnt, *(bufferA + bytecnt));
	}
	NRF_WriteMultiByteReg(nrf, NRF_RX_ADDR_P0, nrf->RX_Address_P0, nrf->AddressFieldWidth);
	NRF_WriteMultiByteReg(nrf, NRF_RX_ADDR_P1, nrf->RX_Address_P1, nrf->AddressFieldWidth);
	NRF_WriteMultiByteReg(nrf, NRF_TX_ADDR, nrf->TX_Address, nrf->AddressFieldWidth);
	NRF_WriteReg(nrf, NRF_RX_ADDR_P2, nrf->RX_Address_P2);
	NRF_WriteReg(nrf, NRF_RX_ADDR_P3, nrf->RX_Address_P3);
	NRF_WriteReg(nrf, NRF_RX_ADDR_P4, nrf->RX_Address_P4);
	NRF_WriteReg(nrf, NRF_RX_ADDR_P5, nrf->RX_Address_P5);

	NRF_WriteReg(nrf, NRF_DYNPD, *bufferB);
	return NRF_WriteReg(nrf, NRF_FEATURE, *(bufferB + 1));
}

/**
  * @brief  Initializes the nRF24L01P according to the specified
  *         parameters in the NRF_InitStructure.
  * @note   
  * @param  Pointer of NRF_InitTypeDef type.
  * @retval STATUS register in nRF24L01P.
  */
uint8_t NRF_Init(nRF24L01P_Object* nrf)
{
	uint8_t status;
	
	NRF_LowLevel_Config(nrf);
	NRF_EXTI_Config(nrf);
	NRF_NVIC_Config(nrf);
	NRF_SPI_Config(nrf);
	status = NRF_Device_Config(nrf);
	
	if(nrf->MasterMode == NRF_MASTER_MODE_RX)
	{
		NRF_CE_HIGH(nrf);
	}
	
	return status;
}

/**
  * @brief  Read incoming payload width and datapipe
  * @note   
	* @param  Pointer to nRF24L01P_Object
  * @retval Payload width in bytes
  */
uint8_t NRF_SPI_GetReceivedPayloadInfo(nRF24L01P_Object* nrf)
{
	uint8_t status;
	
	/* Reset CS pin to initiate an SPI transmission */
	NRF_CS_LOW(nrf);
	
	/* Send NRF_R_RX_PL_WID command */
	status = NRF_SPI_SendByte(nrf, NRF_R_RX_PL_WID);
	
	/* Continue sending NRF_NOP command to retrive data */
	nrf->ReceivedPayload.PayloadWidth = NRF_SPI_SendByte(nrf, NRF_NOP);
	
	nrf->ReceivedPayload.Datapipe = (status & 0x0E) >> 1;
	
	/* Set CS pin to complete transmission */
	NRF_CS_HIGH(nrf);
	
	return status;
}

/**
  * @brief  Read incoming payload data using NRF_RD_RX_PLOAD command
  * @note   
	* @param  Pointer to nRF24L01P_Object
  * @param  Buffer pointer.
	* @param	Payload width.
  * @retval STATUS register in nRF24L01P.
  */
uint8_t NRF_ReadPayloadData(nRF24L01P_Object* nrf, uint8_t* pData, uint8_t width)
{
	uint8_t status;
	uint8_t bytecnt;
	
	/* Reset CS pin to initiate an SPI transmission */
	NRF_CS_LOW(nrf);
	
	/* Send read payload command */
	status = NRF_SPI_SendByte(nrf, NRF_RD_RX_PLOAD);
	
	/* Read remaining bytes */
	for(bytecnt = 0; bytecnt < width; bytecnt++)
	{
		pData[bytecnt] = NRF_SPI_SendByte(nrf, NRF_NOP);
	}
	
	/* Set CS pin to complete transmission */
	NRF_CS_HIGH(nrf);
	
	/* Return STATUS register */
	return status;
}

/**
  * @brief  Get payload.
  * @note   
  * @param  Pointer to nRF24L01P_Object.
  * @retval none.
  */
void NRF_GetPayload(nRF24L01P_Object* nrf)
{
	uint8_t bytecnt;
	
	/* Get payload width for following operations */
	NRF_SPI_GetReceivedPayloadInfo(nrf);
	
	/* Return if there is nothing in RX FIFO */
	if(nrf->ReceivedPayload.PayloadWidth == 0)
	{
		return;
	}
	/* Flush RX FIFO, clear RX_DR flag and return if there is an error occured */
	else if(nrf->ReceivedPayload.PayloadWidth > NRF_MAX_PAYLOAD_WIDTH)
	{		
		NRF_Flush_RX(nrf);
		NRF_WriteReg(nrf, NRF_STATUS, NRF_STATUS_RX_DR);
		return;
	}
	
	/* Read RX FIFO Data */
	NRF_ReadPayloadData(nrf, nrf->ReceivedPayload.Payload, nrf->ReceivedPayload.PayloadWidth);
	
	/* Fill remaining space with zero */
	for(bytecnt = nrf->ReceivedPayload.PayloadWidth; bytecnt < NRF_MAX_PAYLOAD_WIDTH; bytecnt++)
	{
		nrf->ReceivedPayload.Payload[bytecnt] = 0x00;
	}
}

/**
  * @brief  Send a payload.
  * @note   
  * @param  Pointer to nRF24L01P_Object.
  * @retval STATUS register in nRF24L01P.
  */
uint8_t NRF_SendPayload(nRF24L01P_Object* nrf)
{
	uint8_t bytecnt, status;
	
	/* Reset CE pin to enable register write */
	NRF_CE_LOW(nrf);
	
	/* Reset CS pin to initiate an SPI transmission */
	NRF_CS_LOW(nrf);
	
	/* Send NRF_WR_TX_PLOAD command */
	assert_param(nrf->PayloadToSend.PayloadWidth <= NRF_MAX_PAYLOAD_WIDTH);
	status = NRF_SPI_SendByte(nrf, NRF_WR_TX_PLOAD);	
	
	/* Send payload contents */
	for(bytecnt = 0; bytecnt < nrf->PayloadToSend.PayloadWidth; bytecnt++)
	{
		NRF_SPI_SendByte(nrf, nrf->PayloadToSend.Payload[bytecnt]);
	}	
	
	/* Set CS pin to complete transmission */
	NRF_CS_HIGH(nrf);

	/* Hold CE high for at least 10us to send payload */
	NRF_CE_HIGH(nrf);
	
	return status;
}

/**
  * @brief  Send a payload which does not require acknowledgement from RX device.
  * @note   
  * @param  Pointer to nRF24L01P_Object.
  * @retval STATUS register in nRF24L01P.
  */
uint8_t NRF_SendPayload_NOACK(nRF24L01P_Object* nrf)
{
	uint8_t bytecnt, status;
	
	/* Reset CE pin to enable register write */
	NRF_CE_LOW(nrf);
	
	/* Reset CS pin to initiate an SPI transmission */
	NRF_CS_LOW(nrf);
	
	/* Send NRF_W_TX_PAYLOAD_NOACK command */
	assert_param(nrf->PayloadToSend.PayloadWidth <= NRF_MAX_PAYLOAD_WIDTH);
	status = NRF_SPI_SendByte(nrf, NRF_W_TX_PAYLOAD_NOACK);	
	
	/* Send payload contents */
	for(bytecnt = 0; bytecnt < nrf->PayloadToSend.PayloadWidth; bytecnt++)
	{
		NRF_SPI_SendByte(nrf, nrf->PayloadToSend.Payload[bytecnt]);
	}	
	
	/* Set CS pin to complete transmission */
	NRF_CS_HIGH(nrf);

	/* Hold CE high for at least 10us to send payload */
	NRF_CE_HIGH(nrf);

	return status;
}

/**
  * @brief  Set ACK payload for RX device
  * @note   require NRF_ACK_PAY_ENABLE, NRF_DPL_ENABLE
						and enable DynamicPayload on a specific datapipe
  * @param  Pointer to nRF24L01P_Object.
  * @retval STATUS register in nRF24L01P.
  */
uint8_t NRF_SetACKPayload(nRF24L01P_Object* nrf)
{
	uint8_t bytecnt, status;
	
	if(nrf->MasterMode == NRF_MASTER_MODE_RX)
	{
		/* Reset CE pin to enable register write */
		NRF_CE_LOW(nrf);
		
		/* Reset CS pin to initiate an SPI transmission */
		NRF_CS_LOW(nrf);
		
		/* Send NRF_W_ACK_PAYLOAD command */
		assert_param(nrf->ACKPayload.PayloadWidth <= NRF_MAX_PAYLOAD_WIDTH);
		status = NRF_SPI_SendByte(nrf, NRF_W_ACK_PAYLOAD);	
		
		/* Send payload contents */
		for(bytecnt = 0; bytecnt < nrf->ACKPayload.PayloadWidth; bytecnt++)
		{
			NRF_SPI_SendByte(nrf, nrf->ACKPayload.Payload[bytecnt]);
		}	
		
		/* Set CS pin to complete transmission */
		NRF_CS_HIGH(nrf);

		NRF_CE_HIGH(nrf);
	}
	
	return status;
}

/**
  * @brief  Handle interrupt event
  * @note   Call this function, judge whether a payload is received 
						and processes received payload in EXTIx_IRQHandler()
  * @param  Pointer to nRF24L01P_Object
  * @retval STATUS register in nRF24L01P.
  */
uint8_t NRF_IRQ_Handler(nRF24L01P_Object* nrf)
{
	uint8_t status;
	
	NRF_CS_LOW(nrf);
	status = NRF_SPI_SendByte(nrf, NRF_NOP);
	NRF_CS_HIGH(nrf);
	
	if(EXTI_GetITStatus(nrf->EXTI_LINE) != RESET)
	{	
		nrf->ReceivedPayload.Datapipe = NRF_STATUS_RX_P_NO(status);
		if(status & NRF_STATUS_RX_DR)
		{
			NRF_GetPayload(nrf);
			NRF_WriteReg(nrf, NRF_STATUS, NRF_STATUS_RX_DR);
		}
		if(status & NRF_STATUS_TX_DS)
		{
			NRF_WriteReg(nrf, NRF_STATUS, NRF_STATUS_TX_DS);
		}
		if(status & NRF_STATUS_MAX_RT)
		{
			NRF_Flush_TX(nrf);
			NRF_WriteReg(nrf, NRF_STATUS, NRF_STATUS_MAX_RT);
		}
		EXTI_ClearITPendingBit(nrf->EXTI_LINE);
	}
	return status;
}
