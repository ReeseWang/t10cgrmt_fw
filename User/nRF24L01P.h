/******************************************************************************
 * @file			nRF24L01P.h
 * @author		Skywalker
 * @brief			STM32 user peripheral library for RF module nRF24L01P
 * @version		V1.1
 * @date			19. Nov 2013
 *
 * @par
 * Include this header in your project source file(s) and set correct include
 * path.
 ******************************************************************************/

/* Conditional compilation preventing duplicate definition ------------------ */
#ifndef __NRF24L01P_H
#define __NRF24L01P_H


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ----------------------------------------------------------------- */
#include "stm32f10x.h"
	 
/* Exported Define ---------------------------------------------------------- */

/** @defgroup nRF24L01P limits 
  * @{
  */
	
#define NRF_MAX_PAYLOAD_WIDTH	32
#define NRF_DATAPIPE_COUNT 6

/**
  * @}
  */

/** @defgroup SPI(nRF24L01) commands 
  * @{
  */

#define NRF_READ_REG        0x00  // Define read register command
#define NRF_WRITE_REG       0x20  // Define write to register command
#define NRF_RD_RX_PLOAD     0x61  // Define read RX payload command
#define NRF_WR_TX_PLOAD     0xA0  // Define send TX payload command
#define NRF_FLUSH_TX        0xE1  // Define flush TX registers command
#define NRF_FLUSH_RX        0xE2  // Define flush RX registers command
#define NRF_REUSE_TX_PL     0xE3  // Define reuse TX payload register command
#define NRF_R_RX_PL_WID			0x60	// Define read RX payload width for the top R_RX_PAYLOAD in the RX FIFO command
#define NRF_W_ACK_PAYLOAD		0xA8	// Define write payload to be transmitted together with ACK packet command
#define NRF_W_TX_PAYLOAD_NOACK	0xB0	// Define "send a TX payload which doesn't require ACK" command
#define NRF_NOP             0xFF  // Define No operation, might be used to read STATUS register
#define IS_NRF_COMMAND(COMMAND)			(((COMMAND) == NRF_READ_REG) || ((COMMAND) == NRF_WRITE_REG) || \
																		((COMMAND) == NRF_RD_RX_PLOAD) || ((COMMAND) == NRF_WR_TX_PLOAD) || \
																		((COMMAND) == NRF_FLUSH_TX) || ((COMMAND) == NRF_FLUSH_RX) || \
																		((COMMAND) == NRF_R_RX_PL_WID) || ((COMMAND) == NRF_W_ACK_PAYLOAD) || \
																		((COMMAND) == NRF_REUSE_TX_PL) || ((COMMAND) == NRF_NOP))

/**
  * @}
  */

/** @defgroup nRF24L01 register addresses 
  * @{
  */

#define NRF_CONFIG          0x00  // 'Config' register address
#define NRF_EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define NRF_EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define NRF_SETUP_AW        0x03  // 'Setup address width' register address
#define NRF_SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define NRF_RF_CH           0x05  // 'RF channel' register address
#define NRF_RF_SETUP        0x06  // 'RF setup' register address
#define NRF_STATUS          0x07  // 'Status' register address
#define NRF_OBSERVE_TX      0x08  // 'Observe TX' register address
#define NRF_RPD             0x09  // 'Carrier Detect' register address
#define NRF_RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define NRF_RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define NRF_RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define NRF_RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define NRF_RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define NRF_RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define NRF_TX_ADDR         0x10  // 'TX address' register address
#define NRF_RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define NRF_RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define NRF_RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define NRF_RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define NRF_RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define NRF_RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define NRF_FIFO_STATUS     0x17  // 'FIFO Status Register' register address
#define NRF_DYNPD						0x1C	// 'Enable dynamic payload length' register address
#define NRF_FEATURE					0x1D	// 'Feature Register' register address
#define IS_NRF_REGISTER(REG)			(((REG) <= 0x17) || (((REG) == 0x1C) || ((REG) == 0x1D)))

/**
  * @}
  */

/** @defgroup Macros represent variables in NRF_InitTypeDef
  * @{
  */

/* NRF_InterruptMask */
#define NRF_INT_MASK_RX_DR			0x40
#define NRF_INT_MASK_TX_DS			0x20
#define NRF_INT_MASK_MAX_RT			0x10
#define NRF_INT_MASK_NONE				0x00
#define NRF_INT_MASK_ALL				0x00
#define IS_NRF_INT_MASK(MASK)   (!((MASK) & 0x8F))

/* NRF_CRC */
#define NRF_CRC_ENABLE			0x08
#define NRF_CRC_DISABLE			0x00
#define IS_NRF_CRC(PARA)			(!((PARA) & 0xF7))

/* NRF_CRCEncodingScheme */
#define NRF_CRC_ENCODING_1BYTE		0x00
#define NRF_CRC_ENCODING_2BYTES		0x04
#define IS_NRF_CRC_ENCODING(PARA)	(!((PARA) & 0xFB)) 

/* NRF_Power */
#define NRF_PWR_UP					0x02
#define NRF_PWR_DOWN				0x00
#define IS_NRF_PWR(PARA)	(!((PARA) & 0xFD == 0))

/* NRF_MasterMode */
#define NRF_MASTER_MODE_RX	0x01
#define NRF_MASTER_MODE_TX	0x00
#define IS_NRF_MASTER_MODE(PARA)	(!((PARA) & 0xFE))

/* NRF_AddressFieldWidth */
#define IS_NRF_AFW(PARA)		((3 <= (PARA)) && (5 >= (PARA));

/* NRF_AutoRetransmitDelay(us) */
#define NRF_ARD_250					0x00
#define NRF_ARD_500					0x10
#define NRF_ARD_750					0x20
#define NRF_ARD_1000				0x30
#define NRF_ARD_1250				0x40
#define NRF_ARD_1500				0x50
#define NRF_ARD_1750				0x60
#define NRF_ARD_2000				0x70
#define NRF_ARD_2250				0x80
#define NRF_ARD_2500				0x90
#define NRF_ARD_2750				0xA0
#define NRF_ARD_3000				0xB0
#define NRF_ARD_3250				0xC0
#define NRF_ARD_3500				0xD0
#define NRF_ARD_3750				0xE0
#define NRF_ARD_4000				0xF0
#define IS_NRF_ARD(PARA)	(!((PARA) & 0x0F))
#define IS_NRF_ARC(PARA)	((PARA) <= 0x0F)
#define IS_NRF_RF_CHN(PARA)	((PARA) <= 0x7F)

/* NRF_ContinuousCarrierTransmit */
#define NRF_CCT_ENABLE			0x80
#define NRF_CCT_DISABLE			0x00
#define IS_NRF_CCT(PARA)	(!((PARA) & 0x7F))

/* NRF_PLL_Lock */
#define NRF_PLL_LOCK_ENABLE		0x10
#define NRF_PLL_LOCK_DISABLE	0x00
#define IS_NRF_PLL_LOCK(PARA)	(!((PARA) & 0xEF))

/* NRF_AirDataRate */
#define NRF_RF_DR_1M				0x00
#define NRF_RF_DR_2M				0x08
#define NRF_RF_DR_250K			0x28

/* Comment next line if using nRF24L01 */
#define __NRF24L01_PLUS

#ifdef __NRF24L01_PLUS
	#define IS_NRF_RF_DR(PARA)	(((PARA) == NRF_RF_DR_1M) || \
															((PARA) == NRF_RF_DR_250K) || \
															((PARA) == NRF_RF_DR_2M))
	#else
	#define IS_NRF_RF_DR(PARA)	(((PARA) == NRF_RF_DR_1M) || \
															((PARA) == NRF_RF_DR_2M))
#endif

/* NRF_RF_Power */
#define NRF_RF_PWR_18				0x00	/* -18dBm */
#define NRF_RF_PWR_12				0x02	/* -12dBm */
#define NRF_RF_PWR_6				0x04	/* -6dBm */
#define NRF_RF_PWR_0				0x06	/* 0dBm */
#define IS_NRF_RF_PWR(PARA)	(((PARA) == NRF_RF_PWR_18) || \
														((PARA) == NRF_RF_PWR_12) || \
														((PARA) == NRF_RF_PWR_6) || \
														((PARA) == NRF_RF_PWR_0))

/* STATUS */
#define NRF_STATUS_RX_DR					0x40
#define NRF_STATUS_TX_DS					0x20
#define NRF_STATUS_MAX_RT					0x10
#define NRF_STATUS_RX_P_NO(STATUS) (((STATUS) & 0x0E) >> 1)
#define NRF_STATUS_TX_FULL				0x01

/* FIFO_STATUS */
#define NRF_FIFOSTATUS_TX_REUSE			0x40
#define NRF_FIFOSTATUS_TX_FULL			0x20
#define NRF_FIFOSTATUS_TX_EMPTY			0x10
#define NRF_FIFOSTATUS_RX_FULL			0x02
#define NRF_FIFOSTATUS_RX_EMPTY			0x01

/* NRF_DynamicPayloadLength */
#define NRF_DPL_ENABLE			0x04
#define NRF_DPL_DISABLE			0x00
#define IS_NRF_DPL(PARA)	(!((PARA) & 0xFB)) 

/* NRF_PayloadWithACK */
#define NRF_ACK_PAY_ENABLE			0x02
#define NRF_ACK_PAY_DISABLE			0x00
#define IS_NRF_ACK_PAY(PARA)	(!((PARA) & 0xFD))

/* NRF_DynamicACK */
#define NRF_DYN_ACK_ENABLE			0x01
#define NRF_DYN_ACK_DISABLE			0x00
#define IS_NRF_DYN_ACK(PARA)	(!((PARA) & 0xFE))

/**
  * @}
  */
/* NRF_Datapipe */
/* NRF_DynamicPayloadEnabledPipe */
/* NRF_RXAddressEnabledPipe */
/* NRF_AutoACKEnabledPipe */
#define NRF_DATAPIPE_0		0x01
#define NRF_DATAPIPE_1		0x02
#define NRF_DATAPIPE_2		0x04
#define NRF_DATAPIPE_3		0x08
#define NRF_DATAPIPE_4		0x10
#define NRF_DATAPIPE_5		0x20

#define NRF_DATAPIPE_NONE	0x00
#define NRF_DATAPIPE_ALL	0x3f
#define IS_NRF_DATAPIPE(DATAPIPE)	(!((DATAPIPE) & 0xC0))

/* Structure type definitions ----------------------------------------------- */

/** 
  * @brief  nRF24L01P payload definition  
  */

typedef struct
{
	uint8_t Datapipe;													/*!< Data pipe number of the received payload (integer). */
	uint8_t PayloadWidth;											/*!< Payload width in bytes. Larger than NRF_MAX_PAYLOAD_WIDTH
																										 only if an error occured in RF transmission */
	
	uint8_t Payload[NRF_MAX_PAYLOAD_WIDTH];		/*!< Payload data. Constant size array. */
} NRF_ReceivedPayloadType;

typedef struct
{
	uint8_t PayloadWidth;											/*!< Payload width in bytes. Larger than NRF_MAX_PAYLOAD_WIDTH
																										 only if an error occured in RF transmission */
	
	uint8_t Payload[NRF_MAX_PAYLOAD_WIDTH];		/*!< Payload data. Constant size array. */
} NRF_PayloadToSendType;

/** 
  * @brief  nRF24L01P Object definition  
  */

typedef struct
{
	/** @membergroup Hardware Abstract Layer 
  * @{
  */
	SPI_TypeDef* 		SPIx;														/*!< x can be 1, 2 or 3 to select the SPI peripheral.*/
	void 						(*SPI_CLK_ENABLE_FUNCTION)();
	uint32_t				SPI_PERIPH_CLK;
	uint32_t 				SPI_GPIO_CLK;
	GPIO_TypeDef*		SPI_GPIO_PORT;
	uint16_t				SPI_SCK_PIN;
	uint16_t				SPI_MISO_PIN;
	uint16_t				SPI_MOSI_PIN;
	uint16_t				SPI_BaudRatePrescaler;
	
	uint32_t				EXTI_LINE;
	uint8_t					IRQ_GPIO_PORT_SOURCE;
	uint8_t					IRQ_GPIO_PIN_SOURCE;
	IRQn_Type				EXTI_IRQn;
	uint8_t					IRQChannelPreemptionPriority;  	/*!< Specifies the pre-emption priority for the IRQ channel
                                                   specified in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table */
  uint8_t 				IRQChannelSubPriority;         	/*!< Specifies the subpriority level for the IRQ channel specified
                                                   in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table */
	
	uint32_t				IRQ_GPIO_CLK;
	GPIO_TypeDef*		IRQ_GPIO_PORT;
	uint16_t				IRQ_GPIO_PIN;
	
	uint32_t 				CE_GPIO_CLK;
	GPIO_TypeDef*		CE_GPIO_PORT;
	uint16_t				CE_GPIO_PIN;
	
	uint32_t 				CS_GPIO_CLK;
	GPIO_TypeDef*		CS_GPIO_PORT;
	uint16_t				CS_GPIO_PIN;
	
	/**
  * @}	Hardware Abstract Layer
  */
	
	/** @membergroup Device Configuration
  * @{
  */
	
	/* CONFIG 0x00 */
	uint8_t InterruptMask;
	uint8_t IntegratedCRC;
	uint8_t CRCEncodingScheme;
	uint8_t MasterMode;
	
	/* EN_AA 0x01 */
	uint8_t AutoACKEnabledPipe;
	
	/* EN_RXADDR 0x02 */
	uint8_t RXAddressEnabledPipe;
	
	/* SETUP_AW 0x03 */
	uint8_t AddressFieldWidth;	//3, 4 or 5.
	
	/* SETUP_RETR 0x04 */
	uint8_t AutoRetransmitDelay;
	uint8_t AutoRetransmitCount;
	
	/* RF_CH 0x05 */
	uint8_t RFChannel;
	
	/* RF_SETUP 0x06 */
	uint8_t ContinuousCarrierTransmit;
	uint8_t PLL_Lock;
	uint8_t AirDataRate;
	uint8_t RF_Power;
	
	/* RX_ADDR_P0 ~ P5 0x0A ~ 0x0F */
	uint8_t RX_Address_P0[5];
	uint8_t RX_Address_P1[5];
	uint8_t RX_Address_P2;
	uint8_t RX_Address_P3;
	uint8_t RX_Address_P4;
	uint8_t RX_Address_P5;
	
	/* TX_ADDR 0x10 */
	uint8_t TX_Address[5];
	
	/* DYNPD 0x1C */
	uint8_t DynamicPayloadEnabledPipe;
	
	/* FEATURE 0x1D */
	uint8_t DynamicPayloadLength;
	uint8_t PayloadWithACK;
	uint8_t DynamicACK;
	
	/**
  * @}	Device Configuration
  */
	
	/** @membergroup Payload
  * @{
  */
	
	NRF_ReceivedPayloadType ReceivedPayload;
	NRF_PayloadToSendType		PayloadToSend;
	NRF_PayloadToSendType		ACKPayload;
} nRF24L01P_Object;

/* Exported functions ------------------------------------------------------- */

void NRF_LowLevel_Config(nRF24L01P_Object* nrf);
void NRF_Flush_RX(nRF24L01P_Object* nrf);
void NRF_Flush_TX(nRF24L01P_Object* nrf);
uint8_t NRF_SetTXAddr(nRF24L01P_Object* nrf, uint8_t* Addr);
uint8_t NRF_ReadTXAddr(nRF24L01P_Object* nrf, uint8_t* Addr);
uint8_t NRF_SetRXAddr(nRF24L01P_Object* nrf, uint8_t Datapipe, uint8_t* Addr);
uint8_t NRF_ReadReg(nRF24L01P_Object* nrf, uint8_t addr);
uint8_t NRF_INT_MASK_Cmd(nRF24L01P_Object* nrf, uint8_t InterruptMask, FunctionalState NewState);
void NRF_ObjectInit(nRF24L01P_Object* nrf);
uint8_t NRF_Init(nRF24L01P_Object* nrf);
void NRF_GetPayload(nRF24L01P_Object* nrf);
uint8_t NRF_SendPayload(nRF24L01P_Object* nrf);
uint8_t NRF_SendPayload_NOACK(nRF24L01P_Object* nrf);
uint8_t NRF_SetACKPayload(nRF24L01P_Object* nrf);
uint8_t NRF_IRQ_Handler(nRF24L01P_Object* nrf);

#ifdef __cplusplus
	}
#endif /* __cplusplus */

#endif /* __NRF24L01P_H */
