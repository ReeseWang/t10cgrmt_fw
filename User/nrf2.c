#include "nrf2.h"


#define NRF2_CS_HIGH GPIO_SetBits(GPIOB, GPIO_Pin_6);  
#define NRF2_CS_LOW GPIO_ResetBits(GPIOB, GPIO_Pin_6);

#define NRF2_CE_HIGH GPIO_SetBits(GPIOB, GPIO_Pin_5);  
#define NRF2_CE_LOW GPIO_ResetBits(GPIOB, GPIO_Pin_5);

void NRF2_LowLevel_Config(void)				  //初始化LED
{	
	GPIO_InitTypeDef GPIO_InitStructure;  //定义GPIO结构体
	
	/* NRF2_CS */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);		 //开启GPIO B的时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;				     //CE CS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			 //将PB5 配置为通用推挽输出  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //IO口翻转速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //将上述设置赋予GPIOB
	
	NRF2_CS_LOW;
	NRF2_CE_LOW;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void NRF2_SPI_Init(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	SPI_StructInit(&SPI_InitStructure);

	/* Set nRF24L01 SPI Idle */
	NRF2_CS_HIGH;
	
	/* Enable SPI clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	/* Disable SPI */
	SPI_Cmd(SPI2, DISABLE);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;	//Parameter check will be performed in SPI_Init()
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	
	/* Enable SPI */
	SPI_Cmd(SPI2, ENABLE);
	
	NRF2_CS_HIGH;
}

void NRF2_EXTI_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_StructInit(&EXTI_InitStructure);
	
	/* IMPORTANT: Do Not Forget to Configure NVIC */
	
	/* EXTI Line Mode Config */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10); 
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 	
}

uint8_t NRF2_SPI_SendByte(uint8_t data)
{
	/* Wait until last SPI transmit completes */
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	
	/* Send one byte through SPI */
	SPI_I2S_SendData(SPI2, (uint16_t)data);
	
	/* Wait until SPI receives data */
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI2);
}

uint8_t NRF2_WriteReg(uint8_t addr, uint8_t pData)
{
	uint8_t status;
	
	/* Reset CS pin to initiate an SPI transmission */
	NRF2_CS_LOW;
	
	/* Send NRF_WRITE_REG command with register address */
	assert_param(IS_NRF_REGISTER(addr));
	status = NRF2_SPI_SendByte(NRF_WRITE_REG | addr);
	
	/* Send remaining bytes */
	NRF2_SPI_SendByte(pData);
	
	/* Set CS pin to complete transmission */
	NRF2_CS_HIGH;
	
	/* Return STATUS register */
	return status;
}

uint8_t NRF2_WriteMultiByteReg(uint8_t addr, uint8_t *pBuf, uint8_t numbyte)
{
	uint8_t status;
	uint8_t bytecnt;

	/* Reset CS pin to initiate an SPI transmission */
	NRF2_CS_LOW;
	
	/* Send NRF_WRITE_REG command with register address */
	assert_param(IS_NRF_REGISTER(addr));
	status = NRF2_SPI_SendByte(NRF_WRITE_REG | addr);
	
	/* Send remaining bytes */
	for(bytecnt = 0; bytecnt < numbyte; bytecnt++)
	{
		NRF2_SPI_SendByte(*pBuf++);
	}
	
	/* Set CS pin to complete transmission */
	NRF2_CS_HIGH;
	
	/* Return STATUS register */
	return status;
}

uint8_t NRF2_ReadReg(uint8_t addr)
{
	uint8_t data;
	
	/* Reset CS pin to initiate an SPI transmission */
	NRF2_CS_LOW;
	
	/* Send NRF_READ_REG command with register address */
	assert_param(IS_NRF_REGISTER(addr));
	NRF2_SPI_SendByte(NRF_READ_REG | addr);
	
	/* Read byte*/
	data = NRF2_SPI_SendByte(NRF_NOP);

	/* Set CS pin to complete transmission */
	NRF2_CS_HIGH;
	
	/* Return register content */
	return data;
}

uint8_t NRF2_PowerUp(void)
{
	uint8_t config;
	config = NRF2_ReadReg(NRF_CONFIG) | NRF_PWR_UP;
	return NRF2_WriteReg(NRF_CONFIG, config);
}

void NRF2_Init(void)
{
	char TXAddr[5] = "MQUAD";
	
	NRF2_LowLevel_Config();
	NRF2_SPI_Init();
	NRF2_EXTI_Init();
	
	NRF2_WriteReg(NRF_RF_CH, 85);
	NRF2_WriteReg(NRF_DYNPD, 0x01);
	NRF2_WriteReg(NRF_FEATURE, 0x04);
	NRF2_PowerUp();
	
	NRF2_WriteMultiByteReg(NRF_RX_ADDR_P0, (uint8_t *)TXAddr, 5);
	NRF2_WriteMultiByteReg(NRF_TX_ADDR, (uint8_t *)TXAddr, 5);	
}

uint8_t NRF2_SendPayload(NRF_PayloadDataType* NRF_Payload)
{
	uint8_t bytecnt, status;
	
	/* Reset CE pin to enable register write */
	NRF2_CE_LOW;
	
	/* Reset CS pin to initiate an SPI transmission */
	NRF2_CS_LOW;
	
	/* Send NRF_WR_TX_PLOAD command */
	assert_param(NRF_Payload->NRF_PayloadWidth <= NRF_MAX_PAYLOAD_WIDTH);
	status = NRF2_SPI_SendByte(NRF_WR_TX_PLOAD);	
	
	/* Send payload contents */
	for(bytecnt = 0; bytecnt < NRF_Payload->NRF_PayloadWidth; bytecnt++)
	{
		NRF2_SPI_SendByte(NRF_Payload->NRF_Payload[bytecnt]);
	}	
	
	/* Set CS pin to complete transmission */
	NRF2_CS_HIGH;

	/* Hold CE high for at least 10us to send payload */
	NRF2_CE_HIGH;
	NRF_Delay(200);
	NRF2_CE_LOW;
	
	return status;
}

uint8_t NRF2_IRQ_Handler(NRF_PayloadDataType* ReceivedPayload)
{
	uint8_t status;
	NRF_PayloadDataType Send;
	
	NRF2_CS_LOW;
	status = NRF2_SPI_SendByte(NRF_NOP);
	NRF2_CS_HIGH;
	
	ReceivedPayload->Datapipe = NRF_STATUS_RX_P_NO(status);
	if(status & NRF_STATUS_RX_DR)
	{
		NRF_GetPayload(ReceivedPayload);
		NRF2_WriteReg(NRF_STATUS, NRF_STATUS_RX_DR);
	}
	if(status & NRF_STATUS_TX_DS)
	{
		NRF2_WriteReg(NRF_STATUS, NRF_STATUS_TX_DS);
	}
	if(status & NRF_STATUS_MAX_RT)
	{
		NRF2_WriteReg(NRF_STATUS, NRF_STATUS_MAX_RT);
		GPIO_ResetBits(NRF_CS_GPIO_PORT, NRF_CS_GPIO_PIN);
		NRF_SPI_SendByte(NRF_NOP);
		GPIO_SetBits(NRF_CS_GPIO_PORT, NRF_CS_GPIO_PIN);
		Send.NRF_PayloadWidth = 1;
		Send.NRF_Payload[0] = 'A';
		NRF2_SendPayload(&Send);
	}	
	return status;
}


