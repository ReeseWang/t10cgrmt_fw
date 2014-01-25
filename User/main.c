/*     BYYB   SKYWORKS   TSINGHUA     */
/*	   USING  STM32  LIB V3.5.0		  */
/*	   2011.07.03    @C#410			  */
#include "stm32f10x.h"
#include "nRF24L01P.h"
#include "rc_transmitter.h"
#include <string.h>

nRF24L01P_Object TX;// RX;
RC_Tx_Object T10CG;

void Delay(u32 nCount);				  //ÑÓÊ±º¯Êý

void TriggerGPIOInit(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_5);
}

void HW_Config(void)
{
	uint8_t Address[5] = "MQuad";
	
	NRF_ObjectInit(&TX);
	
	TX.SPIx 										= SPI1;
	TX.SPI_CLK_ENABLE_FUNCTION 	= RCC_APB2PeriphClockCmd;
	TX.SPI_PERIPH_CLK 					= RCC_APB2Periph_SPI1;
	TX.SPI_GPIO_CLK							= RCC_APB2Periph_GPIOA;
	TX.SPI_GPIO_PORT						= GPIOA;
	
	TX.SPI_SCK_PIN							= GPIO_Pin_5;
	TX.SPI_MISO_PIN							= GPIO_Pin_6;
	TX.SPI_MOSI_PIN							= GPIO_Pin_7;
	TX.SPI_BaudRatePrescaler		= SPI_BaudRatePrescaler_2;
	
	TX.EXTI_LINE								= EXTI_Line9;
	TX.IRQ_GPIO_PORT_SOURCE			= GPIO_PortSourceGPIOC;
	TX.IRQ_GPIO_PIN_SOURCE			= GPIO_PinSource9;
	TX.EXTI_IRQn								= EXTI9_5_IRQn;
	TX.IRQChannelPreemptionPriority = 1;
	TX.IRQChannelSubPriority		= 0;
	
	TX.IRQ_GPIO_CLK							= RCC_APB2Periph_GPIOC;
	TX.IRQ_GPIO_PORT						= GPIOC;
	TX.IRQ_GPIO_PIN							= GPIO_Pin_9;
	
	TX.CE_GPIO_CLK							= RCC_APB2Periph_GPIOB;
	TX.CE_GPIO_PORT							= GPIOB;
	TX.CE_GPIO_PIN							= GPIO_Pin_0;
	
	TX.CS_GPIO_CLK							= RCC_APB2Periph_GPIOB;
	TX.CS_GPIO_PORT							= GPIOB;
	TX.CS_GPIO_PIN							= GPIO_Pin_1;
	
	TX.RFChannel = 91;
	TX.DynamicPayloadLength = NRF_DPL_ENABLE;
	TX.DynamicPayloadEnabledPipe = NRF_DATAPIPE_0;
	TX.AutoRetransmitDelay = NRF_ARD_3000;
	TX.DynamicACK = NRF_DYN_ACK_ENABLE;
	TX.PayloadWithACK = NRF_ACK_PAY_ENABLE;
	TX.MasterMode = NRF_MASTER_MODE_TX;
	TX.AirDataRate = NRF_RF_DR_2M;
	
	memcpy(TX.RX_Address_P0, Address, 5);
	memcpy(TX.TX_Address, Address, 5);
	
	NRF_Init(&TX);

	T10CG.GPIO_CLK								= RCC_APB2Periph_GPIOA;
	T10CG.GPIO_Port								= GPIOA;
	T10CG.GPIO_PIN								= GPIO_Pin_0;
	
	T10CG.TIM_CLK									= RCC_APB1Periph_TIM2;
	T10CG.TIMx										= TIM2;
	T10CG.TIM_Channel							= TIM_Channel_1;
	
	T10CG.TIM_IT									= TIM_IT_CC1;
	T10CG.TIM_IRQn								= TIM2_IRQn;
	T10CG.IRQChannelPreemptionPriority = 0;
	T10CG.IRQChannelSubPriority		= 0;
	
	T10CG.PPM_Polarity						= TIM_ICPolarity_Falling;
	T10CG.Max_Channel							= 8;
	
	RC_Tx_Init(&T10CG);
}

void EXTI9_5_IRQHandler()
{
	if(NRF_IRQ_Handler(&TX) & NRF_STATUS_MAX_RT)
		GPIOB->ODR ^= 0x0020;
}

void TIM2_IRQHandler(void)
{
	RC_Tx_IRQHandler(&T10CG);
}

int main()
{
	TriggerGPIOInit();
	GPIO_SetBits(GPIOC, GPIO_Pin_7);
	HW_Config();
	TX.PayloadToSend.PayloadWidth = 16;
	
	while(1)
	{
		while(T10CG.DataReady != READY);
		memcpy(TX.PayloadToSend.Payload, T10CG.PW, 16);
		NRF_SendPayload(&TX);
		while(T10CG.DataReady == READY);
	}
}

void Delay(u32 nCount)
{
   for(; nCount != 0; nCount--);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

