#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_adc.h"

#define Ts_Cal1                      (*((uint16_t*)0x1FFFF7B8))
#define Ts_Cal2                      (*((uint16_t*)0x1FFFF7C2))

uint8_t SPI_transceive(uint8_t data);
static void manualCalibration(void);
int16_t RF_getRSSI(void);
uint8_t RF_init();
void RF_send(uint8_t length, uint8_t data[]);
uint8_t CC112X_setting(uint16_t addr, uint8_t data);
void USART_Send_Preemptive(volatile char *str);
void Load_Buffer(uint8_t INPUTbuff[],uint8_t buff_end,volatile char *str);
void processPacket(uint8_t length, uint8_t data[], uint8_t srcIf);

GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
USART_ClockInitTypeDef USART_ClockInitStruct;
SPI_InitTypeDef SPI_InitStructure;
EXTI_InitTypeDef   EXTI_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
ADC_InitTypeDef     ADC_InitStructure;

/*
 * HW description
 * A0	- PA_EN		(RF_AMP)	(HIGH=TX  ON)
 * A1	- LNA_EN	(RF_AMP)	(HIGH=RX  ON)
 * A2	- USART2_TX (RS485)
 * A3	- USART2_RX (RS485)
 * A4	- RF_CS/				(HIGH=COM OFF)
 * A5	- SPI1_SCK
 * A6	- SPI1_MISO
 * A7	- SPI1_MOSI
 * A8	- RS485_DIR		0=rx 1=tx
 * A9 	- USART1_TX (USB)
 * A10 	- USART1_RX (USB)
 * A11	- CTS/		(USB)
 * A12	- RTS/		(USB)
 * B0	- RF_GPIO2
 * B1	- RF_GPIO3
 * B2	- RF_RST/				(HIGH=DEV OFF)
 * B3	- LED					(HIGH=ON)
 * F0	- HGM		(RF_AMP)	(HIGH=ON)
 */

const int16_t FREQOFF_DEV[] = {0,0,0};
float Avg_Slope;

// global temp var
uint16_t i,j,k;
uint8_t  temp;
uint8_t lastPulseSrc;
uint8_t watchDogVariable;

// global var
 int16_t MCU_TEMP;

//global buffers
uint8_t USB_RX_BUFFER[128], USB_RX_INDEX, USB_RX_NUM;
uint8_t USB_TX_BUFFER[128], USB_TX_INDEX, USB_TX_NUM;

uint8_t RS485_RX_BUFFER[128], RS485_RX_INDEX, RS485_RX_NUM;
uint8_t RS485_TX_BUFFER[128], RS485_TX_INDEX, RS485_TX_NUM;

uint8_t RF_RX_BUFFER[128], RF_RX_INDEX, RF_RX_NUM;
uint8_t RF_TX_BUFFER[128], RF_TX_INDEX, RF_TX_NUM;

//	RF_settings
uint8_t	ADDR=6;// = ((uint32_t *)0x1FFFF7AC)[0];
 int16_t TEMPERATURE;
uint32_t FREQ;
int16_t FREQOFF;
uint32_t BAUD;
uint8_t  CHBW;
uint8_t  TXPW;
uint8_t  FMOD;
int16_t  RSSI;
uint8_t RF_stat = 0,
		RF_flag = 0;	// 0x01=packet received!

const char *str = "*\r\n";	// greeting after start-up


int main(void)
{
	Avg_Slope = ((float)(Ts_Cal1 - Ts_Cal2)) / (110 - 30);
	//ADDR = ((uint32_t *)0x1FFFF7AC)[0];

	RCC_HSICmd(ENABLE);
	/* Wait till HSI is ready                                               */
	while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
	/* PLLCLK = 8MHz/2 * 12 = 48 MHz                                           */
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_6);
	/* Enable PLL                     */
	RCC_PLLCmd (ENABLE);
	/* Wait till PLL is ready                                               */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	/* Select PLL as system clock source                                    */
	RCC_SYSCLKConfig (RCC_SYSCLKSource_PLLCLK);
	/* Wait till PLL is used as system clock source                         */
	while (RCC_GetSYSCLKSource() != 0x08);
	/* Set system dividers */
	RCC_HCLKConfig  (RCC_SYSCLK_Div1);   /* HCLK   = SYSCLK                */
	RCC_PCLKConfig  (RCC_HCLK_Div1);     /* PCLK2  = HCLK                  */
//	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);    /* ADCCLK = PCLK/4               */
	RCC_USARTCLKConfig(RCC_USART1CLK_PCLK);

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA |RCC_AHBPeriph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 | RCC_APB1Periph_USART2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG | RCC_APB2ENR_SPI1EN | RCC_APB2Periph_USART1 /*| RCC_APB2Periph_ADC1*/, ENABLE);

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOF, &GPIO_InitStructure);
		GPIO_WriteBit(GPIOF, GPIO_Pin_0, 1);				// HGM
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, DISABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;		// same for all below
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1  | GPIO_Pin_4 | GPIO_Pin_8 ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;
		SPI_Init(SPI1, &SPI_InitStructure);
		SPI_Cmd(SPI1, ENABLE);
		SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);

		// USB
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);		// needs to be before the init
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);		//
		USART_InitStructure.USART_BaudRate = 115200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART1, &USART_InitStructure);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);
		USART_MSBFirstCmd(USART1,  DISABLE);
		USART_Cmd(USART1, ENABLE);

		// RS485
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);		//
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);		//
		GPIO_WriteBit(GPIOA, GPIO_Pin_8, 0);					// RS485_DIR 0=rx
		USART_InitStructure.USART_BaudRate = 9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART2, &USART_InitStructure);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		USART_ITConfig(USART2, USART_IT_TC, ENABLE);
		USART_MSBFirstCmd(USART2,  DISABLE);
		USART_Cmd(USART2, ENABLE);

		TIM_DeInit(TIM6);		// riadenie DAC
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;		// 48MHz / 4 = 12MHz		24Mhz/4=6M
		TIM_TimeBaseStructure.TIM_Prescaler = 	65535;				// / 65k = 180Hz			6M/65k=95
		TIM_TimeBaseStructure.TIM_Period = 	1000;					// /500 => T=~3s			1000/95	=>	10s
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	//	TIM_Cmd(TIM6, ENABLE);

/*
		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
		ADC_Init(ADC1, &ADC_InitStructure);
		ADC_TempSensorCmd(ENABLE);
		ADC_ChannelConfig(ADC1, ADC_Channel_6, ADC_SampleTime_239_5Cycles);
		ADC_ChannelConfig(ADC1, ADC_Channel_TempSensor , ADC_SampleTime_239_5Cycles);
		ADC_WaitModeCmd(ADC1, ENABLE);
*/

		// GPIO default states
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1);	// RF_CS/
		GPIO_WriteBit(GPIOB, GPIO_Pin_2, 1);	// RF_RST/

//		__disable_irq();

		/* Connect EXTI0 Line to PB0 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);
		/* */
		EXTI_ClearITPendingBit(EXTI_Line0);
		EXTI_InitStructure.EXTI_Line = EXTI_Line0;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		/*  */
		EXTI_ClearITPendingBit(EXTI_Line1);
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		// Enable and set EXTI Interrupt
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		/*USART1*/ //USB
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		/*USART2*/ //RS485
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		/*TIM6 IRQ*/
		NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);



//		GPIO_WriteBit(GPIOA, GPIO_Pin_0, 1);	// PA_EN

		GPIO_WriteBit(GPIOB, GPIO_Pin_3, 1);	// LED

		while(*str)	// blocking USART-to-USB send to wait for RF power stabilisation
		{
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
			USART_SendData(USART1,*str);
			str++;
		}

		RF_flag=0; RSSI=0;
//		__enable_irq();

    	GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0);	// RF_CS/
    	while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)==1);		// wait till RF rdy
    	RF_init();
    	RF_stat=SPI_transceive(0x3D);
    	if (RF_stat==0x0F)
    		GPIO_WriteBit(GPIOB, GPIO_Pin_3, 0);	// LED
//    	SPI_transceive(0x37);	// AFC strobe

    	for(i=0;i<120;i++)
    	{	USB_TX_BUFFER[i] = i;
    	RS485_TX_BUFFER[i]=i;}
    	USB_TX_NUM=0; RS485_TX_NUM=0;
    	USB_RX_INDEX=0;

    	GPIO_WriteBit(GPIOA, GPIO_Pin_1, 1);	// LNA_EN -> RX rdy
		// SFRX - RX FLUSH
		SPI_transceive(0x3A);
		// set to receive
		SPI_transceive(0x34);
		for(i=0;i<1000;i++);	// RX settling delay	- trigger autocalibrate ?
		RF_stat=SPI_transceive(0x3D);

		TIM_Cmd(TIM6, ENABLE);	// anti-stuck watchdog	- consider commenting-away while debugging


    while(1)
    {

    	if (USB_RX_NUM!=0)		// 1 - USB data received	(handling of fast UART - termination character)
		{
    		while (USB_RX_NUM > USB_RX_INDEX)
    		{
				if (USB_RX_BUFFER[USB_RX_INDEX++]=='\r')		// "enter" found
				{
					processPacket(USB_RX_INDEX-1, USB_RX_BUFFER,1);	//pkt closed ... send!
					if (USB_RX_NUM > USB_RX_INDEX)	// more data received while processing ?
					{

					}
					else							// if no more data
					{
						USB_RX_NUM=0; USB_RX_INDEX=0;
					}
				}
    		}
		}
    	else if (RS485_RX_NUM!=0)		// 2 - RS485 data received	(handling of slow UART - message length)
		{
    		i=RS485_RX_INDEX;
    		if ( (i+RS485_RX_BUFFER[RS485_RX_INDEX]-48) < RS485_RX_NUM )	// -48 for ascii input
    		{
				processPacket(RS485_RX_NUM-RS485_RX_INDEX-1, RS485_RX_BUFFER+1,2);	//pkt closed ... send!
				RS485_RX_NUM=0; RS485_RX_INDEX=0;
			}
		}
    	else if (RF_flag==1) 	// 3 - RF packet ready
    	{
    		//j=SPI_transceive(0x3D);		// chceck state - debug

    		// TODO improve?

    		__disable_irq();
			SPI_transceive(0xAF);	SPI_transceive(0xD7);	temp=SPI_transceive(0xFF);	// FIFO_RX_NUM - data rdy?
			__enable_irq();

			if (temp==0)													// if not, report lost packet		-- change?
			{
				USART_Send_Preemptive(" - PACKET RECEPTION CORRUPTED - \n");	// send over USB
				//continue;
			}
			else															// else read packet from radio
			{
				__disable_irq();
				while (temp!=0)
				{
					SPI_transceive(0xBF);	RF_RX_BUFFER[0]=SPI_transceive(0xFF);			// read first byte (packet length)
					for (RF_RX_NUM=1;RF_RX_NUM<RF_RX_BUFFER[0]+3;RF_RX_NUM++)								// read the rest +2 CRC bytes
					{
						SPI_transceive(0xBF);	RF_RX_BUFFER[RF_RX_NUM]=SPI_transceive(0xFF);

					}

					processPacket(RF_RX_NUM-3, RF_RX_BUFFER+1,3);

					SPI_transceive(0xAF);	SPI_transceive(0xD7);	temp=SPI_transceive(0xFF);	// FIFO_RX_NUM - data rdy?
				}
				__enable_irq();

			}
			RF_flag=0;

    	}
    	/*else*/ { watchDogVariable=0;}

    	__disable_irq();
    	RF_stat = SPI_transceive(0x3D);	//get status

    	if (RF_stat==0x6F)	// lazy problem fixer
    	{
    	// SFRX - RX FLUSH
		SPI_transceive(0x3A);
		// set to receive
		SPI_transceive(0x34);
    	}
    	__enable_irq();

    }
}

void processPacket(uint8_t length, uint8_t data[],uint8_t srcIf)
{
	uint16_t temp2A=0;

	if (data[0]=='a')	// address input detected - addr input is optional
	{
		if (data[1]==ADDR+48)	// correct addres - continue
		{
			data+=2;	length-=2;
		}
		else
		{
			return;				// msg for someone else - return
		}
	}

	switch(data[0])		// all available functions
	{
		case 'p':		// pulse for RSSI measurement - add src + dest ADDR?
		{
			lastPulseSrc=srcIf;
			RF_send(1,"r");
			break;
		}
		case 'r':		// pulse reading - send RSSI level
		{
			lastPulseSrc=0;
			RF_TX_NUM = 0;
			RF_TX_BUFFER[RF_TX_NUM++]='e';
			RF_TX_BUFFER[RF_TX_NUM++]=ADDR;
			if(RSSI<0) { RF_TX_BUFFER[RF_TX_NUM++]= '-'; temp2A=RSSI*(-1);}
			else temp2A=RSSI;
			RF_TX_BUFFER[RF_TX_NUM++]= (temp2A/100)+48;
			RF_TX_BUFFER[RF_TX_NUM++]= ( ( (temp2A-(temp2A%10))%100 )/10 )+48;
			RF_TX_BUFFER[RF_TX_NUM++]= (temp2A%10)+48;

			RF_send(RF_TX_NUM,RF_TX_BUFFER);
			RF_TX_NUM=0;
			break;
		}
		case 'e':		// echo received - human readable version - TODO lookup table + ability to send the table
		{
			if (lastPulseSrc==1)
			{
				while ( (USB_TX_NUM +1) > 128);	// wait if buffer is full

				USB_TX_BUFFER[USB_TX_NUM++]='e'; USB_TX_BUFFER[USB_TX_NUM++]='c'; USB_TX_BUFFER[USB_TX_NUM++]='h'; USB_TX_BUFFER[USB_TX_NUM++]='o'; USB_TX_BUFFER[USB_TX_NUM++]=' ';
				USB_TX_BUFFER[USB_TX_NUM++]='f'; USB_TX_BUFFER[USB_TX_NUM++]='r'; USB_TX_BUFFER[USB_TX_NUM++]='o'; USB_TX_BUFFER[USB_TX_NUM++]='m'; USB_TX_BUFFER[USB_TX_NUM++]=':';
				USB_TX_BUFFER[USB_TX_NUM++]=' ';

				USB_TX_BUFFER[USB_TX_NUM++]=  (data[1]/100)+48;
				USB_TX_BUFFER[USB_TX_NUM++]= ( (data[1]- ((data[1]/100) *100)) /10) +48;
				USB_TX_BUFFER[USB_TX_NUM++]=  (data[1]%10)+48;

				USB_TX_BUFFER[USB_TX_NUM++]=' '; USB_TX_BUFFER[USB_TX_NUM++]='a'; USB_TX_BUFFER[USB_TX_NUM++]='t'; USB_TX_BUFFER[USB_TX_NUM++]=' ';
				for (i=2;i<length;i++)
				{
					USB_TX_BUFFER[USB_TX_NUM++]=data[i];
				}
				USB_TX_BUFFER[USB_TX_NUM++]='\r';
				USB_TX_BUFFER[USB_TX_NUM++]='\n';

				if(USB_TX_INDEX == 0)		// if UART not sending
				{
					USART_SendData(USART1,USB_TX_BUFFER[USB_TX_INDEX++]);	// manually initiate first transfer to trigger interrupt-based flush
				}
			}
			else if (lastPulseSrc==2)
			{
				while ( (RS485_TX_NUM +1) > 128);	// wait if buffer is full

				RS485_TX_BUFFER[RS485_TX_NUM++]=24-1+48;	// length=24
				RS485_TX_BUFFER[RS485_TX_NUM++]='e'; RS485_TX_BUFFER[RS485_TX_NUM++]='c'; RS485_TX_BUFFER[RS485_TX_NUM++]='h'; RS485_TX_BUFFER[RS485_TX_NUM++]='o'; RS485_TX_BUFFER[RS485_TX_NUM++]=' ';
				RS485_TX_BUFFER[RS485_TX_NUM++]='f'; RS485_TX_BUFFER[RS485_TX_NUM++]='r'; RS485_TX_BUFFER[RS485_TX_NUM++]='o'; RS485_TX_BUFFER[RS485_TX_NUM++]='m'; RS485_TX_BUFFER[RS485_TX_NUM++]=':';
				RS485_TX_BUFFER[RS485_TX_NUM++]=' ';

				RS485_TX_BUFFER[RS485_TX_NUM++]=  (data[1]/100)+48;
				RS485_TX_BUFFER[RS485_TX_NUM++]= ( (data[1]- ((data[1]/100) *100)) /10) +48;
				RS485_TX_BUFFER[RS485_TX_NUM++]=  (data[1]%10)+48;

				RS485_TX_BUFFER[RS485_TX_NUM++]=' '; RS485_TX_BUFFER[RS485_TX_NUM++]='a'; RS485_TX_BUFFER[RS485_TX_NUM++]='t'; RS485_TX_BUFFER[RS485_TX_NUM++]=' ';
				for (i=2;i<length;i++)
				{
					RS485_TX_BUFFER[RS485_TX_NUM++]=data[i];
				}
				RS485_TX_BUFFER[RS485_TX_NUM++]='\r';
				RS485_TX_BUFFER[RS485_TX_NUM++]='\n';

				if(RS485_TX_INDEX == 0)		// if UART not sending
				{
					GPIO_WriteBit(GPIOA, GPIO_Pin_8, 1);	// RS485_DIR 0=rx
					USART_SendData(USART2,RS485_TX_BUFFER[RS485_TX_INDEX++]);	// manually initiate first transfer to trigger interrupt-based flush
				}
			}
			break;
		}
		case 'x':				// expected message "x 1234 56"
		{
			for(i=0;i<4;i++)	// format correction - all ASCII to number
			{
				if (data[i+2]>96)
					data[i+2]-=87;
				else if (data[i+2]>64)
					data[i+2]-=55;
				else if (data[i+2]>47)
					data[i+2]-=48;
			}
			i=data[5];			// reform hex value from numbers
			i+=data[4]*16;
			i+=data[3]*16*16;
			i+=data[2]*16*16*16;

			for(j=0;j<2;j++)	// format correction - all ASCII to number
			{
				if (data[j+7]>96)
					data[j+7]-=87;
				else if (data[j+7]>64)
					data[j+7]-=55;
				else if (data[j+7]>47)
					data[j+7]-=48;
			}
			j=data[8];			// reform hex value from numbers
			j+=data[7]*16;											//	0x0014,BAUD);   //SYMBOL_RATE2 	// 0x23-0.3k  0x33-0.6k  0x43-1.2k  0x53-2.4k  0x63-4.8k  0x73-9.6k  0x78-12k	| max= BW/2

			CC112X_setting(i,j);	// i - 4 hex digits		j - 2 hex digits
			break;
		}
		case '*':
		{
			NVIC_SystemReset();
			break;
		}
		case '1':		// USB interface
		{
			for (i=1;i<length;i++)					// add to UART TX Buffer
			{
				USB_TX_BUFFER[USB_TX_NUM++]=data[i];
			}
			USB_TX_BUFFER[USB_TX_NUM++]='\r';		// formatting suffix - debug
			USB_TX_BUFFER[USB_TX_NUM++]='\n';

			if(USB_TX_INDEX == 0)		// if UART not sending
			{
				USART_SendData(USART1,USB_TX_BUFFER[USB_TX_INDEX++]);	// manually initiate first transfer to trigger interrupt-based flush
			}
			break;
		}
		case '2':		// RS485 interface
		{
			RS485_TX_BUFFER[RS485_TX_NUM++]=length-1+48;
			for (i=1;i<length;i++)					// add to UART TX Buffer
			{
				RS485_TX_BUFFER[RS485_TX_NUM++]=data[i];
			}
	//		RS485_TX_BUFFER[RS485_TX_NUM++]='\r';		// formatting suffix - debug
	//		RS485_TX_BUFFER[RS485_TX_NUM++]='\n';

			if(RS485_TX_INDEX == 0)		// if UART not sending
			{
				GPIO_WriteBit(GPIOA, GPIO_Pin_8, 1);	// RS485_DIR 0=rx
				USART_SendData(USART2,RS485_TX_BUFFER[RS485_TX_INDEX++]);	// manually initiate first transfer to trigger interrupt-based flush
			}
			break;
		}
		case '3':		// radio interface
		{
			RF_send(length-1,data+1);
			break;
		}
	}
}


void EXTI0_1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
	}

	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		if (RF_stat==31)	// RX
		{
			if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==0)	// low after finished rx
			{
				RF_flag=1;
			}
			else											// high after RX start
			{
				RSSI = RF_getRSSI();
			}
		}
		else	//TX
		{

		}
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
	  USB_RX_BUFFER[USB_RX_NUM++] = USART_ReceiveData(USART1);
//	  USART_SendData(USART1, USB_RX_BUFFER[USB_RX_NUM-1]);	// debug tool - direct echo

	  if(USB_RX_NUM==128)
	  {
		  USB_RX_NUM=0;
	  }

	  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }

  if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
  {
	  if(USB_TX_INDEX != USB_TX_NUM)
	  {
		  USART_SendData(USART1,USB_TX_BUFFER[USB_TX_INDEX++]);
	  }
	  else
	  {
		  USB_TX_INDEX=0;
		  USB_TX_NUM  =0;
	  }

	  USART_ClearITPendingBit(USART1, USART_IT_TC);
  }
}

void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
	  RS485_RX_BUFFER[RS485_RX_NUM++] = USART_ReceiveData(USART2);
//	  USART_SendData(USART2, RS485_RX_BUFFER[RS485_RX_NUM-1]);	// debug tool - direct echo

	  if(RS485_RX_NUM==128)
	  {
		  RS485_RX_NUM=0;
	  }

	  USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }

  if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
  {
	  if(RS485_TX_INDEX != RS485_TX_NUM)
	  {
		  USART_SendData(USART2,RS485_TX_BUFFER[RS485_TX_INDEX++]);
	  }
	  else
	  {
		  RS485_TX_INDEX=0;
		  RS485_TX_NUM  =0;
		  GPIO_WriteBit(GPIOA, GPIO_Pin_8, 0);	// RS485_DIR 0=rx

	  }

	  USART_ClearITPendingBit(USART2, USART_IT_TC);
  }
}

// watchdog function - reset if main loop takes more than 6sec
void TIM6_DAC_IRQHandler(void)
{
//	GPIOB->ODR ^=0x0008;	// LED toggle
//	RF_stat = SPI_transceive(0x3D);
	if (watchDogVariable==1) NVIC_SystemReset();
	watchDogVariable=1;
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
}


void USART_Send_Preemptive(volatile char *str)
{
	while(*str)		// Load data to UART TX buffer
	{
		while(USB_TX_NUM>128);	// wait for buffer space
		USB_TX_BUFFER[USB_TX_NUM++]=*str;
		str++;
	}

	if(USB_TX_INDEX == 0)		// if UART not sending		// TODO use apropriate flag
	{
		USART_SendData(USART1,USB_TX_BUFFER[USB_TX_INDEX++]);	// manually initiate first transfer to trigger interrupt-based flush
	}
}

uint8_t CC112X_setting(uint16_t addr, uint8_t data)
{
	char addrHigh = addr>>8;

	if (addrHigh!=0x00) SPI_transceive(addrHigh);	// send only for extended addr range registers

	SPI_transceive((char)addr);
	addrHigh = SPI_transceive(data);

    return addrHigh;
}

int16_t RF_getRSSI(void)
{
	uint16_t RSSI_raw = 0;
	int16_t newRSSI = 0;
	uint8_t RF_status = 0;

	// wait for RSSI_rdy
	do {
		SPI_transceive(0xAF);	SPI_transceive(0x72);	RF_status = SPI_transceive(0xFF);
	   } while ((RF_status&0x1)==0);

	// get RSSI-MSB
	SPI_transceive(0xAF);	SPI_transceive(0x71);	RSSI_raw = SPI_transceive(0xFF);

	//RSSI_raw = (RSSI_raw<<4) | ((RF_status>>3)&0xF);
	if (RSSI_raw >= 0x80) {newRSSI = 0x00FF & (~(RSSI_raw-1)); newRSSI -= 2*newRSSI;  }
	else newRSSI = RSSI_raw;

	return newRSSI;

}


/*
 * - blocking till transmission finished
 */
void RF_send(uint8_t length, uint8_t data[])
{
	uint32_t cycleIndex = 0;

	// protocol delay - to avoid collisions after broadcast request
	watchDogVariable=0;
	for(cycleIndex=0;cycleIndex<(ADDR*length*44000);cycleIndex++);

	//SPI_transceive(0x31); 			// FS TX mode

//	if (RF_stat==0x7F)
//		SPI_transceive(0x3B);	// SFTX - TX FLUSH

	__disable_irq();
	//TX FIFO - length byte
	//CC112X_setting(0x003F,length);
	SPI_transceive(0x3F);	SPI_transceive(length);
	//TX FIFO - data byte
	//CC112X_setting(0x003F,data[cycleIndex]);
	for(cycleIndex=0;cycleIndex<length;cycleIndex++)
	{
		SPI_transceive(0x3F);	SPI_transceive(data[cycleIndex]);
	}
	__enable_irq();

	GPIO_WriteBit(GPIOA, GPIO_Pin_1, 0);	// RX end
	GPIO_WriteBit(GPIOA, GPIO_Pin_0, 1);	// TX rdy

	GPIOB->ODR ^=0x0008;	// LED toggle

	RF_stat = 47;	// TX
	SPI_transceive(0x35); 			// TX mode
	while (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==0);	// wait to send sync word
	while (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==1);	// wait to finish sending

	GPIOB->ODR ^=0x0008;	// LED toggle

	GPIO_WriteBit(GPIOA, GPIO_Pin_0, 0);	// TX end
	GPIO_WriteBit(GPIOA, GPIO_Pin_1, 1);	// RX rdy

	RF_stat = SPI_transceive(0x3D);	//get status
}


/*
 * TODO - Return status of init ?
 */
uint8_t RF_init(void)
{
	// settings
	FREQ = 0x6CAD00;	//0x6C8066=868.0125MHz 0x6C80CD=868.025MHz 0x6CAD00=869.40625MHz
	FREQOFF= 0;//FREQOFF_DEV[ADDR];
	BAUD = 0x33;
	CHBW = 0x14;
	TXPW = 0x77;

	CC112X_setting(0x0000,0x06);   //IOCFG3
	CC112X_setting(0x0001,0x87);   //IOCFG2
	CC112X_setting(0x0002,0xB0);   //IOCFG1
	CC112X_setting(0x0003,0xBC);   //IOCFG0
	CC112X_setting(0x0008,0x0B);   //SYNC_CFG1		// FSK-0x0B GFSK-0x08
	CC112X_setting(0x000A,0x89);   //DEVIATION_M	// def=0x06   0x89/0x02-3k
	CC112X_setting(0x000B,0x02);   //MODCFG_DEV_E 	// 0x03-4k 0x02-2k 0x01-1k		4bit=0-FSK =1-GFSK
	CC112X_setting(0x000C,0x1C);   //DCFILT_CFG
	CC112X_setting(0x000D,0x18);   //PREAMBLE_CFG1
	CC112X_setting(0x0010,0xC6);   //IQIC
	CC112X_setting(0x0011,CHBW);   //CHAN_BW		// 0x50-7k8  0x14-10k  0x10-12k5  0x0A-20k  0x08-25k
	CC112X_setting(0x0013,0x05);   //MDMCFG0
	CC112X_setting(0x0014,BAUD);   //SYMBOL_RATE2 	// 0x23-0.3k  0x33-0.6k  0x43-1.2k  0x53-2.4k  0x63-4.8k  0x73-9.6k  0x78-12k	| max= BW/2
	CC112X_setting(0x0017,0x20);   //AGC_REF
	CC112X_setting(0x0018,0x19);   //AGC_CS_THR
	CC112X_setting(0x0019,0xA6);   //AGC_GAIN_ADJUST	// ignored?
	CC112X_setting(0x001C,0xA9);   //AGC_CFG1
	CC112X_setting(0x001D,0xCF);   //AGC_CFG0
	CC112X_setting(0x001E,0x00);   //FIFO_CFG
	CC112X_setting(0x0020,0x03);   //SETTLING_CFG
	CC112X_setting(0x0021,0x12);   //FS_CFG
	CC112X_setting(0x0026,0x10);   //PKT_CFG2			//default=0x04(LBT)  0x10=LBT ETSI
	CC112X_setting(0x0028,0x20);   //PKT_CFG0
	CC112X_setting(0x0029,0x3F);   //RFEND_CFG1			//default=0x0F	RX after packet=3f
	CC112X_setting(0x002A,0x30);   //RFEND_CFG0			//default=0x00	RX after TX =30
	CC112X_setting(0x002B,TXPW);   //PA_CFG2		// 0x77=27dBm/11dBm
	CC112X_setting(0x002E,0xFF);   //PKT_LEN
	CC112X_setting(0x2F00,0x00);   //IF_MIX_CFG
	CC112X_setting(0x2F01,0x23);   //FREQOFF_CFG
	CC112X_setting(0x2F0A,FREQOFF>>8);	//FREQOFF1
	CC112X_setting(0x2F0B,FREQOFF);	//FREQOFF0
	CC112X_setting(0x2F0C,(FREQ>>16));	//FREQ2
	CC112X_setting(0x2F0D,(FREQ>>8));   //FREQ1
	CC112X_setting(0x2F0E,(FREQ>>0));   //FREQ0
	CC112X_setting(0x2F12,0x00);   //FS_DIG1
	CC112X_setting(0x2F13,0x5F);   //FS_DIG0
	CC112X_setting(0x2F16,0x40);   //FS_CAL1
	CC112X_setting(0x2F17,0x0E);   //FS_CAL0
	CC112X_setting(0x2F19,0x03);   //FS_DIVTWO
	CC112X_setting(0x2F1B,0x33);   //FS_DSM0
	CC112X_setting(0x2F1D,0x17);   //FS_DVC0
	CC112X_setting(0x2F1F,0x50);   //FS_PFD
	CC112X_setting(0x2F20,0x6E);   //FS_PRE
	CC112X_setting(0x2F21,0x14);   //FS_REG_DIV_CML
	CC112X_setting(0x2F22,0xAC);   //FS_SPARE
	CC112X_setting(0x2F27,0xB4);   //FS_VCO0
	CC112X_setting(0x2F32,0x0E);   //XOSC5
	CC112X_setting(0x2F36,0x03);   //XOSC1
	CC112X_setting(0x2F91,0x08);   //Serial_Status

	// Analog temp reading setup
	CC112X_setting(0x2F9C,0x2A);	// ATEST
	CC112X_setting(0x2F9E,0x0C);	// ATEST_MODE
	CC112X_setting(0x2F2D,0x07);	// GBIAS1

	manualCalibration();

	return 0;
}


// blocking send'n'read - redo via irq? (pointless for given speeds)
uint8_t SPI_transceive(uint8_t data)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_SR_TXE) == RESET);
	SPI_SendData8(SPI1, data);

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_ReceiveData8(SPI1);
}

#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2
static void manualCalibration(void) {

    uint8_t original_fs_cal2;
    uint8_t calResults_for_vcdac_start_high[3];
    uint8_t calResults_for_vcdac_start_mid[3];
    uint8_t marcstate;
    uint8_t writeByte;

//    GPIO_ResetBits(GPIOC,GPIO_Pin_10);	//CS
    // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    SPI_transceive(0x2F); SPI_transceive(0x25); SPI_transceive(writeByte);

    // 2) Start with high VCDAC (original VCDAC_START + 2):
    SPI_transceive(0xAF); SPI_transceive(0x15); original_fs_cal2 = SPI_transceive(0xFF);
    writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
    SPI_transceive(0x2F); SPI_transceive(0x15); SPI_transceive(writeByte);

    // 3) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    SPI_transceive(0x33);

    do {
    	SPI_transceive(0xAF); SPI_transceive(0x73); marcstate = SPI_transceive(0xFF);
    } while (marcstate != 0x41);

    // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
    //    high VCDAC_START value
    SPI_transceive(0xAF); SPI_transceive(0x25); calResults_for_vcdac_start_high[FS_VCO2_INDEX] = SPI_transceive(0xFF);
    SPI_transceive(0xAF); SPI_transceive(0x23); calResults_for_vcdac_start_high[FS_VCO4_INDEX] = SPI_transceive(0xFF);
    SPI_transceive(0xAF); SPI_transceive(0x18); calResults_for_vcdac_start_high[FS_CHP_INDEX] = SPI_transceive(0xFF);

    // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    SPI_transceive(0x2F); SPI_transceive(0x25); SPI_transceive(writeByte);

    // 6) Continue with mid VCDAC (original VCDAC_START):
    writeByte = original_fs_cal2;
    SPI_transceive(0x2F); SPI_transceive(0x15); SPI_transceive(writeByte);

    // 7) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    SPI_transceive(0x33);

    do {
       	SPI_transceive(0xAF); SPI_transceive(0x73); marcstate = SPI_transceive(0xFF);
       } while (marcstate != 0x41);

    // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
    //    with mid VCDAC_START value
    SPI_transceive(0xAF); SPI_transceive(0x25); calResults_for_vcdac_start_mid[FS_VCO2_INDEX] = SPI_transceive(0xFF);
	SPI_transceive(0xAF); SPI_transceive(0x23); calResults_for_vcdac_start_mid[FS_VCO4_INDEX] = SPI_transceive(0xFF);
	SPI_transceive(0xAF); SPI_transceive(0x18); calResults_for_vcdac_start_mid[FS_CHP_INDEX] = SPI_transceive(0xFF);


    // 9) Write back highest FS_VCO2 and corresponding FS_VCO
    //    and FS_CHP result
    if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
        calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
        writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
        SPI_transceive(0x2F); SPI_transceive(0x25); SPI_transceive(writeByte);
        writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
        SPI_transceive(0x2F); SPI_transceive(0x23); SPI_transceive(writeByte);
        writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
        SPI_transceive(0x2F); SPI_transceive(0x18); SPI_transceive(writeByte);
    } else {
        writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
        SPI_transceive(0x2F); SPI_transceive(0x25); SPI_transceive(writeByte);
        writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
        SPI_transceive(0x2F); SPI_transceive(0x23); SPI_transceive(writeByte);
        writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
        SPI_transceive(0x2F); SPI_transceive(0x18); SPI_transceive(writeByte);
    }
//    GPIO_SetBits(GPIOC,GPIO_Pin_10);	//CS
}


