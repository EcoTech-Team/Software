/*
 * bus_driver.c
 */


#include <stdbool.h>
#include "bus_driver.h"

//! Free bus flag
static volatile bool _BusFree = true;
//! Message is pending to send
static volatile bool _TxPending = false;
//! Tranmission ongoing
static volatile bool _TxOngoing = false;
//! Receive buffer
static uint8_t _RxBuff[BUS_BUFF_NUM][BUS_BUFF_LEN];
//! Rx message index
static uint8_t _RxMsg = 0;
//! Rx data intex
static uint8_t _RxData = 0;
//! Rx overflowed
static bool _RxOverflow = false;
//! Transmit buffer
static uint8_t _TxBuff[BUS_BUFF_LEN];
//! Tx data index
static uint8_t _TxData = 0;
//! Tx data to send
static uint8_t _TxLen = 0;

uint16_t _sr;

void BUS_Init(void)
{
	/*!
	 * BUS_RX_PIN = GPIOB_PIN11
	 *
	 * CNF1		CNF0	MODE1	MODE0	ODR
	 * 0		1		0		0		0
	 *
	 * Input floating
	 *
	 * BUS_TX_PIN = GPIOB_PIN10
	 *
	 * CNF1		CNF0	MODE1	MODE0	ODR
	 * 1		0		1		0		0
	 *
	 * Alternate function push-pull @ 2 MHz
	 *
	 */

	BUS_GPIO->ODR |= GPIO_ODR_ODR10;
	BUS_GPIO->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
	BUS_GPIO->CRH |= GPIO_CRH_CNF11_0 | GPIO_CRH_MODE10_1;

	#ifdef BUS_TIM_USED
	//1÷(72000000÷65535÷3) ~~ 2.7ms
	BUS_TIM->PSC = 3;
	BUS_TIM->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);
	// enable BUS_TIM module
	BUS_TIM->CR1 |= OPM;
	#endif

	//! Calculated on asap:
	//! 36000000 / (16 * 38400) = 585.9375
	//! DIV_Mantissa = 585
	//! DIV_Fraction = 0.9375 * 16 = 15
	//! TODO: Change to calculations based on BUS_BAUD macro
	BUS_UART->BRR = (58 << 4) | 10;
	BUS_UART->CR1 |= USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_RE;
	BUS_UART->CR3 |= USART_CR3_EIE;
	NVIC_EnableIRQ(USART3_IRQn);
}

#ifdef BUS_TIM_USED
void TIM2_IRQHandler(void)
	{
		//! Send message if was pending
		if(_TxPending)
		{
			_TxOngoing = true;
			_TxPending = false;
			// Set interupt on clearing TX register.
			BUS_UART->CR1 |= USART_CR1_TXEIE;
			// start uart transmission
			BUS_UART->DR = _TxBuff[_TxData++]; 
		}
		else _BusFree = true;
	}
#endif

void USART3_IRQHandler(void)
{
	uint8_t _TempDR;
	_sr = BUS_UART->SR;
	if(_sr & USART_SR_RXNE)
	{
		_BusFree = false;
		if(_RxData < BUS_BUFF_LEN) _RxBuff[_RxMsg][_RxData++] = BUS_UART->DR;
		else _RxOverflow = true;
	}
	if(_sr & USART_SR_IDLE)
	{
		//! Read DR to clear flag
		_TempDR = BUS_UART->DR;
		
		uint8_t temp_RxMsg = _RxMsg;
		uint8_t temp_RxData = _RxData;
		bool temp_RxOverflow = _RxOverflow;

		_RxMsg = (_RxMsg + 1) % BUS_BUFF_NUM;
		_RxData = 0;
		_RxOverflow = false;

		//! Call receive function if receive buffer is not overflowed
		if(!temp_RxOverflow) BUS_Received(_RxBuff[temp_RxMsg], temp_RxData);

		#ifdef BUS_TIM_USED
		BUS_TIM->CNT = 0; // just in case, it should be 0 already.
		// timer enable
		BUS_TIM->CR1 |= TIM_CR1_CEN; 
		#else
		//! Send message if was pending
		if(_TxPending)
		{
			_TxOngoing = true;
			_TxPending = false;
			// Set interupt on clearing TX register.
			BUS_UART->CR1 |= USART_CR1_TXEIE;
			// start uart transmission
			BUS_UART->DR = _TxBuff[_TxData++]; 
		}
		else _BusFree = true;
		#endif
	}
	if(_sr & USART_SR_TXE && _TxOngoing)
	{
		if(_TxData < _TxLen) BUS_UART->DR = _TxBuff[_TxData++];
		else
		{
			// clear interupt on clearing TX register.
			BUS_UART->CR1 &= ~USART_CR1_TXEIE;
			_BusFree = true;
			_TxOngoing = false;
		}
	}
	if(_sr & (USART_SR_ORE || USART_SR_FE || USART_SR_NE ))
	{
		_TempDR = BUS_UART->DR;
	}
}

void BUS_Send(uint8_t *buff, uint8_t len)
{
	//! Dump message if transmission is already ongoing or buffered
	if(_TxOngoing || _TxPending) return;

	if( len > BUS_BUFF_LEN) len = BUS_BUFF_LEN;

	//! Copy passed message to internal buffer
	for(uint8_t i=0; i<len; i++)
		_TxBuff[i] = buff[i];
	//! Set tx size and pointer
	_TxLen = len;
	_TxData = 0;

	//! Start transmission if bus is free
	if(_BusFree)
	{
		_BusFree = false;
		_TxOngoing = true;
		// Set interupt on clearing TX register.
		BUS_UART->CR1 |= USART_CR1_TXEIE;
		BUS_UART->DR = _TxBuff[_TxData++];
	}
	//! Pend message to be send after received frame
	else _TxPending = true;
}

void BUS_SendBlocking(uint8_t *buff, uint8_t len)
{
	for(uint8_t i=0; i<len; i++)
	{
		while(!(USART1->SR & USART_SR_TXE));
		BUS_UART->DR = buff[i];
	}
}
