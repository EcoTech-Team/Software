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


void BUS_Init(void)
{
    //! Uart initialisation
	unsigned int ubrr = MYUBRR; //todo troche to na pałe
	UBRRH  = (uint8_t) ((ubrr)>>8);
	UBRRL  = (uint8_t) ubrr;
	//Enable receiver and transmitter and interrupt*/
	//UCSRB = (1<<RXEN) | (1<<TXEN) | (1<<RXCIE) | (1<<TXCIE);
	UCSRB = (1<<TXEN) | (1<<TXCIE);
	/* Set frame format: 8data, 1stop bit , parity disabled*/
	UCSRC |= (1<<URSEL) | (1<<UCSZ0) | (1<<UCSZ1);
	UCSRC &= ~((1<<UPM0) | (1<<UPM1));

	// TIM0 peripheral initialization
	//todo prescaler TIM0 wypadało by dobrac do szybkosci zegara i ubrr. z 8 uwalało bajty przy 1mhz i 4800, ale na 8mhz moze byc ok.
	TIMSK |= (1<<BUS_TOIE); //interr after overflow
	sei();
}


void BUS_Send(uint8_t *buff, uint8_t len)
{
	//! Dump message if transmission is already ongoing or buffered
	if(_TxOngoing || _TxPending) return;

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
		UDR = _TxBuff[_TxData++];
	}
	//! Pend message to be send after received frame
	else _TxPending = true;
}


void BUS_SendBlocking(uint8_t *buff, uint8_t len)
{
	for(uint8_t i=0; i<len; i++)
	{
		while ( !( UCSRA & (1<<UDRE)) );
		UDR = buff[i];
	}
}


ISR(USART_TXC_vect)
{
	// copy byte from _TxBuff to uart
	if(_TxData < _TxLen) UDR = _TxBuff[_TxData++];
	else
	{
		_BusFree = true;
		_TxOngoing = false;
	}
}


ISR(USART_RXC_vect)
{
	_BusFree=false;

	//! Copy data to buffer
	if(_RxData < BUS_BUFF_LEN) _RxBuff[_RxMsg][_RxData++] = UDR;
	else _RxOverflow = true;

	//! Start and reset timer to fire the isr if this is last byte
	BUS_TCNT = 0;
	BUS_TCCR |= (1<<CS01) | (1<<CS00);
}


ISR(TIMER0_OVF_vect)
{
	uint8_t temp_RxMsg = _RxMsg;
	uint8_t temp_RxData = _RxData;
	bool temp_RxOverflow = _RxOverflow;

	//! Stop timer
	BUS_TCCR &= ~((1<<CS01) | (1<<CS00));
	//! Set next rx buffer and reset data index to 0
	_RxMsg = (_RxMsg + 1) % BUS_BUFF_NUM;
	_RxData = 0;
	_RxOverflow = false;

	//! Call receive function if receive buffer is not overflowed
	if(!temp_RxOverflow) BUS_Received(_RxBuff[temp_RxMsg], temp_RxData);

	//! Send message if was pending
	if(_TxPending)
	{
		_TxOngoing = true;
		_TxPending = false;
		UDR = _TxBuff[_TxData++]; // start uart transmission
	}
	else _BusFree = true;
}


