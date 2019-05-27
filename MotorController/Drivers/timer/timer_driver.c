/*
 * timer_driver.c
 */

#include <stdint.h>
#include "timer_driver.h"


//! Global SysTick timer. Can be externed in other files
//uint16_t SysTick = 0;


void TIM_Init(void)
{
	TCCR1B |= (1<<WGM12); //CTC Mode
	OCR1A = 1337;                   // todo ##################### to na pewno trzeba dobrać #####################
	TIMSK |= (1<<OCIE1A); //setting interr after achieving value in OCR2
	TIM_Start();
}


ISR(TIMER1_COMPA_vect)
{
	//SysTick++;
	TIM_SysTick();
}


void TIM_Reset(void)
{
	TCNT1 = 0;
}


void TIM_Start(void)
{
	TCNT1 = 0;
	TCCR1B |= (1<<CS10) | (1<<CS12); //prescaler 1024
}


void TIM_Stop(void)
{
	TCCR1B &= !((1<<CS10) | (1<<CS12));
}


