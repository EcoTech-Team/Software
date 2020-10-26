/*
 * led_driver.c
 */

#include <stdbool.h>
#include "led_driver.h"
#include "hardware.h"


void LED_Init(void)
{
	LED_DDR |= (1<<LED_R) | (1<<LED_G);
}


void LED_RedOn(void)
{
	LED_PORT |= (1<<LED_R);
}


void LED_RedOff(void)
{
	LED_PORT &= ~(1<<LED_R);
}


void LED_RedToggle(void)
{
	LED_PORT ^= (1<<LED_R);
}


void LED_RedSet(LED_State state)
{
	if(state) LED_PORT |= (1<<LED_R);
	else LED_PORT &=  (1<<LED_R);
}


void LED_GreenOn(void)
{
	LED_PORT |= (1<<LED_G);
}


void LED_GreenOff(void)
{
	LED_PORT &= ~(1<<LED_G);
}


void LED_GreenToggle(void)
{
	LED_PORT ^= (1<<LED_G);
}


void LED_GreenSet(LED_State state)
{
	if(state) LED_PORT |= (1<<LED_G);
	else LED_PORT &= ~(1<<LED_G);
}


