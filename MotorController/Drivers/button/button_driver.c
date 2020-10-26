/*
 * button_driver.c
 */

#include <stdint.h>
#include "button_driver.h"


void BTN_Init(void)
{
	// Set GPIO in input configuration
	BTN_DDR_A &= ~(1<<BTN_P_A);
	BTN_PORT_A |= (1<<BTN_P_A);

	BTN_DDR_B &= ~(1<<BTN_P_B);
	BTN_PORT_B |= (1<<BTN_P_B);

	BTN_DDR_C &= ~(1<<BTN_P_C); //input
	BTN_PORT_C |= (1<<BTN_P_C); //pull up

	BTN_DDR_D &= ~(1<<BTN_P_D);
	BTN_PORT_D |= (1<<BTN_P_D);
}


bool BTN_StateA(void)
{
	// Return state of selected button
	if (BTN_PIN_A & (1<<BTN_P_A)) return false;
	else return true;
}


bool BTN_StateB(void)
{
	// Return state of selected button
	if (BTN_PIN_B & (1<<BTN_P_B)) return false;
	else return true;
}


bool BTN_StateC(void)
{
	// Return state of selected button
	if(BTN_PIN_C & (1<<BTN_P_C)) return false;
	else return true;
}


bool BTN_StateD(void)
{
    // Return state of selected button
	if (BTN_PIN_D & (1<<BTN_P_D)) return false;
	else return true;
}


bool BTN_Pressed(BTN_Button button)
{
	switch (button)
	{
	case BTN_A:
		return BTN_StateA();
		break;
	case BTN_B:
		return BTN_StateB();
		break;
	case BTN_C:
		return BTN_StateC();
		break;
	case BTN_D:
		return BTN_StateD();
		break;
	default:
		return false;
		break;
	}
}


