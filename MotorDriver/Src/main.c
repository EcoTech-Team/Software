/*
 * main.c
 */


#include "system_driver.h"
#include "led_driver.h"
#include "motor_driver.h"
#include "button_driver.h"
#include "encoder_driver.h"
#include "bus_driver.h"
#include "msg_lib.h"


bool _ProcessSamples;
uint16_t *_SamplesPointer;
uint16_t _AvgSample;
uint32_t _Rotations;
int16_t _Rpm;
uint8_t _TableIndex;


bool _MotorState = false;
uint8_t _ControllerState=0;
uint8_t _MessageTimer = 0;

uint8_t messageBuffer[7];


const uint16_t _CurrentTable[120] = {2045, 2105, 2164, 2224, 2283, 2343, 2402, 2462, 2522, 2581, 2590, 2586, 2582, 2579, 2575, 2571, 2567, 2564, 2560, 2556, 2553, 2549, 2545, 2541, 2538, 2518, 2497, 2477, 2457, 2437, 2417, 2397, 2377, 2357, 2337, 2317, 2296, 2276, 2256, 2236, 2216, 2196, 2176, 2156, 2136, 2116, 2095, 2075, 2055, 2035, 2009, 2008, 2007, 2007, 2006, 2006, 2005, 2005, 2004, 2004, 2003, 2003, 2002, 2002, 2001, 2001, 2000, 2000, 1999, 1999, 1998, 1998, 1997, 1997, 1996, 1995, 1995, 1994, 1994, 1993, 1993, 1992, 1992, 1991, 1991, 1990, 1990, 1989, 1989, 1988, 1988, 1987, 1987, 1986, 1986, 1985, 1985, 1984, 1983, 1983, 1982, 1982, 1981, 1981, 1980, 1980, 1979, 1979, 1978, 1978, 1977, 1977, 1976, 1976, 1975, 1975, 1974, 1974, 1973, 1973};

const uint16_t _CurrentTable1[120] = {1975, 2026, 2078, 2129, 2180, 2232, 2283, 2334, 2386, 2437, 2466, 2462, 2458, 2455, 2451, 2447, 2443, 2440, 2436, 2432, 2428, 2425, 2421, 2417, 2396, 2379, 2362, 2345, 2328, 2311, 2293, 2276, 2259, 2242, 2225, 2208, 2191, 2174, 2156, 2139, 2122, 2105, 2088, 2071, 2054, 2037, 2019, 2002, 1985, 1968, 1970, 1970, 1970, 1969, 1969, 1969, 1968, 1968, 1968, 1968, 1967, 1967, 1967, 1966, 1966, 1966, 1966, 1965, 1965, 1965, 1964, 1964, 1964, 1963, 1963, 1963, 1963, 1962, 1962, 1962, 1961, 1961, 1961, 1960, 1960, 1960, 1960, 1959, 1959, 1959, 1958, 1958, 1958, 1957, 1957, 1957, 1957, 1956, 1956, 1956, 1955, 1955, 1955, 1954, 1954, 1954, 1954, 1953, 1953, 1953, 1952, 1952, 1952, 1952, 1951, 1951, 1951, 1950, 1950, 1950};
int main(void)
{

	SYS_Init();
	LED_Init();
	MTR_Init();
	BTN_Init();
	ENC_Init();
	BUS_Init();
	MSG_CrcInit();

	while(1)
	{
		SYS_KickWatchdog();

		if(_ProcessSamples)
		{
			_ProcessSamples = false;
			uint32_t acc = 0;
			for(uint16_t i=0; i<MTR_BUFFLEN; i++)
				acc += _SamplesPointer[i];
			_AvgSample = acc / MTR_BUFFLEN;
		}

		if(!_ControllerState)
		{
			MTR_TurnOff();
			_MotorState=false;
		}
		else if(!_MotorState)
		{
			MTR_TurnOn();
			_MotorState=true;
		}

		//! TESTING PURPOSES!!!!
		if(_ControllerState==1) //standard mapa
		{
			LED_SetLedOn(LED3);
			LED_SetLedOff(LED2);
			LED_SetLedOff(LED1);
		}
		else if(_ControllerState==2) //res
		{
			LED_SetLedOn(LED2);
			LED_SetLedOff(LED3);
			LED_SetLedOff(LED1);
		}
		else if(_ControllerState==3) //mapa slow
		{
			LED_SetLedOn(LED2);
			LED_SetLedOn(LED3);
			LED_SetLedOff(LED1);
		}
		else if(_ControllerState==4) //mapa fast
				{
			LED_SetLedOff(LED2);
			LED_SetLedOff(LED3);
			LED_SetLedOn(LED1);
				}
		else {
			LED_SetLedOff(LED2);
			LED_SetLedOff(LED3);
			LED_SetLedOff(LED1);
		}
	}
}


//! Systick handler called in interrupt
void SYS_Tick(void)
{
	
	//! Read rpm value from encoder
	_Rpm = ENC_ReadAndReset();
	//! Calculate rpm to table index value
	if(_Rpm >=0) _TableIndex = _Rpm / 40;
	else _TableIndex = 0;
	if(_TableIndex >= 120) _TableIndex = 119;
	if(_ControllerState == 2)
	{
		MTR_SetLimit(_CurrentTable1[_TableIndex]);
	}
	else MTR_SetLimit(_CurrentTable[_TableIndex]);

	//! TODO: Handle events
	if(_MessageTimer > 10)
	{
		_ControllerState=0;
	}
	else _MessageTimer++;

	// send telemetry packet every 10ms.
	uint8_t len = MSG_PackDriverState(4 /*volt*/, 2 /*amp*/, 0 /*rpm*/, messageBuffer);
	BUS_Send(messageBuffer, len);
}


//! Samples handler called in interrupt
void MTR_SamplesReady(uint16_t *samples)
{
	_SamplesPointer = samples;
	_ProcessSamples = true;
	//	LED_ToggleLed(LED2);
}


void ENC_FullRotation(void)
{
	_Rotations++;
}


void MSG_Received(uint8_t *buff, uint8_t len)
{
	//1 to turbo lewa gora
	//3 to gaz lewa dol
	//2 reset
	//4 prawy dol
	_MessageTimer = 0;
	//! TODO: Do something with message
	if(buff[0]) _ControllerState = 2;
	else if(buff[2] || buff[3])
		_ControllerState = 1;
	else _ControllerState = 0;
}
