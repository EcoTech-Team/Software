/*
 * system.c
 */


#include "system_driver.h"
#include "stm32f10x.h"
#include "hardware.h"


uint32_t _Tick;


void SYS_Init(void)
{
	//! CLOCK SETUP
	/* Prefetch 2 cycles for 72MHz clock */
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1;

	RCC->CFGR =	  //RCC_CFGR_MCO_0
			RCC_CFGR_PLLMULL9			/* PLL multiplicator is 9 */
			| RCC_CFGR_PLLSRC			/* PLL input is external oscillator */
			| RCC_CFGR_ADCPRE_DIV6 		/* ADC prescaler 6 */
			| RCC_CFGR_PPRE2_DIV1		/* APB2 prescaler 1 */
			| RCC_CFGR_PPRE1_DIV2		/* APB1 prescaler 2 */
			| RCC_CFGR_HPRE_DIV1;		/* AHB prescaler 1 */
	RCC->CR = RCC_CR_HSEON;         /* Enable external oscillator */

	/* Wait for locked external oscillator */
	RCC->CR |= 0x10 << 3;
	while((RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY);

	RCC->CR |= RCC_CR_PLLON;
	/* Wait for locked PLL */
	while((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY);

	RCC->CFGR &= ~0x00000003; /* clear */
	RCC->CFGR |= RCC_CFGR_SW_PLL;   /* SYSCLK is PLL */

	/* Wait for SYSCLK to be PPL */
	while((RCC->CFGR & RCC_CFGR_SW_PLL) != RCC_CFGR_SW_PLL);

	//! TODO: Enable all used peripheral clocks
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_ADC1EN |
					RCC_APB2ENR_TIM1EN | RCC_APB2ENR_AFIOEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_USART3EN;
	#ifdef BUS_TIM_USED
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	#endif
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;


	IWDG->KR = 0x5555;
	IWDG->PR |= IWDG_PR_PR_0;
	IWDG->KR = 0xAAAA;

	SysTick_Config(720000);
}


uint32_t SYS_GetTick(void)
{
	return _Tick;
}


void SYS_KickWatchdog(void)
{
	IWDG->KR = 0xAAAA;
}


void SysTick_Handler(void)
{
	_Tick++;
	SYS_Tick();
}


