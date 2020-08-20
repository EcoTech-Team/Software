/*
 * motor_driver.c
 *
 *  Created on: Jun 5, 2019
 *      Author: pawel
 */


#include <stdbool.h>
#include "motor_driver.h"


MTR_Samples _Samples;
uint16_t _HighLimit;
uint16_t _LowLimit;
bool _ON = true;


void MTR_Init(void)
{
	/*!
	 * MTR_HIN = GPIOA_PIN8
	 * MTR_LIN = GPIOB_PIN_13
	 *
	 * CNF1		CNF0	MODE1	MODE0	ODR
	 * 1		0		1		0		0
	 *
	 * Alternate function push-pull
	 */
	MTR_HIN_GPIO->CRH &= ~GPIO_CRH_CNF8;
	MTR_HIN_GPIO->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8_1;
	MTR_LIN_GPIO->CRH &= ~GPIO_CRH_CNF13;
	MTR_LIN_GPIO->CRH |= GPIO_CRH_CNF13_1 | GPIO_CRH_MODE13_1;

	//! Timer setup
	//! PWM mode 1
	MTR_TIM->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	//! C1 and C1N outputs enable
	MTR_TIM->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;
	//! Set dead time
	MTR_TIM->BDTR = MTR_DT;
	MTR_TIM->ARR = MTR_TIM_ARR;
	MTR_TIM->CR1 |= TIM_CR1_CEN;

	/*!
	 * MTR_IIN = GPIO_PIN_2
	 *
	 * CNF1		CNF0	MODE1	MODE0	ODR
	 * 0		0		0		0		0
	 *
	 * Analog mode
	 */
	//! Fix wrong connections: pins 12 and 13 into analog mode
	//MTR_IIN_GPIO->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12 | GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
	MTR_IIN_GPIO->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2); 

	MTR_DMA->CMAR = (uint32_t) &_Samples;
	MTR_DMA->CPAR = (uint32_t) &MTR_ADC->DR;
	MTR_DMA->CNDTR = MTR_BUFFLEN * 2;
	MTR_DMA->CCR |= DMA_CCR1_MSIZE_0 | DMA_CCR1_PSIZE_0 | DMA_CCR1_MINC |
			DMA_CCR1_CIRC | DMA_CCR1_HTIE | DMA_CCR1_TCIE | DMA_CCR1_EN;

	MTR_ADC->CR1 |= ADC_CR1_AWDIE;
	MTR_ADC->CR2 |= ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_2 | ADC_CR2_DMA | ADC_CR2_ADON;
	MTR_ADC->SQR3 |= ADC_SQR3_SQ1_3;
	MTR_ADC->SMPR2 |= ADC_SMPR2_SMP8_2;
	MTR_ADC->LTR = 0;
	MTR_ADC->HTR = 0xFFF;

	//! Wait for two ADC clocks before calibration
	uint16_t i = 0xFFF;
	while(i > 0) i--;
	//! Calibrate ADC
	MTR_ADC->CR2 |= ADC_CR2_CAL;
	//! Watchdog should reset if calibration fails
	while(MTR_ADC->CR2 & ADC_CR2_CAL);

	MTR_ADC_TIM->ARR = MTR_ADC_ARR;
	MTR_ADC_TIM->CR2 |= TIM_CR2_MMS_1;
	MTR_ADC_TIM->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	NVIC_EnableIRQ(ADC1_2_IRQn);
}


void MTR_TurnOn(void)
{
	_ON = true;
	MTR_TIM->CCR1 = MTR_TIM_ON;
	MTR_TIM->BDTR |= TIM_BDTR_MOE;
	MTR_ADC->HTR = 0xFFFF;
	MTR_ADC->LTR = _LowLimit;
	MTR_ADC->CR1 |= ADC_CR1_AWDEN;
}


void MTR_TurnOff(void)
{
	MTR_TIM->BDTR &= ~TIM_BDTR_MOE;
	MTR_ADC->CR1 &= ~ADC_CR1_AWDEN;
}


void MTR_SetLimit(uint16_t limit)
{
	_LowLimit = limit - MTR_HIST;
	_HighLimit = limit + MTR_HIST;
	if(_ON)
	{
		MTR_ADC->LTR = _LowLimit;
		MTR_ADC->HTR = 0xFFFF;
	}
	else
	{
		MTR_ADC->HTR = _HighLimit;
		MTR_ADC->LTR = 0;
	}
}


void ADC1_2_IRQHandler(void)
{
	if(MTR_ADC->SR & ADC_SR_AWD)
	{
		if(_ON)
		{
			//! Turn on low transistor
			MTR_TIM->CCR1 = MTR_TIM_OFF;
			MTR_ADC->LTR = 0;
			MTR_ADC->HTR = _HighLimit;
			_ON = false;
		}
		else
		{
			//! Turn on high transistor
			MTR_TIM->CCR1 = MTR_TIM_ON;
			MTR_ADC->HTR = 0xFFFF;
			MTR_ADC->LTR = _LowLimit;

			_ON = true;
		}
		MTR_ADC->SR &= ~ADC_SR_AWD;
	}
}


void DMA1_Channel1_IRQHandler(void)
{
	if(DMA1->ISR & DMA_ISR_HTIF1)
	{
		DMA1->IFCR |= DMA_IFCR_CHTIF1;
		MTR_SamplesReady(_Samples.A);
	}
	if(DMA1->ISR & DMA_ISR_TCIF1)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF1;
		MTR_SamplesReady(_Samples.B);
	}
}

