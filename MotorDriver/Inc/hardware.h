/*
 * hardware.h
 */

#ifndef HARDWARE_H_
#define HARDWARE_H_


#include "stm32f10x.h"


//! ########################### GPIO DEFINES ########################### 

//! ########################### PIN DEFINITION #######################
#define GPIO_PIN_0              ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1              ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2              ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3              ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4              ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5              ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6              ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7              ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8              ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9              ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10             ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11             ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12             ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13             ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14             ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15             ((uint16_t)0x8000)  /* Pin 15 selected   */

//! ########################### LEDS ###########################
#define LED_GPIO	GPIOA
#define LED_1_PIN	GPIO_PIN_5
#define LED_2_PIN	GPIO_PIN_4
#define LED_3_PIN	GPIO_PIN_3

//! ########################### BUS ########################### 
#define BUS_GPIO	GPIOB
#define BUS_TX_PIN	GPIO_PIN_10
#define BUS_RX_PIN	GPIO_PIN_11
#define BUS_UART	USART3
#define BUS_TIM     TIM2
//#define BUS_TIM_USED  // define to use TIMER

//! ########################### ENCODER ########################### 
#define ENC_GPIO	GPIOB
#define ENCA_PIN	GPIO_PIN_7
#define ENCB_PIN	GPIO_PIN_6
#define ENCI_PIN	GPIO_PIN_5
#define ENC_TIM		TIM4

//! ########################### INPUT ADC ###########################
#define ADC_GPIO	GPIOA
#define ADC_VIN_PIN	GPIO_PIN_12

//! ########################### DRIVER ###########################
#define MTR_HIN_GPIO	GPIOA
#define MTR_HIN_PIN		GPIO_PIN_8
#define MTR_LIN_GPIO	GPIOB
#define MTR_LIN_PIN		GPIO_PIN_13
#define MTR_IIN_GPIO	GPIOB
#define MTR_IIN_PIN		GPIO_PIN_0
#define MTR_TIM			TIM1
#define MTR_ADC			ADC1
#define MTR_ADC_TIM		TIM3
#define MTR_DMA			DMA1_Channel1

//! ########################### BUTTON ###########################
#define BTN_GPIO		GPIOA
#define BTN_PIN			GPIO_PIN_15

//! ########################### DEBUG? #######################
#define SWDIO_GPIO                              GPIOA
#define SWDIO_Pin                               GPIO_PIN_13

#define SWCLK_GPIO                              GPIOA
#define SWCLK_Pin                               GPIO_PIN_14


#endif
