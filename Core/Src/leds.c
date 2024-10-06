/*
 * leds.c
 *
 *  Created on: Oct 3, 2023
 *      Author: schulman
 */


/* Include memory map of our MCU */
#include "leds.h"
#include "../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l475xx.h"

void leds_init()
{
	  // Enable BUS that GPIOA USES
	  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	  //Configure PA5 as an output by clearing all bits and setting the mode
	  GPIOA->MODER &= ~GPIO_MODER_MODE5;
	  GPIOA->MODER |= GPIO_MODER_MODE5_0;

	  // Configure the GPIO output as push pull (transistor for high and low)
	  GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;

	  /* Disable the internal pull-up and pull-down resistors */
	  GPIOA->PUPDR &= GPIO_PUPDR_PUPD5;

	  /* Configure the GPIO to use low speed mode */
	  GPIOA->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED5_Pos);

	  /* Turn off the LED */
	  GPIOA->ODR &= ~GPIO_ODR_OD5;

	  // SETUP FOR LED2
	  // Enable BUS that GPIOB USES
	  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	  /* Configure PB14 as an output by clearing all bits and setting the mode */
	  GPIOB->MODER &= ~GPIO_MODER_MODE14;
	  GPIOB->MODER |= GPIO_MODER_MODE14_0;

	  /* Configure the GPIO output as push pull (transistor for high and low) */
	  GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;

      /* Disable the internal pull-up and pull-down resistors */
	  GPIOB->PUPDR &= GPIO_PUPDR_PUPD14;

	  /* Configure the GPIO to use low speed mode */
	  GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED14_Pos);

	  /* Turn off the LED */
	  GPIOB->ODR &= ~GPIO_ODR_OD14;
}

void leds_set(uint8_t led)
{
	GPIOA->ODR &= ~GPIO_ODR_OD5;  //Clear both LEDs
	GPIOB->ODR &= ~GPIO_ODR_OD14;
	if (led == 0b01){ // Turns on LED1 and turns off LED2
		GPIOA->BSRR |= GPIO_BSRR_BS5;//(1<<5); BIT SET 5  --LED1
		GPIOB->BSRR |= GPIO_BSRR_BR14;// (0<<14) BIT RESET 14 --LED2
	}
	else if (led == 0b10){				// Turns off LED1 and turns on LED2
		GPIOA->BSRR |= GPIO_BSRR_BR5;//(0<<5); BIT RESET 5
		GPIOB->BSRR |= GPIO_BSRR_BS14; //(1<<14); BIT SET 14
	}
	else if (led == 0b11) {				// Turns on LED1 and turns on LED2
		GPIOA->BSRR |= GPIO_BSRR_BS5;   //(1<<5);
		GPIOB->BSRR |= GPIO_BSRR_BS14;   //(1<<14);
	}
	else if (led == 0b00) {			//Turns both LEDs off
		GPIOA->BSRR |= GPIO_BSRR_BR5;   //(0<<5);
		GPIOB->BSRR |= GPIO_BSRR_BR14;  //(0<<14);
	}
}
