/*
 * i2c.c
 *
 *  Created on: Nov 4, 2023
 *      Authors: Christian Velasquez and Pranav Mehta
 */

/* Memory map of our MCU */
#include "i2c.h"
#include "../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l475xx.h"


void i2c_init(){
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN; //Connect I2C2 bus to clock tree
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; //Connect pins used for I2C to clock tree

	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL10_2; //Select the alternate function as I2C2 (AF4)
	GPIOB->OTYPER |= GPIO_OTYPER_OT10; //configure the pin as open drain
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0; //Configure pin as pull up
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED10; //Configure the GPIO to use high speed mode
	GPIOB->MODER &= ~GPIO_MODER_MODE10; //Clear the bits used to determine the mode of the pin
	GPIOB->MODER |= GPIO_MODER_MODE10_1; //Configure pin 10 in alternate function mode by setting the mode

	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL11_2; //Select the alternate function as I2C2 (AF4)
	GPIOB->OTYPER |= GPIO_OTYPER_OT11; //configure the pin as open drain
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD11_0; //Configure pin as pull up
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED11; //Configure the GPIO to use high speed mode
	GPIOB->MODER &= ~GPIO_MODER_MODE11; //Clear the bits used to determine the mode of the pin
	GPIOB->MODER |= GPIO_MODER_MODE11_1; //Configure pin 11 in alternate function mode by setting the mode

	I2C2->CR1 &= ~I2C_CR1_PE; //clear PE bit to allow change of timing configuration
	I2C2->TIMINGR |= ((1 << I2C_TIMINGR_PRESC_Pos) | (0x4 << I2C_TIMINGR_SCLDEL_Pos) | (0x2 << I2C_TIMINGR_SDADEL_Pos) | (0xF << I2C_TIMINGR_SCLH_Pos) | (0x13 << I2C_TIMINGR_SCLL_Pos)); //setup I2C to use 100 kHz baud rate from 8 MHz clock

	I2C2->CR1 |= I2C_CR1_PE; //Enable I2C
}


uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len){
	if (dir == 0){  //transmit message
		I2C2->CR2 = (len << I2C_CR2_NBYTES_Pos | dir << I2C_CR2_RD_WRN_Pos | address << I2C_CR2_SADD_Pos | I2C_CR2_AUTOEND | I2C_CR2_START);  //begin transmitting with transmit parameters
		for(int i = 0; i < len; i++){            //For loop to go through each byte to be transmitted in message
			while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));   //Waits for either NACKF or TXIS flags to be set to determine if can transmit, or if there was an error
			if (I2C2->ISR & I2C_ISR_NACKF){        //Error in transmission
				return 1;
			}
			else if (I2C2->ISR & I2C_ISR_TXIS){      //TXIS flag set, free to transmit one byte over i2c
				I2C2->TXDR = data[i];                 //Transmit one byte over i2c
			}
		}
		return 0;      //Transmit completed successfully
	}
	else if (dir == 1){   //receive message
		I2C2->CR2 = (((0x01) << I2C_CR2_NBYTES_Pos) | (0x00) << I2C_CR2_RD_WRN_Pos | address << I2C_CR2_SADD_Pos | I2C_CR2_START);  //begin writing subaddress with parameters for writing subaddress, 0x01 because writing one byte and 0x00 because writing
		while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));   //Waits for either NACKF or TXIS flags to be set to determine if can write, or if there was an error
		if (I2C2->ISR & I2C_ISR_NACKF){        //Error in writing subaddress
			return 1;
		}
		else if (I2C2->ISR & I2C_ISR_TXIS){      //TXIS set, free to write subaddress
			I2C2->TXDR = data[0];                //Write subaddress
			while(!(I2C2->ISR & I2C_ISR_TC));    //Wait for TC to ensure subaddress was written
		}

		address |= (1U);                                   //Change address from writing to reading
		I2C2->CR2 = ((len - 1) << I2C_CR2_NBYTES_Pos | dir << I2C_CR2_RD_WRN_Pos | address << I2C_CR2_SADD_Pos | I2C_CR2_AUTOEND | I2C_CR2_START);  //begin reading with read parameters, len-1 because the first byte was the subaddress
		for(int i = 1; i < len; i++){   //For loop to go through every byte that needs to be read to the buffer, i starts at 1 because the first byte in the array was the subaddress and the remaining are for the buffer
			while(!(I2C2->ISR & I2C_ISR_RXNE));     //Waits for RXNE flag to be set to determine if can read
			data[i] = I2C2->RXDR;                //Reads data one byte at a time and stores it in the buffer
		}
		return 0;    //Read operation completed successfully
	}
	return 1;        //Error in transaction
}


