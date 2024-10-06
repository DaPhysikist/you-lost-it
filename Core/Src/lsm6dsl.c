/*
 * lsm6dsl.c
 *
 *  Created on: Nov 12, 2023
 *      Author: Christian Velasquez
 */

#include "lsm6dsl.h"
#include "i2c.h"
#include "../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/stm32l475xx.h"

uint8_t SLAVE_ADDR = (0xD4);  //0b11010100, given by the datasheet for the accelerometer
uint8_t XLDA = (0x01);  //bit in STATUS register that indicates that there are new accelerometer values to be read

void lsm6dsl_init(){
	/*
	GPIOD->MODER &= ~GPIO_MODER_MODE11; //Clear the bits used to determine the mode of the pin, sets the pin to input mode (00)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;       //need to setup syscfg clock to enable syscfg peripheral for interrupt config
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PD;  //setup port D for the interrupt (PD11)
	EXTI->IMR1 |= EXTI_IMR1_IM11;  //setup pin 11 as interrupt
	EXTI->RTSR1 |= (EXTI_RTSR1_RT11);  //enable rising edge interrupt (low->high)
	EXTI->FTSR1 &= ~(EXTI_FTSR1_FT11);  //disable falling edge interrupt (high->low)
	NVIC_SetPriority(EXTI15_10_IRQn, 0);  //setup interrupt in NVIC
	NVIC_EnableIRQ(EXTI15_10_IRQn);*/

	uint8_t acc1[] = {0x10, 0x60};    //configure accelerometer at 104 Hz
	i2c_transaction(SLAVE_ADDR, 0, acc1, 2);

	uint8_t acc2[] = {0x0D, 0x01};    //set accelerometer to count inactivity for ~60 secs (59.08)
	i2c_transaction(SLAVE_ADDR, 0, acc2, 2);

	/*
	uint8_t acc5[] = {0x58, 0xE0};    //inactivity configuration -> acc to 12.5 LP and enable slope filter
	i2c_transaction(SLAVE_ADDR, 0, acc5, 2);

	uint8_t acc3[] = {0x5C, 0x02};    //set accelerometer to count inactivity for ~60 secs (59.08)
	i2c_transaction(SLAVE_ADDR, 0, acc3, 2);

	uint8_t acc4[] = {0x5B, 0x02};    //set inactivity/activity threshold to 0.5g (16*2g/2^6) -> which is close to our previously configured threshold of 8000
	i2c_transaction(SLAVE_ADDR, 0, acc4, 2);

	uint8_t acc6[] = {0x5E, 0x80};    //inactivity/activity interrupt driven on INT1 pin
	i2c_transaction(SLAVE_ADDR, 0, acc6, 2);*/
}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z){
	int status = 0;  //variable used in loop for checking for new accelerometer values
	while (status == 0){       //loop to check for new accelerometer values
		uint8_t statusReceive[2] = {0x1E};  //two byte array, first byte is address of STATUS register and second is buffer to store its value
		i2c_transaction(SLAVE_ADDR, 1, statusReceive, 2);  //receive STATUS register value and store it in buffer
		if (statusReceive[1] & XLDA){        //If the XLDA bit is set in the STATUS register, there are new accelerometer values to be read
			status = 1;                   //Exit loop so accelerometer values can now be read
		}
	}

	uint8_t readX_L[2] = {0x28};  //two byte array, first byte is address of X_L register and second is buffer to store its value
	i2c_transaction(SLAVE_ADDR, 1, readX_L, 2);  //receive X_L value and store in in buffer

	uint8_t readX_H[2] = {0x29};  //two byte array, first byte is address of X_H register and second is buffer to store its value
	i2c_transaction(SLAVE_ADDR, 1, readX_H, 2);  //receive X_H value and store it in buffer

	uint8_t readY_L[2] = {0x2A};  //two byte array, first byte is address of Y_L register and second is buffer to store its value
	i2c_transaction(SLAVE_ADDR, 1, readY_L, 2);  //receive Y_L value and store it in buffer

	uint8_t readY_H[2] = {0x2B};  //two byte array, first byte is address of Y_H register and second is buffer to store its value
	i2c_transaction(SLAVE_ADDR, 1, readY_H, 2); //receive Y_H value and store it in buffer

	uint8_t readZ_L[2] = {0x2C};  //two byte array, first byte is address of Z_L register and second is buffer to store its value
	i2c_transaction(SLAVE_ADDR, 1, readZ_L, 2);  //receive Z_L value and store it in buffer

	uint8_t readZ_H[2] = {0x2D}; //two byte array, first byte is address of Z_H register and second is buffer to store its value
	i2c_transaction(SLAVE_ADDR, 1, readZ_H, 2); //receive Z_H value and store it in buffer

	*x = ((readX_H[1] << 8) | readX_L[1]);  //concatenate values of X_H and X_L stored in the buffers to get value of x and store it in memory
	*y = ((readY_H[1] << 8) | readY_L[1]);  //concatenate values of Y_H and Y_L stored in the buffers to get value of y and store it in memory
	*z = ((readZ_H[1] << 8) | readZ_L[1]);  //concatenate values of Z_H and Z_L stored in the buffers to get value of z and store it in memory
}
