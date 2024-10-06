/*
 * i2c.h
 *
 *  Created on: Nov 4, 2023
 *      Author: Christian Velasquez
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

/**
 * Configures/Enables I2C peripheral
 *
 * Communicates in master mode, @BAUD RATE = 'RATEHERE'
 */
void i2c_init();

/**
 * 	Perform transaction to R/W bytes to/from MCU(primary) to any
 *  I2C peripheral (secondary) w/ secondary **address** parameter
 *
 *  If dir = 0, it is WRITING to secondary; if 1, it is READING FROM secondary
 *  i2c_transaction is blocking, should return only when the entire transaction
 *  has completed
 */

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len);


#endif /* I2C_H_ */
