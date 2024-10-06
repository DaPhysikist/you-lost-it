/*
 * lsm6dsl.h
 *
 *  Created on: Nov 12, 2023
 *      Author: Christian Velasquez
 */

#ifndef LSM6DSL_H_
#define LSM6DSL_H_

#include <stdint.h>

void lsm6dsl_init();

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z);


#endif /* LSM6DSL_H_ */
