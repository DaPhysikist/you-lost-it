/*
 * leds.h
 *
 *  Created on: Oct 3, 2023
 *      Author: schulman
 */

#ifndef LEDS_H_
#define LEDS_H_

#include <stdint.h>

void leds_init();

//This function will set the two user LEDs on or off
//input parameter led is 2-bit bitmask in the byte that corresponds to each of the leds
void leds_set(uint8_t led);

#endif /* LEDS_H_ */
