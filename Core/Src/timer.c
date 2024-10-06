/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Authors: Pranav Mehta and Christian Velasquez
 */

#include "timer.h"

/*
 * Setup TIM2
 *
 */
void timer_init(TIM_TypeDef* timer)
{
	uint32_t clockEnable;     //Determine IRQn, priority and peripheral clock enable values based on timer given
	int IRQn;
	uint8_t priority;
	if (timer == TIM2){
		clockEnable = RCC_APB1ENR1_TIM2EN;
		IRQn = TIM2_IRQn;
		priority = 2;
	}
	else if (timer == TIM3) {
		clockEnable = RCC_APB1ENR1_TIM3EN;
		IRQn = TIM3_IRQn;
		priority = 3;
	}
	else { return; }

	RCC->APB1ENR1 |= clockEnable; //connect timer to clock tree

	//Stop timer and clear out state
	timer->CR1 &= ~(TIM_CR1_CEN);
	timer->CR1 &= ~(TIM_CR1_DIR); // set to upcount timer mode

	//Reset counter
	timer_reset(timer);

	//Divide by prescaler to get correct time
	timer->PSC = 7;  //Dividing by 8 (7 because prescaler is of by one due to being zero-based) (from 8 MHz -> 1 MHz)

	//Enable timer interrupt internally
	timer->DIER |= TIM_DIER_UIE;

	//Enable timer interrupt in interrupt controller
	NVIC_SetPriority(IRQn, priority);
	NVIC_EnableIRQ(IRQn);

	//enable counter
	timer->CR1 |= TIM_CR1_CEN;
}

void timer_reset(TIM_TypeDef* timer)
{
	//Reset the counter
	timer->CNT  = 0;
}


void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
	//Set the reload value (counter counts up to this then resets)
	// *1000 because counter is counting microseconds.
	timer->ARR = period_ms*1000;
	//Reset the counter
	timer_reset(timer);
}
