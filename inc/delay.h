/*
 * delay.h
 *
 *  Created on: 31 янв. 2018 г.
 *      Author: user
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f10x.h"

void Delay_Init(void);

void delay_ms(uint32_t milliseconds);

#endif /* DELAY_H_ */
