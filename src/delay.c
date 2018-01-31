/*
 * delay.c
 *
 *  Created on: 31 янв. 2018 г.
 *      Author: user
 */
#include "delay.h"

uint32_t TimingDelay = 0;

void Delay_Init(void)
{
	SysTick_Config(SystemCoreClock / 1000);
}

void SysTick_Handler(void)
{
	if (TimingDelay != 0x00)
		TimingDelay--;

}

void delay_ms(uint32_t milliseconds)
{
	TimingDelay = milliseconds;
	while(TimingDelay != 0);
}
