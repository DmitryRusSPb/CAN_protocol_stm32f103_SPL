/*
 * usart.h
 *
 *  Created on: 2 февр. 2018 г.
 *      Author: user
 */

#ifndef USART_H_
#define USART_H_

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "misc.h"

void USARTSend(uint8_t* txBuffer, uint8_t lenBuf);
void Init_USART(void);
void clear_RXBuffer(void);
void SetSysClockTo72(void);

#define RX_BUF_SIZE 80
#define TX_BUF_SIZE 40

volatile uint8_t RX_FLAG_END_LINE;
volatile uint8_t RXi;
volatile uint8_t RXc;
volatile uint8_t RX_BUF[RX_BUF_SIZE];
uint8_t txBuff[TX_BUF_SIZE];

#endif /* USART_H_ */
