/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"

#include "stdio.h"
#include "can.h"
#include "usart.h"
#include "delay.h"
#include "NazaCanDecoderLib.h"
#include "stdlib.h"
#include "string.h"
#include <stdio.h>

/*Privat varibles*/
uint16_t messageId = 0;
uint16_t msCounter = 0;

/* Отправка в USART через printf*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART1, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}

	return ch;
}
/*------------------------------*/

/**
 * @brief
 * @brief
 * @note
 * @param  None
 * @retval None
 */
void RCC_Config(void)
{
	// Для настройки CAN в максимальном режиме работы на скорости до 1Mb нам необходимо
	// Настроить частоту перефирии APB1 на 16 MHz

	RCC_ClocksTypeDef RCC_Clocks;
	ErrorStatus HSEStartUpStatus;

	// Сбросим настройки тактирования системы
	// RCC system reset
	RCC_DeInit();

	// Включим внешний кварц, как источник сигнала
	// Enable HSE
	RCC_HSEConfig(RCC_HSE_ON);

	// Подождем включения HSE
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	// Если включился кварц
	if (HSEStartUpStatus == SUCCESS)
	{
		// Настроим тактирование так, как нам требуется
		// HCLK = SYSCLK     (64MHz)
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		// PCLK1 = HCLK / 4  (16MHz)
		RCC_PCLK1Config(RCC_HCLK_Div4);
		// PCLK2 = HCLK      (64MHz)
		RCC_PCLK2Config(RCC_HCLK_Div1);
		// ADC CLK
		RCC_ADCCLKConfig(RCC_PCLK2_Div2);

		// PLLCLK = 8MHz * 8 = 64 MHz
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_8);
		// Включаем PLL
		RCC_PLLCmd(ENABLE);

		// Ждем включения PLL
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{}

		// Выбираем PLL как источник системного тактирования
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		// Ждем, пока не установится PLL, как источник системного тактирования
		while (RCC_GetSYSCLKSource() != 0x08)
		{}
	}

	// Предназначен для отладки, проверяем, как настроены частоты устройства
	RCC_GetClocksFreq (&RCC_Clocks);
}

/**
 * @brief
 * @note
 * @param  None
 * @retval None
 */


int main(void)
{
	//	initialise_monitor_handles();
	// Настройка тактирования
	RCC_Config();

	Delay_Init();

	// Инициализации CAN-интерфеса
	Init_CAN();
	Init_USART();

	NazaCanDecoderLib_Begin();

	printf(" Hello.\r\nI am ready!\r\n");

	Heartbeat();

	while(1)
	{
		if (CAN_GetFlagStatus(CAN1, CAN_FLAG_FMP0) == SET)
		{
			messageId = NazaCanDecoderLib_Decode();
		}
		if(msCounter == 100)
		{
			printf("Pitch: %d Roll: %d\r\n", nazaDecode_getPitch(), nazaDecode_getRoll());

			// Display other data at 5Hz rate so every 200 milliseconds
			printf("Mode: ");
			switch (nazaDecode_getMode())
			{
			case MANUAL:
				printf("MAN\r\n");
				break;
			case GPS:
				printf("GPS\r\n");
				break;
			case FAILSAFE:
				printf("FS\r\n");
				break;
			case ATTI:
				printf("ATT\r\n");
				break;
			default:
				printf("UNK\r\n");
				break;
			}
			printf("Bat: %.3f V\r\n" , nazaDecode_getBattery()/1000.0);
			printf("Motor 1: %d\r\n", nazaDecode_getMotorOut(0));
			printf("Motor 2: %d\r\n", nazaDecode_getMotorOut(1));
			printf("Motor 3: %d\r\n", nazaDecode_getMotorOut(2));
			printf("Motor 4: %d\r\n", nazaDecode_getMotorOut(3));
			// Выдаст значение 0, так как подключено только 4 двигателя
			printf("Motor 5: %d\r\n", nazaDecode_getMotorOut(4));
			printf("Left Stick (horisontal): %d\r\n", nazaDecode_getRcIn(5));
			printf("Right Stick (horisontal): %d\r\n", nazaDecode_getRcIn(1));
			printf("Right Stick (vertical): %d\r\n", nazaDecode_getRcIn(2));
			printf("Left Stick (horisontal): %d\r\n", nazaDecode_getRcIn(3));
			printf("Alt: %.3f m\r\n", nazaDecode_getAlt());
			// Работает лишь при подключенном GPS
			printf("Date: %d.%d.%d\r\n", nazaDecode_getDay(), nazaDecode_getMonth(), nazaDecode_getYear());
			printf("Time: %d:%d:%d\r\n",nazaDecode_getHour(), nazaDecode_getMinute(),nazaDecode_getSecond());
			msCounter = 0;
		}
//		Heartbeat();
	}

}
