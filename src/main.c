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
		// Display attitude at 10Hz rate so every 100 milliseconds
		if(msCounter == 1000)
		{
			printf("Pitch: %.3f Roll: %.3f\r\n", pitchRad * 57.295779513, rollRad * 57.295779513);

			// Display other data at 5Hz rate so every 200 milliseconds

			printf("Mode: ");
			switch (mode)
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

			printf("Bat: %.3f V\r\n" , battery/1000.0);
			printf("Motor 1: %d\r\n", motorOut[0]);
			printf("Motor 2: %d\r\n", motorOut[1]);
			printf("Motor 3: %d\r\n", motorOut[2]);
			printf("Motor 4: %d\r\n", motorOut[3]);
			printf("rcIn 1: %d\r\n", rcIn[0]);
			printf("rcIn 2: %d\r\n", rcIn[1]);
			printf("rcIn 3: %d\r\n", rcIn[2]);
			printf("rcIn 4: %d\r\n", rcIn[3]);
			printf("Alt: %.3f m\r\n", alt);

			msCounter = 0;
		}

		Heartbeat();
	}

}
