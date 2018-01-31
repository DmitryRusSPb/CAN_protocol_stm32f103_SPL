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
#include "delay.h"
#include "NazaCanDecoderLib.h"


/*Privat varibles*/
uint16_t messageId = 0;
uint16_t msCounter = 0;

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

	NazaCanDecoderLib_Begin();


	while(1)
	{
		messageId = NazaCanDecoderLib_Decode();

		if(messageId)
		{
//			printf("Message ");
//			printf("%X", messageId);
//			printf(" decoded");
		}

//		// Display attitude at 10Hz rate so every 100 milliseconds
//		if(msCounter == 100)
//		{
//			printf("Pitch: ");
//			printf("%d", pitch);
//			printf(", Roll: ");
//			printf("%d", roll);
//		}
//
//		// Display other data at 5Hz rate so every 200 milliseconds
//		if(msCounter == 200)
//		{
//			printf("Mode: ");
//			switch (mode)
//			{
//			case MANUAL:
//				printf("MAN");
//				break;
//			case GPS:
//				printf("GPS");
//				break;
//			case FAILSAFE:
//				printf("FS");
//				break;
//			case ATTI:
//				printf("ATT");
//				break;
//			default:
//				printf("UNK");
//				break;
//			}
//
//			printf(", Bat: ");
//			printf("%10.2G", battery/1000.0);
//
//			printf("Lat: ");
//			printf("%10.7G", lat);
//			printf(", Lon: ");
//			printf("%10.7G", lon);
//			printf(", GPS alt: ");
//			printf("%10G", gpsAlt);
//			printf(", COG: ");
//			printf("%10G", cog);
//			printf(", Speed: ");
//			printf("%10G", spd);
//			printf(", VSI: ");
//			printf("%10G", vsi);
//			printf(", Fix: ");
//
//			switch (fix)
//			{
//			case NO_FIX:
//				printf("No fix");
//				break;
//			case FIX_2D:
//				printf("2D");
//				break;
//			case FIX_3D:
//				printf("3D");
//				break;
//			case FIX_DGPS:
//				printf("DGPS");
//				break;
//			default:
//				printf("UNK");
//				break;
//			}
//
//			printf(", Sat: ");
//			printf("%d \n", sat);
//
//			printf("Alt: ");
//			printf("%10G", alt);
//			printf(", Heading: ");
//			printf("%10G \n", heading);
//		}
//
//		// Display date/time at 1Hz rate so every 1000 milliseconds
//		if(msCounter == 1000)
//		{
//			uint32_t dateAndTime[6];
//			sprintf(dateAndTime, "%4u.%02u.%02u %02u:%02u:%02u",
//					year + 2000, month, day,
//					hour, minute, second);
//			printf("Date/Time: ");
//			printf(dateAndTime);
//		}

		Heartbeat();
	}

}
