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

#include "can.h"

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
	uint16_t Count = 0;

	// Настройка тактирования
	RCC_Config();

	// Инициализации CAN-интерфеса
	Init_CAN();

	while(1)
	{
		// Èñïîëüçóåì ñ÷åò÷èê
		if (Count == 0x5)
		{
			// Åñëè äîøëè äî ìàêñèìóìà, òî îòïðàâèì ñîîáùåíèå â øèíó
			Count = 0;
			CAN_Send_Test();
		}
		else
		{
			// Èíà÷å - óâåëè÷èì ñ÷åò÷èê íà åäèíèöó
			Count++;
		}
	}

}
