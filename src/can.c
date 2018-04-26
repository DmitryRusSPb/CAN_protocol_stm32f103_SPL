/**
 *****************************************************************************
 * @title		can.c
 * @platform	STM32F103
 * @author		Dmitry Ovcharenko (https://www.smartmode.info/)
 * @version	V1.1.0
 * @date		07.07.2016
 *
 * @brief		������ CAN - �������������, �����-��������
 *
 *******************************************************************************
 * @attention
 *
 *
 * <h2><center>&copy; COPYRIGHT 2016 SmartMODE</center></h2>
 */

#include "can.h"
#include "NazaCanDecoderLib.h"

uint8_t num;
uint8_t errcnt = 0;

//extern uint16_t messageId;

void Init_CAN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* CAN GPIOs configuration */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(CAN1_Periph, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);


	GPIO_InitStructure.GPIO_Pin   = CAN1_RX_SOURCE;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin   = CAN1_TX_SOURCE;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

#ifdef CAN1_ReMap
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
#endif

	// Структура для инициализации CAN
	CAN_InitTypeDef CAN_InitStructure;

	// Отключаем CAN для его настройки
	CAN_DeInit(CAN1);
	// Заполняет все значения структуры CAN_InitStructure значениями по умолчанию
	CAN_StructInit(&CAN_InitStructure);

	// CAN cell init
	/* Включение или отключение режима Time Triggered Mode */
	/* Этот параметр влияет на включение таймера.
	 * Внутренний 16-битный таймер используется для привязки метки времени
	 * к каждому принятому и отправленному сообщению. Этот таймер начинает
	 * счет с момента разрешения работы контроллера CAN. Этот счетчик может
	 * быть сброшен приложением или автоматически после приема в последний
	 * mailbox, когда установлен Time Triggered Mode */
	CAN_InitStructure.CAN_TTCM = DISABLE;

	/* Включение или отключение автоматического пробуждения устройства по
	 * сигналу с CAN-шины. */
	/* Если режим включен, то при накоплении ошибок приема данных из шины,
	 * CAN автоматически будет отключен.В любом случае необходимо контролировать
	 * ошибки приема, содержимое почтовых ящиков и, при необходимости сбрасывать
	 * ошибки вручную. */
	CAN_InitStructure.CAN_ABOM = DISABLE;

	/* Включение или отключение автоматического пробуждения устройства
	 * по сигналу с CAN-шины.  */
	/* При включении этого параметра устройство будет автоматически просыпаться,
	 * но следует обратить внимание что на активацию устройства требуется
	 * некоторое время и первый пакет, переданный по шине, может быть утерян. */
	CAN_InitStructure.CAN_AWUM = DISABLE;

	/* Включение или отключение режима проверки получения пакета. */
	/* Если этот режим включен, то микроконтроллер при передаче кадра не будет
	 * проверять подтверждение получения пакета всеми устройством на шине.
	 * Если выключен - то, при передаче кадра, микроконтроллер будет слушать
	 * шину на предмет получения подтверждения от всех устройства о том,
	 * что пакет получен и, если не получен хотя бы одним устройтсвом, то будет
	 * пытаться отправить повторно до тех пор, пока все устройства не подтвердят
	 * получение пакета. Другими словами: При выключенном параметре передал
	 * и забыл, при включенном - передал и проверил, если не смог отправить,
	 * то пытается еще раз, пока не сможет отправить.Если устройство не подключено
	 * к шине или если неправильно настроены тайминги, то при включении этого
	 * параметра контроллер будет бесконечно пытаться отправлять пакет в шину. */
	CAN_InitStructure.CAN_NART = ENABLE;

	/* Включение или отключение режима блокировки Receive FIFO */
	/* 0 - при переполнении RX_FIF0 прием не прерывается
	 * (3 сообщения до заполнения FIFO), каждый новый пакет затирает предыдущий
	 * 1 - при переполнении RX_FIF0 прием прерывается
	 * (3 сообщения до заполнения FIFO), новые пакеты отбрасываются
	 * до освобождения RX_FIFO. */
	CAN_InitStructure.CAN_RFLM = DISABLE;

	/* Включение или отключение приоритета передачи FIFO */
	/* Включение этого параметра определяет, в каком порядке сообщения будут
	 * отправляться в шину. Если параметр включен, то сообщения отправляются
	 * в хронологическом порядке:
	 * FIFO - First Input First Output - Первый пришел - первым ушел.
	 * Если же выключен, то пакеты передаются в зависимости от приоритета ID пакета.
	 * Т.е. пакет с более высоким приоритетом будет отправлен раньше. */
	CAN_InitStructure.CAN_TXFP = DISABLE;

	/* Определяет в каком режиме контроллер будет работать с CAN-шиной */
	/* CAN_Mode_Norma - При этом параметре МК будет работать в обычном (нормальном)
	 * режиме работы. Данные будут передаваться и читаться из шины.
	 * CAN_Mode_LoopBack - При выборе режима LoopBack, контроллер будет передавать
	 * данные в шину и слушать себя же одновременно. Это равносильно тому,
	 * что в USART мы бы замкнули ножку TX на ножку RX. Но пакеты из шины доходить
	 * до контроллера не будут.
	 * CAN_Mode_Silent - Идеален при настройке устройств, которым нужно только
	 * слушать шину. Например, если нам нужно подключиться к CAN шине автомобиля,
	 * но мы боимся допустить какие либо сбои из-за неправильной отправки пакетов
	 * в шину, то этот режим будет идеальным, так как в шину автомобиля пакеты из
	 * устройства попадать не будут.
	 * CAN_Mode_Silent_LoopBack - В данном режиме все пакеты будут полностью
	 * крутится внутри контроллера без выхода в общую шину.Из шины соответственно
	 * ни один пакет данных не дойдет до устройства.Этот режим идеален для отладки
	 * устройства. При включении его мы можем как передать данные в шину, так и
	 * обработать те данные, которые мы же и отправили, при этом не имея физического
	 * подключения к шине.*/
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	//	CAN_InitStructure.CAN_Mode = CAN_Mode_Silent_LoopBack;

	/* CAN Baudrate = 1MBps*/
	/* Определяет максимальное количество квантов времени, на которое может быть
	 * увеличено или уменьшено количество квантов времени битовых сегментов.
	 * Возможные значения этого показателя от 1-го до 4-х квантов. */
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;

	/* Определяет местоположение точки захвата (Sample Point). Он включает в себя
	 * Prop_Seg и PHASE_SEG1 стандарта CAN. Его продолжительность программируется
	 * от 1 до 16 квантов времени. */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;
	/* Определяет местоположение точки передачи. Он представляет собой PHASE_SEG2
	 * стандарта CAN. Его продолжительность программируется от 1 до 8 квантов
	 * времени.*/
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	/* Множитель, из значения которого рассчитывается размер кванта времени.
	 * Рассчитывается исходя от частоты работы периферии микроконтроллера. */
	CAN_InitStructure.CAN_Prescaler = CAN1_SPEED_PRESCALE;

	CAN_Init(CAN1, &CAN_InitStructure);

	//	// CAN filter init
	//	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	//	// Номер фильтра, доступны с 0 по 13
	//	CAN_FilterInitStructure.CAN_FilterNumber = 1;
	//	// Режим работы фильтра
	//	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	//	// Разрядность (масштабирование)
	//	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	//	// Старшая часть ID
	//	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	//	// Младшая часть ID
	//	CAN_FilterInitStructure.CAN_FilterIdHigh =  0x0000;
	//	// Старшая часть маски ID
	//	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	//	// Младшая часть маски ID
	//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	//	// Имя буфера FIFO (у нас их всего два)
	//	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	//	// Состояние фильтра
	//	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	//	CAN_FilterInit(&CAN_FilterInitStructure);

#ifdef USB_HP_CAN1_TX_IRQHandler_ENABLE
	// CAN Transmit mailbox empty Interrupt enable
	// Обрабатывается в прерывании USB_HP_CAN1_TX_IRQHandler
	// Прерывание при освобождении исходящего почтового ящика
	CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);
#endif
#ifdef USB_LP_CAN1_RX0_IRQHandler_ENABLE
	// CAN Receive Interrupt enable
	// Обрабатывается в прерывании USB_LP_CAN1_RX0_IRQHandler
	// Прерывание получения пакета в буфер FIFO 0
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	// Прерывание при заполнении буфера FIFO 0
	CAN_ITConfig(CAN1, CAN_IT_FF0, ENABLE);
	// Прерывание при переполнении буфера FIFO 0
	CAN_ITConfig(CAN1, CAN_IT_FOV0, ENABLE);
#endif
#ifdef CAN1_RX1_IRQHandler_ENABLE
	// Обрабатывается в прерывании CAN1_RX1_IRQHandler
	// Прерывание получения пакета в буфер FIFO 1
	CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);
	// Прерывание получения пакета в буфер FIFO 1
	CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);
	// Прерывание при заполнении буфера FIFO 1
	CAN_ITConfig(CAN1, CAN_IT_FF1, ENABLE);
	// Прерывание при переполнении буфера FIFO 1
	CAN_ITConfig(CAN1, CAN_IT_FOV1, ENABLE);
#endif
#ifdef CAN1_SCE_IRQHandler_ENABLE
	// CAN Operating Mode Interrupt enable
	// Обрабатывается в прерывании CAN1_SCE_IRQHandler
	// Прерывание при "пробуждении" - выход из "спящего" режима
	CAN_ITConfig(CAN1, CAN_IT_WKU, ENABLE);
	// Прерывание при переходе в "спящий" режим
	CAN_ITConfig(CAN1, CAN_IT_SLK, ENABLE);

	// CAN Error Interrupts
	// Обрабатывается в прерывании CAN1_SCE_IRQHandler
	// Error warning Interrupt (error counter >= 96)
	CAN_ITConfig(CAN1, CAN_IT_EWG, ENABLE);
	// Error passive Interrupt (error counter > 127)
	CAN_ITConfig(CAN1, CAN_IT_EPV, ENABLE);
	// Bus-off Interrupt (error counter > 255)
	CAN_ITConfig(CAN1, CAN_IT_BOF, ENABLE);
	// Last error code - при возникновении ошибок приема-передачи
	CAN_ITConfig(CAN1, CAN_IT_LEC, ENABLE);
	// Прерывание при возникновении ошибок bxCan
	CAN_ITConfig(CAN1, CAN_IT_ERR, ENABLE);
#endif


	// Структура для инициализации прерываний
	// NVIC Configuration
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef USB_HP_CAN1_TX_IRQHandler_ENABLE
	// Enable CAN1 TX0 interrupt IRQ channel
	// Выбор канала для приёма сообщения
	NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
	// Устанавливает основной приоритет для канала, указанного в NVIC_IRQChannel
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	// Устанавливает приоритет внутри группы для канала, указанного в NVIC_IRQChannel
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	// Разрешает прерывание, указанное в NVIC_IRQChannel
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// Применяет все настройки, указанные выше
	NVIC_Init(&NVIC_InitStructure);
#endif
#ifdef USB_LP_CAN1_RX0_IRQHandler_ENABLE
	// Enable CAN1 RX0 interrupt IRQ channel
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
#ifdef CAN1_RX1_IRQHandler_ENABLE
	// Enable CAN1 RX1 interrupt IRQ channel
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
#ifdef CAN1_SCE_IRQHandler_ENABLE
	// Enable CAN1 SCE (Status Change Error) interrupt IRQ channel
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef USB_HP_CAN1_TX_IRQHandler_ENABLE
	NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
#endif
#ifdef USB_LP_CAN1_RX0_IRQHandler_ENABLE
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
#endif
#ifdef CAN1_RX1_IRQHandler_ENABLE
	NVIC_EnableIRQ(CAN1_RX1_IRQn);
#endif
#ifdef CAN1_SCE_IRQHandler_ENABLE
	NVIC_EnableIRQ(CAN1_SCE_IRQn);
#endif
}

#ifdef USB_HP_CAN1_TX_IRQHandler_ENABLE
void USB_HP_CAN_TX_IRQHandler(void)
{
	// CAN Transmit mailbox empty Interrupt enable
	// Обработаем прерывания при освобождении исходящего почтового ящика
	if (CAN_GetITStatus(CAN1, CAN_IT_TME)==SET)
	{
		// Сбросим флаг прерывания
		CAN_ClearITPendingBit(CAN1, CAN_IT_TME);

		/* Вставляем любой свой код обработки входящего пакета */

		/* --------------------------------------------------- */
	}
}
#endif

#ifdef CAN1_RX1_IRQHandler_ENABLE
/**
 * @brief
 * @note   None
 * @param  None
 * @retval None
 */
void CAN_RX1_IRQHandler(void)
{
	CanRxMsg RxMessage;

	// CAN Receive Interrupt enable FIFO 1
	// Обработаем прерывания приеного буфера FIFO 1

	// Прерывание получения пакета в буфер FIFO 1
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP1) == SET)
	{
		// Сбросим флаг прерывания
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP1);

		// Обнулим данные пакета
		RxMessage = {0};

		// Получим сообщение
		CAN_Receive(CAN1, CAN_FIFO1, &RxMessage);

		/* Вставляем любой свой код обработки входящего пакета */

		/* --------------------------------------------------- */
	}

	// Прерывание при заполнении буфера FIFO 1
	if (CAN_GetITStatus(CAN1, CAN_IT_FF1)==SET)
	{
		// Сбросим флаг прерывания
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF1);

		/* Вставляем любой свой код обработки входящего пакета */

		/* --------------------------------------------------- */

		// Не забываем после обработки сбросить флаг ошибки
		CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
	}

	// Прерывание при переполнении буфера FIFO 1
	if (CAN_GetITStatus(CAN1, CAN_IT_FOV1)==SET)
	{
		// Сбросим флаг прерывания
		CAN_ClearITPendingBit(CAN1, CAN_IT_FOV1);

		/* Вставляем любой свой код обработки входящего пакета */

		/* --------------------------------------------------- */

		// Не забываем после обработки сбросить флаг ошибки
		CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
	}
}
#endif

#ifdef USB_LP_CAN1_RX0_IRQHandler_ENABLE
void USB_LP_CAN_RX0_IRQHandler(void)
{
	// CAN Receive Interrupt enable FIFO 0
	// Обработаем прерывания приемного буфера FIFO 0

	// Прерывание получения пакета в буфер FIFO 0
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET)
	{
		// Сбросим флаг прерывания
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);

		// Обнулим данные пакета
		CanRxMsg RxMessage = {0};

		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

		/* Вставляем любой свой код обработки входящего пакета */

		/* --------------------------------------------------- */
	}

	// Прерывание при заполнении буфера FIFO 0
	if (CAN_GetITStatus(CAN1, CAN_IT_FF0)==SET)
	{
		// Сбросим флаг прерывания
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);

		/* Вставляем любой свой код обработки входящего пакета */

		/* --------------------------------------------------- */

		// Не забываем после обработки сбросить флаг ошибки
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
	}

	// Прерывание при переполнении буфера FIFO 0
	if (CAN_GetITStatus(CAN1, CAN_IT_FOV0)==SET)
	{
		// Сбросим флаг прерывания
		CAN_ClearITPendingBit(CAN1, CAN_IT_FOV0);

		/* Вставляем любой свой код обработки входящего пакета */

		/* --------------------------------------------------- */

		// Не забываем после обработки сбросить флаг ошибки
		CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
	}
}
#endif

#ifdef CAN1_SCE_IRQHandler_ENABLE
void CAN_SCE_IRQHandler(void)
{
	uint8_t errorcode = 0;

	errcnt = CAN_GetReceiveErrorCounter(CAN1);
	errcnt = CAN_GetLSBTransmitErrorCounter(CAN1);

	// Прерывание при возникновении ошибки
	if (CAN_GetITStatus(CAN1, CAN_IT_ERR)==SET)
	{
		// Сбросим флаг прерывания
		CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);

		// CAN Error Interrupts
		// Обработка прерываний по ошибке

		// Error warning Interrupt (счетчик ошибок >= 96)
		if (CAN_GetITStatus(CAN1, CAN_IT_EWG)==SET)
		{
			// Сбросим флаг прерывания
			CAN_ClearITPendingBit(CAN1, CAN_IT_EWG);

			/* Вставляем любой свой код обработки входящего пакета */

			/* --------------------------------------------------- */
		}

		// Error passive Interrupt  (счетчик ошибок > 127)
		if (CAN_GetITStatus(CAN1, CAN_IT_EPV)==SET)
		{
			// Сбросим флаг прерывания
			CAN_ClearITPendingBit(CAN1, CAN_IT_EPV);

			/* Вставляем любой свой код обработки входящего пакета */

			/* --------------------------------------------------- */
		}

		// Bus-off. Прерывание при переполнении счетчика ошибок (>255)
		// bxCan уходит в режим Bus-OFF
		if (CAN_GetITStatus(CAN1, CAN_IT_BOF)==SET)
		{
			// Сбросим флаг прерывания
			CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);

			/* Вставляем любой свой код обработки входящего пакета */

			/* --------------------------------------------------- */
		}

		// Прерывание при ошибке приема передачи сообщения
		if (CAN_GetITStatus(CAN1, CAN_IT_LEC)==SET)
		{
			// Сбросим флаг прерывания
			CAN_ClearITPendingBit(CAN1, CAN_IT_LEC);
			// Получим код ошибки

			errorcode = CAN_GetLastErrorCode(CAN1);

			/* Вставляем любой свой код обработки входящего пакета */

			/* --------------------------------------------------- */

			// Не забываем после обработки сбросить флаг ошибки
			CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
		}

	}
	else
	{
		// CAN Operating Mode Interrupt
		// Обработка прерываний по режимам сна/пробуждения

		// Прерывание при "пробуждении" - выход из "спящего" режима
		if (CAN_GetITStatus(CAN1, CAN_IT_WKU)==SET)
		{
			// Сбросим флаг прерывания
			CAN_ClearITPendingBit(CAN1, CAN_IT_WKU);

			/* Вставляем любой свой код обработки входящего пакета */

			/* --------------------------------------------------- */

			// Не забываем после обработки сбросить флаг ошибки
			CAN_ClearFlag(CAN1, CAN_FLAG_WKU);
		}

		// Прерывание при переходе в "спящий" режим
		if (CAN_GetITStatus(CAN1, CAN_IT_SLK)==SET)
		{
			// Сбросим флаг прерывания
			CAN_ClearITPendingBit(CAN1, CAN_IT_SLK);

			/* Вставляем любой свой код обработки входящего пакета */

			/* --------------------------------------------------- */

			// Не забываем после обработки сбросить флаг ошибки
			CAN_ClearFlag(CAN1, CAN_FLAG_SLAK);
		}
	}
}
#endif

/**
 * @brief
 * @note
 * @param  None
 * @retval None
 */
void CAN_Send_Test(void)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_CMD_Test_Send;

	TxMessage.ExtId = 0x00;

	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;

	TxMessage.Data[0] = 0x00;
	TxMessage.Data[1] = 0x01;
	TxMessage.Data[2] = 0x02;
	TxMessage.Data[3] = 0x02;
	TxMessage.Data[4] = 0x02;
	TxMessage.Data[5] = 0x02;
	TxMessage.Data[6] = 0x02;
	TxMessage.Data[7] = num;

	num++;
	CAN_Transmit(CAN1, &TxMessage);
}


/**
 * @brief
 * @note
 * @param  None
 * @retval None
 */
void CAN_Send_Ok(void)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_CMD_Test_Ok;

	TxMessage.ExtId = 0x00;

	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.Data[0] = 0x05;
	TxMessage.Data[1] = 0x05;
	TxMessage.Data[2] = 0x05;
	TxMessage.Data[3] = 0x05;
	TxMessage.Data[4] = 0x05;
	TxMessage.Data[5] = 0x05;
	TxMessage.Data[6] = 0x05;
	TxMessage.Data[7] = 0x05;

	CAN_Transmit(CAN1, &TxMessage);
}


