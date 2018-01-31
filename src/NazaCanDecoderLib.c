/*
 * NazaCanDecoderLib.c
 *
 *  Created on: 26 янв. 2018 г.
 *      Author: root
 */
/*
  DJI Naza (v1/V2 + PMU, Phantom) CAN data decoder library
  (c) Pawelsky 20171104
  Not for commercial use

  Requires FlexCan library v1.0 (or newer if compatible)
  https://github.com/teachop/FlexCAN_Library/releases/tag/v1.0

  Requires Teensy 3.1 board and CAN transceiver
  Complie with "CPU speed" set to "96MHz (overclock)"
  Refer to naza_can_decoder_wiring.jpg diagram for proper connection
  Connections can be greatly simplified using CAN bus and MicroSD or AllInOne shields by Pawelsky
  (see teensy_shields.jpg or teensy_aio_shield.jpg for installation and naza_can_decoder_wiring_shields.jpg or naza_can_decoder_wiring_aio_shield.jpg for wiring)
 */

#include "NazaCanDecoderLib.h"


const uint16_t HEARTBEAT_1 [2][8] = {{0x108, 0, 8, 0, 0, 0, 0, 0}, {0x55, 0xAA, 0x55, 0xAA, 0x07, 0x10, 0x00, 0x00}};
const uint16_t HEARTBEAT_2 [2][8] = {{0x108, 0, 4, 0, 0, 0, 0, 0}, {0x66, 0xCC, 0x66, 0xCC, 0x00, 0x00, 0x00, 0x00}};
const uint16_t FILTER_MASK = 0x7FF;
const uint16_t FILTER_090  = 0x090;
const uint16_t FILTER_108  = 0x108;
const uint16_t FILTER_7F8  = 0x7F8;

//NazaCanDecoderLib::NazaCanDecoderLib()
//{
//	// Empty
//}

void NazaCanDecoderLib_Begin(void)
{
	//CAN_begin(FILTER_MASK);
	for(uint8_t i = 0; i < 3; i++)
		CAN_setFilter(FILTER_090, i);

	for(uint8_t i = 3; i < 6; i++)
		CAN_setFilter(FILTER_108, i);

	for(uint8_t i = 6; i < 8; i++)
		CAN_setFilter(FILTER_7F8, i);
}

uint16_t NazaCanDecoderLib_Decode(void)
{
	CanRxMsg RxMessage;

	// Обнулим данные пакета
	RxMessage.DLC =     0x00;
	RxMessage.ExtId =   0x00;
	RxMessage.FMI =     0x00;
	RxMessage.IDE =     0x00;
	RxMessage.RTR =     0x00;
	RxMessage.StdId =   0x00;
	RxMessage.Data [0] = 0x00;
	RxMessage.Data [1] = 0x00;
	RxMessage.Data [2] = 0x00;
	RxMessage.Data [3] = 0x00;
	RxMessage.Data [4] = 0x00;
	RxMessage.Data [5] = 0x00;
	RxMessage.Data [6] = 0x00;
	RxMessage.Data [7] = 0x00;

	uint16_t msgId = NAZA_MESSAGE_NONE;
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	switch(RxMessage.StdId)
	{
	case 0x090:
		canMsgIdIdx = 0;
		break;
	case 0x108:
		canMsgIdIdx = 1;
		break;
	case 0x7F8:
		canMsgIdIdx = 2;
		break;
	default:
		return msgId; // we don't care about other CAN messages
	}

	for(uint8_t i = 0; i < RxMessage.DLC; i++)
	{
		canMsgByte = RxMessage.Data[i];
		if(collectData[canMsgIdIdx] == 1)
		{
			msgBuf[canMsgIdIdx].bytes[msgIdx[canMsgIdIdx]] = canMsgByte;
			if(msgIdx[canMsgIdIdx] == 3)
			{
				msgLen[canMsgIdIdx] = msgBuf[canMsgIdIdx].header.len;
			}
			msgIdx[canMsgIdIdx] += 1;
			if((msgIdx[canMsgIdIdx] > (msgLen[canMsgIdIdx] + 8)) || (msgIdx[canMsgIdIdx] > 256))
				collectData[canMsgIdIdx] = 0;
		}

		// Look fo header
		if(canMsgByte == 0x55)
		{
			if(header[canMsgIdIdx] == 0)
				header[canMsgIdIdx] = 1;
			else
				if(header[canMsgIdIdx] == 2)
					header[canMsgIdIdx] = 3;
				else header[canMsgIdIdx] = 0;
		}
		else
			if(canMsgByte == 0xAA)
			{
				if(header[canMsgIdIdx] == 1)
					header[canMsgIdIdx] = 2;
				else
					if(header[canMsgIdIdx] == 3)
					{
						header[canMsgIdIdx] = 0;
						collectData[canMsgIdIdx] = 1;
						msgIdx[canMsgIdIdx] = 0;
					}
					else
						header[canMsgIdIdx] = 0;
			}
			else
				header[canMsgIdIdx] = 0;

		// Look fo footer
		if(canMsgByte == 0x66)
		{
			if(footer[canMsgIdIdx] == 0)
				footer[canMsgIdIdx] = 1;
			else
				if(footer[canMsgIdIdx] == 2)
					footer[canMsgIdIdx] = 3;
				else
					footer[canMsgIdIdx] = 0;
		}
		else
			if(canMsgByte == 0xCC)
			{
				if(footer[canMsgIdIdx] == 1)
					footer[canMsgIdIdx] = 2;
				else
					if(footer[canMsgIdIdx] == 3)
					{
						footer[canMsgIdIdx] = 0;
						if(collectData[canMsgIdIdx] != 0)
							collectData[canMsgIdIdx] = 2;
					}
					else
						footer[canMsgIdIdx] = 0;
			}
			else
				footer[canMsgIdIdx] = 0;

		if(collectData[canMsgIdIdx] == 2)
		{
			if(msgIdx[canMsgIdIdx] == (msgLen[canMsgIdIdx] + 8))
			{
				if(msgBuf[canMsgIdIdx].header.id == NAZA_MESSAGE_MSG1002)
				{
					float magCalX = msgBuf[canMsgIdIdx].msg1002.magCalX;
					float magCalY = msgBuf[canMsgIdIdx].msg1002.magCalY;
					headingNc = -atan2(magCalY, magCalX) / M_PI * 180.0;
					if(headingNc < 0) headingNc += 360.0;
					float q0 = msgBuf[canMsgIdIdx].msg1002.quaternion[0];
					float q1 = msgBuf[canMsgIdIdx].msg1002.quaternion[1];
					float q2 = msgBuf[canMsgIdIdx].msg1002.quaternion[2];
					float q3 = msgBuf[canMsgIdIdx].msg1002.quaternion[3];
					heading = atan2(2.0 * (q3 * q0 + q1 * q2) , -1.0 + 2.0 * (q0 * q0 + q1 * q1)) / M_PI * 180.0;
					if(heading < 0) heading += 360.0;
					sat = msgBuf[canMsgIdIdx].msg1002.numSat;
					gpsAlt = msgBuf[canMsgIdIdx].msg1002.altGps;
					lat = msgBuf[canMsgIdIdx].msg1002.lat / M_PI * 180.0;
					lon = msgBuf[canMsgIdIdx].msg1002.lon / M_PI * 180.0;
					alt = msgBuf[canMsgIdIdx].msg1002.altBaro;
					float nVel = msgBuf[canMsgIdIdx].msg1002.northVelocity;
					float eVel = msgBuf[canMsgIdIdx].msg1002.eastVelocity;
					spd = sqrt(nVel * nVel + eVel * eVel);
					cog = atan2(eVel, nVel) / M_PI * 180;
					if(cog < 0) cog += 360.0;
					vsi = -msgBuf[canMsgIdIdx].msg1002.downVelocity;
					msgId = NAZA_MESSAGE_MSG1002;
				}
				else
					if(msgBuf[canMsgIdIdx].header.id == NAZA_MESSAGE_MSG1003)
					{
						uint32_t dateTime = msgBuf[canMsgIdIdx].msg1003.dateTime;
						second = dateTime & 0b00111111; dateTime >>= 6;
						minute = dateTime & 0b00111111; dateTime >>= 6;
						hour = dateTime & 0b00001111; dateTime >>= 4;
						day = dateTime & 0b00011111; dateTime >>= 5;
						if(hour > 7)
							day++;
						month = dateTime & 0b00001111; dateTime >>= 4;
						year = dateTime & 0b01111111;
						gpsVsi = -msgBuf[canMsgIdIdx].msg1003.downVelocity;
						vdop = (double)msgBuf[canMsgIdIdx].msg1003.vdop / 100;
						double ndop = (double)msgBuf[canMsgIdIdx].msg1003.ndop / 100;
						double edop = (double)msgBuf[canMsgIdIdx].msg1003.edop / 100;
						hdop = sqrt(ndop * ndop + edop * edop);
						uint8_t fixType = msgBuf[canMsgIdIdx].msg1003.fixType;
						uint8_t fixFlags = msgBuf[canMsgIdIdx].msg1003.fixStatus;
						switch(fixType)
						{
						case 2:
							fix = FIX_2D; break;
						case 3:
							fix = FIX_3D; break;
						default:
							fix = NO_FIX; break;
						}
						if((fix != NO_FIX) && (fixFlags & 0x02))
							fix = FIX_DGPS;
						msgId = NAZA_MESSAGE_MSG1003;
					}
					else
						if(msgBuf[canMsgIdIdx].header.id == NAZA_MESSAGE_MSG1009)
						{
							for(uint8_t j = 0; j < 8; j++)
							{
								motorOut[j] = msgBuf[canMsgIdIdx].msg1009.motorOut[j];
							}
							for(uint8_t j = 0; j < 10; j++)
							{
								rcIn[j] = msgBuf[canMsgIdIdx].msg1009.rcIn[j];
							}
#ifndef GET_SMART_BATTERY_DATA
							battery = msgBuf[canMsgIdIdx].msg1009.batVolt;
#endif
							rollRad = msgBuf[canMsgIdIdx].msg1009.roll;
							pitchRad = msgBuf[canMsgIdIdx].msg1009.pitch;
							roll = (uint16_t)(rollRad * 180.0 / M_PI);
							pitch = (uint8_t)(pitchRad * 180.0 / M_PI);
							mode = msgBuf[canMsgIdIdx].msg1009.flightMode;
							msgId = NAZA_MESSAGE_MSG1009;
						}
#ifdef GET_SMART_BATTERY_DATA
						else
							if(msgBuf[canMsgIdIdx].header.id == NAZA_MESSAGE_MSG0926)
							{
								battery = msgBuf[canMsgIdIdx].msg0926.voltage;
								batteryPercent = msgBuf[canMsgIdIdx].msg0926.chargePercent;
								for(uint8_t j = 0; j < 3; j++)
								{
									batteryCell[j] = msgBuf[canMsgIdIdx].msg0926.cellVoltage[j];
								}
								msgId = NAZA_MESSAGE_MSG0926;
							}
#endif
			}
			collectData[canMsgIdIdx] = 0;
		}
	}
	return msgId;
}

void Heartbeat()
{
	CanTxMsg TxMessage;

	TxMessage.StdId = HEARTBEAT_1[0][0];
	TxMessage.ExtId = 0x00;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = HEARTBEAT_1[0][7];

	TxMessage.Data[0] = HEARTBEAT_1[1][0];
	TxMessage.Data[1] = HEARTBEAT_1[1][1];
	TxMessage.Data[2] = HEARTBEAT_1[1][2];
	TxMessage.Data[3] = HEARTBEAT_1[1][3];

	CAN_Transmit(CAN1, &TxMessage);

	TxMessage.StdId = HEARTBEAT_2[0][0];
	TxMessage.ExtId = 0x00;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = HEARTBEAT_2[0][7];

	TxMessage.Data[0] = HEARTBEAT_2[1][0];
	TxMessage.Data[1] = HEARTBEAT_2[1][1];
	TxMessage.Data[2] = HEARTBEAT_2[1][2];
	TxMessage.Data[3] = HEARTBEAT_2[1][3];
	TxMessage.Data[4] = HEARTBEAT_2[1][4];
	TxMessage.Data[5] = HEARTBEAT_2[1][5];
	TxMessage.Data[6] = HEARTBEAT_2[1][6];
	TxMessage.Data[7] = HEARTBEAT_2[1][7];

	CAN_Transmit(CAN1, &TxMessage);
}

StatusTypeDef CAN_setFilter(uint16_t filter_id, uint8_t filter_num)
{
	// CAN filter init
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	// Номер фильтра, доступны с 0 по 13
	CAN_FilterInitStructure.CAN_FilterNumber = filter_num;
	// Режим работы фильтра
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	// Разрядность (масштабирование)
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	// Старшая часть ID
	CAN_FilterInitStructure.CAN_FilterIdLow = filter_id << 5;
	// Младшая часть ID
	CAN_FilterInitStructure.CAN_FilterIdHigh =  0x0000;
	// Старшая часть маски ID
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	// Младшая часть маски ID
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	// Имя буфера FIFO (у нас их всего два)
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	// Состояние фильтра
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	return Status_OK;
}



