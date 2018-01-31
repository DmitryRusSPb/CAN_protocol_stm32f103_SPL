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

#ifndef __NAZA_CAN_DECODER_LIB_H__
#define __NAZA_CAN_DECODER_LIB_H__

#include "stm32f10x.h"
#include "can.h"
#include "math.h"

/* Формат приходящих сообщений:
 * 55 AA 55 AA ID ID ML ML <полезная нагрузка> 66 CC 66 CC
 * , где
 * 55 AA 55 AA - заголовок сообщения
 * ID - 2-байтовый идентификатор сообщения
 * ML - длина байта полезной нагрузки сообщения длиной 2 байта (малое число)
 * 66 CC 66 CC - нижний колонтитул сообщения
 *
 * Перечень идентификаторов:
 *
 * ID = 03 10, ML = 3A 00 с CAN id 0x7F8 - это данные GPS
 * Формат полезной нагрузки такой же, как и в сообщении модуля
 * GPS 0x10, за исключением того, что не требуется XORing.
 *
 * ID = 04 10, ML = 06 00 с идентификатором CAN 0x118 - который является данными компаса
 * Формат полезной нагрузки такой же, как и в сообщении модуля GPS 0x20,
 * за исключением того, что не требуется XORing.
 *
 * ID = 21 09, ML = 0C 00 с CAN id 0x7F8 (отправляется каждые ~ 250 мс) - это данные
 * версии модуля GPS
 * Формат полезной нагрузки такой же, как в сообщении модуля GPS 0x30.
 *
 * ID = 22 09, ML = 00 00 с идентификатором CAN 0x7F8 (отправляется каждую секунду)
 * - у него нет полезной нагрузки, кажется, это пульс PMU
 *
 * ID = 07 10, ML = 00 00 с CAN-идентификатором 0x108 (отправляется каждые 2 с) - у него нет
 * полезной нагрузки, кажется, это пульс модуля OSD (заставляет FC отправлять следующие 3 сообщения)
 *
 * ID = 02 10, ML = 78 00 с CAN-идентификатором 0x090 (отправляется каждые 10 мс)
 * - кажется, это сообщение данных OSD
 *
 * ID = 09 10, ML = B8 00 с CAN-идентификатором 0x108 (отправляется каждые 20 мс)
 * - сообщение ввода / вывода
 *
 * ID = 01 10, ML = 0E 00 с идентификатором CAN 0x090 - данные необработанного
 * гироскопа
 *
 * ID = 00 10, ML = 34 00 с идентификатором CAN 0x108 - TBD
 *
 * ID = 26 09, ML = 23 00 с CAN-идентификатором 0x7F8 (отправляется каждые 200 мс)
 * - Фантомные интеллектуальные данные батареи
 *
 * ID = D1 C0, ML = 04 00 с CAN id 0x1FE (отправляется каждую секунду)
 * - выглядит как версия прошивки смарт-смартфона Phantom
 */

// Uncommnet to read smart battery data (if available) e.g. on Phantom controller
//#define GET_SMART_BATTERY_DATA

#define CAN_SPEED				1000000

#define NAZA_MESSAGE_COUNT   	3
#define NAZA_MESSAGE_NONE    	0x0000
#define NAZA_MESSAGE_MSG1002 	0x1002
#define NAZA_MESSAGE_MSG1003 	0x1003
#define NAZA_MESSAGE_MSG1009 	0x1009
#ifdef 	GET_SMART_BATTERY_DATA
#define NAZA_MESSAGE_MSG0926 	0x0926
#endif

// IO ports (for AllInOne Board v0.2)
#define IO_1 A9
#define IO_2 A8
#define IO_3 A7
#define IO_4 A6

typedef enum
{
	NO_FIX = 0,
	FIX_2D = 2,
	FIX_3D = 3,
	FIX_DGPS = 4
} fixType_t;  // GPS fix type

typedef enum
{
	MANUAL = 0,
	GPS = 1,
	FAILSAFE = 2,
	ATTI = 3
} modes_t;  // Flying mode

typedef enum
{
	MOTOR_M1 = 0,
	MOTOR_M2 = 1,
	MOTOR_M3 = 2,
	MOTOR_M4 = 3,
	MOTOR_M5 = 4,
	MOTOR_M6 = 5,
	MOTOR_F1 = 6,
	MOTOR_F2 = 7
} motorOut_t;  // Motor output index

typedef enum
{
	RC_UNUSED_1 = 0,
	RC_A = 1,
	RC_E = 2,
	RC_R = 3,
	RC_U = 4,
	RC_T = 5,
	RC_UNUSED_2 = 6,
	RC_X1 = 7,
	RC_X2 = 8,
	RC_UNUSED_3 = 9
} rcInChan_t;  // RC channel index

typedef struct __attribute__((packed))
{
	uint16_t id;
	uint16_t len;
} naza_msg_header_t;

typedef struct __attribute__((packed))
{
	naza_msg_header_t header;
	double  lon;           // longitude (radians)
	double  lat;           // lattitude (radians)
	float   altGps;        // altitude from GPS (meters)
	float   accX;          // accelerometer X axis data (??)
	float   accY;          // accelerometer Y axis data (??)
	float   accZ;          // accelerometer Z axis data (??)
	float   gyrX;          // gyroscope X axis data (??)
	float   gyrY;          // gyroscope Y axis data (??)
	float   gyrZ;          // gyroscope Z axis data (??)
	float   altBaro;       // altitude from barometric sensor (meters)
	float   quaternion[4]; // attitude quaternions
	float   unk1[3];
	float   northVelocity; // averaged northward velocity or 0 when less than 5 satellites locked (m/s)
	float   eastVelocity;  // averaged eastward velocity or 0 when less than 5 satellites locked (m/s)
	float   downVelocity;  // downward velocity (barometric) (m/s)
	float   unk2[3];
	uint16_t  magCalX;     // calibrated magnetometer X axis data
	uint16_t  magCalY;     // calibrated magnetometer Y axis data
	uint16_t  magCalZ;     // calibrated magnetometer Z axis data
	uint8_t  unk3[10];
	uint8_t  numSat;       // number of locked satellites
	uint8_t  unk4;
	uint16_t seqNum;       // sequence number - increases with every message
} naza_msg1002_t;

typedef struct __attribute__((packed))
{
	naza_msg_header_t header;
	uint32_t dateTime[6];      // date/time
	uint32_t lon;           // longitude (x10^7, degree decimal)
	uint32_t lat;           // lattitude (x10^7, degree decimal)
	uint32_t altGps;        // altitude from GPS (millimeters)
	uint32_t hae;           // horizontal accuracy estimate (millimeters)
	uint32_t vae;           // vertical accuracy estimate (millimeters)
	uint8_t  unk0[4];
	uint32_t  northVelocity;// northward velocity (cm/s)
	uint32_t  eastVelocity; // eastward velocity (cm/s)
	uint32_t  downVelocity; // downward velocity (cm/s)
	uint16_t pdop;          // position DOP (x100)
	uint16_t vdop;          // vertical DOP (see uBlox NAV-DOP message for details)
	uint16_t ndop;          // northing DOP (see uBlox NAV-DOP message for details)
	uint16_t edop;          // easting DOP (see uBlox NAV-DOP message for details)
	uint8_t  numSat;        // number of locked satellites
	uint8_t  unk1;
	uint8_t  fixType;       // fix type (0 - no lock, 2 - 2D lock, 3 - 3D lock, not sure if other values can be expected - see uBlox NAV-SOL message for details)
	uint8_t  unk2;
	uint8_t  fixStatus;     // fix status flags (see uBlox NAV-SOL message for details)
	uint8_t  unk3[3];
	uint16_t seqNum;        // sequence number - increases with every message
} naza_msg1003_t;

typedef struct __attribute__((packed))
{
	naza_msg_header_t header;
	uint8_t  unk1[4];
	uint16_t motorOut[8];  // motor output (M1/M2/M3/M4/M5/M6/F1/F2)
	uint8_t  unk2[4];
	int16_t  rcIn[10];     // RC controller input (order: unused/A/E/R/U/T/unused/X1/X2/unused)
	uint8_t  unk3[11];
	uint8_t  flightMode;   // (0 - manual, 1 - GPS, 2 - failsafe, 3 - atti)
	uint8_t  unk4[8];
	double   homeLat;      // home lattitude (radians)
	double   homeLon;      // home longitude (radians)
	float    homeAltBaro;  // home altitude from barometric sensor plus 20m (meters)
	uint16_t seqNum;       // sequence number - increases with every message
	uint8_t  unk5[2];
	float    stabRollIn;   // attitude stabilizer roll input (-1000~1000)
	float    stabPitchIn;  // attitude stabilizer pitch input (-1000~1000)
	float    stabThroIn;   // altitude stabilizer throttle input (-1000~1000)
	uint8_t  unk6[4];
	float    actAileIn;    // actual aileron input, mode and arm state dependent (-1000~1000)
	float    actElevIn;    // actual elevator input, mode and arm state dependent (-1000~1000)
	float    actThroIn;    // actual throttle input, mode and arm state dependent (-1000~1000)
	uint16_t batVolt;      // main battery voltage (milivolts)
	uint16_t becVolt;      // BEC voltage (milivolts)
	uint8_t  unk7[4];
	uint8_t  controlMode;  // (0 - GPS/failsafe, 1 - waypoint mode?, 3 - manual, 6 - atti)
	uint8_t  unk8[5];
	uint16_t  gyrScalX;    // ???
	uint16_t  gyrScalY;    // ???
	uint16_t  gyrScalZ;    // ???
	uint8_t  unk9[32];
	float    downVelocity; // downward velocity (m/s)
	float    altBaro;      // altitude from barometric sensor (meters)
	float    roll;         // roll angle (radians)
	float    pitch;        // pitch angle (radians)
} naza_msg1009_t;

#ifdef GET_SMART_BATTERY_DATA
typedef struct __attribute__((packed))
{
	naza_msg_header_t header;
	uint16_t designCapacity;  // design capacity in mAh
	uint16_t fullCapacity;    // design capacity in mAh
	uint16_t currentCapacity; // design capacity in mAh
	uint16_t voltage;         // battry voltage in mV
	int16_t  current;         // current in mA
	uint8_t  lifePercent;     // percentage of life
	uint8_t  chargePercent;   // percentage of charge
	int16_t temperature;      // temperature in tenths of a degree Celsius
	uint16_t dischargeCount;  // number of discharges
	uint16_t serialNumber;    // serial number
	uint16_t cellVoltage[3];  // individual cell voltage in mV
	uint8_t  unk1[11];
} naza_msg0926_t;
#endif

typedef union
{
	uint8_t           bytes[256]; // Max message size (184) + header size (4) + footer size (4)
	naza_msg_header_t header;
	naza_msg1002_t    msg1002;
	naza_msg1003_t    msg1003;
	naza_msg1009_t    msg1009;
#ifdef GET_SMART_BATTERY_DATA
	naza_msg0926_t    msg0926;
#endif
} naza_msg_t;


uint32_t heartbeatTime;
uint8_t canMsgIdIdx;
uint8_t canMsgByte;

naza_msg_t msgBuf[NAZA_MESSAGE_COUNT];
uint16_t msgLen[NAZA_MESSAGE_COUNT];
uint16_t msgIdx[NAZA_MESSAGE_COUNT];
uint8_t header[NAZA_MESSAGE_COUNT];
uint8_t footer[NAZA_MESSAGE_COUNT];
uint8_t collectData[NAZA_MESSAGE_COUNT];

double lon;       // longitude in degree decimal
double lat;       // latitude in degree decimal
double alt;       // altitude in m (from barometric sensor)
double gpsAlt;    // altitude in m (from GPS)
double spd;       // speed in m/s
fixType_t fix;     // fix type (see fixType_t enum)
uint8_t sat;       // number of satellites
double heading;   // heading in degrees (titlt compensated)
double headingNc; // heading in degrees (not titlt compensated)
double cog;       // course over ground
double vsi;       // vertical speed indicator (barometric) in m/s (a.k.a. climb speed)
double hdop;      // horizontal dilution of precision
double vdop;      // vertical dilution of precision
double gpsVsi;    // vertical speed indicator (GPS based) in m/s (a.k.a. climb speed)
float pitchRad;   // pitch in radians
float rollRad;    // roll in radians
uint8_t pitch;      // pitch in degrees
uint16_t roll;      // roll in degrees
uint8_t year;      // year (minus 2000)
uint8_t month;
uint8_t day;
uint8_t hour;    // hour (for time between 16:00 and 23:59 the hour returned from GPS module is actually 00:00 - 7:59)
uint8_t minute;
uint8_t second;
uint16_t battery; // battery voltage in mV
uint16_t motorOut[8]; // motor output value (0 when unused, otherwise 16920~35000, 16920 = motor off), use motorOut_t enum to index the table
uint16_t rcIn[10]; // RC stick input (-1000~1000), use rcInChan_t enum to index the table
modes_t mode;      // flight mode (see mode_t enum)

#ifdef GET_SMART_BATTERY_DATA
uint8_t  batteryPercent; // smart battery charge percentage (0-100%)
uint16_t batteryCell[3]; // smart battery cell voltage in mV, use smartBatteryCell_t enum to index the table
#endif

// Periodically (every 2 sec., keeps it inner counter) sends a heartbeat message to the controller
void Heartbeat(void);
// Start the NazaCanDecoder
void begin(void);
// Decode incoming CAN message if any (shall be called in a loop)
uint16_t NazaCanDecoderLib_Decode(void);
// Returns latitude in degree decimal
double getLat(double lat);
// Returns longitude in degree decimal
double getLon(double lon);
// Returns altitude in m (from barometric sensor)
double getAlt(double alt);
// Returns altitude in m (from GPS)
double getGpsAlt(double gpsAlt);
// Returns speed in m/s
double getSpeed(double gpsAlt);
// Returns fix type (see fixType_t enum)
fixType_t getFixType(void);
// Returns number of satellites
uint8_t getNumSat(uint8_t sat);
// Returns heading in degrees (titlt compensated)
double getHeading(double heading);
// Returns heading in degrees (not titlt compensated)
double getHeadingNc(double headingNc);
// Returns course over ground
double getCog(double cog);
// Returns vertical speed (barometric) in m/s (a.k.a. climb speed)
double getVsi(double vsi);
// Returns vertical speed (from GPS) in m/s (a.k.a. climb speed)
double getGpsVsi(double gpsVsi);
// Returns horizontal dilution of precision
double getHdop(double hdop);
// Returns vertical dilution of precision
double getVdop(double vdop);
// Returns pitch in degrees
uint8_t getPitch(uint8_t pitch);
// Returns roll in degrees
uint16_t getRoll(uint16_t roll);
// Returns year from GPS (minus 2000)
uint8_t getYear(uint8_t year);
// Returns month from GPS
uint8_t getMonth(uint8_t month);
// Returns day from GPS
uint8_t getDay(uint8_t day);
// Returns hour from GPS (Note that for time between 16:00
// and 23:59 the hour returned from GPS module is actually 00:00 - 7:59)
uint8_t getHour(uint8_t hour);
// Returns minute from GPS
uint8_t getMinute(uint8_t minute);
// Returns second from GPS
uint8_t getSecond(uint8_t second);
// Returns battery voltage in mV
uint16_t getBattery(uint16_t battery);
// Returns motor output value (0 when unused, otherwise 16920~35000, 16920 = motor off),
// use motorOut_t enum to index the table
uint16_t getMotorOut(motorOut_t mot);
// Returns RC stick input (-1000~1000), use rcInChan_t enum to index the table
uint16_t getRcIn(rcInChan_t chan);
// Returns flight mode (see mode_t enum)
//mode_t getMode(mode_t mode);

#ifdef GET_SMART_BATTERY_DATA
typedef enum
{
	CELL_1 = 0,
	CELL_2 = 1,
	CELL_3 = 2,
} smartBatteryCell_t;  // Smart battery cell index
uint8_t  getBatteryPercent(uint8_t batteryPercent); // battery charge percentage (0-100%)
uint16_t getBatteryCell(smartBatteryCell_t cell); // battery cell voltage in mV, use smartBatteryCell_t enum to index the table
#endif

typedef enum
{
	Status_ERROR,
	Status_OK
}StatusTypeDef;

StatusTypeDef CAN_setFilter(uint16_t filter_id, uint8_t filter_num);
StatusTypeDef CAN_write(const uint16_t (*array)[2]);
void NazaCanDecoderLib_Begin(void);


#endif // __NAZA_CAN_DECODER_LIB_H__
