/******************************************************************************
LSM9DS1_Types.h
SFE_LSM9DS1 Library - LSM9DS1 Types and Enumerations
Jim Lindblom @ SparkFun Electronics
Original Creation Date: April 21, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file defines all types and enumerations used by the LSM9DS1 class.

Development environment specifics:
	IDE: Arduino 1.6.0
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __LSM9DS1_Types_H__
#define __LSM9DS1_Types_H__

#include "LSM9DS1_Registers.h"
#include <Wire.h>

// The LSM9DS1 functions over both I2C or SPI. This library supports both.
// But the interface mode used must be sent to the LSM9DS1 constructor. Use
// one of these two as the first parameter of the constructor.
enum interface_mode
{
	IMU_MODE_SPI,
	IMU_MODE_I2C,
};

// accel_scale defines all possible FSR's of the accelerometer:
enum accel_scale
{
	A_SCALE_2G  = 0b00, // 00:  2g
	A_SCALE_16G = 0b01, // 01:  16g
	A_SCALE_4G  = 0b10, // 10:  4g
	A_SCALE_8G  = 0b11  // 11:  8g
};

// gyro_scale defines the possible full-scale ranges of the gyroscope:
enum gyro_scale
{
	G_SCALE_245DPS  = 0b00, // 00:  245 degrees per second
	G_SCALE_500DPS  = 0b01, // 01:  500 dps
	G_SCALE_2000DPS = 0b11  // 11:  2000 dps
};

// mag_scale defines all possible FSR's of the magnetometer:
enum mag_scale
{
	M_SCALE_4GS  = 0b00, // 00:  4Gs
	M_SCALE_8GS  = 0b01, // 01:  8Gs
	M_SCALE_12GS = 0b10, // 10:  12Gs
	M_SCALE_16GS = 0b11, // 11:  16Gs
};

// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
enum gyro_odr
{
	//! TODO
	G_ODR_PD    = 0, // Power down (0)
	G_ODR_149HZ = 1, // 14.9 Hz (1)
	G_ODR_595HZ = 2, // 59.5 Hz (2)
	G_ODR_119HZ = 3, // 119 Hz (3)
	G_ODR_238HZ = 4, // 238 Hz (4)
	G_ODR_476HZ = 5, // 476 Hz (5)
	G_ODR_952HZ = 6  // 952 Hz (6)
};
// accel_oder defines all possible output data rates of the accelerometer:
enum accel_odr
{
	A_ODR_PD    = 0, // Power-down mode (0)
	A_ODR_10HZ  = 1, // 10 Hz (1)
	A_ODR_50HZ  = 2, // 50 Hz (2)
	A_ODR_119HZ = 3, // 119 Hz (3)
	A_ODR_238HZ = 4, // 238 Hz (4)
	A_ODR_476HZ = 5, // 476 Hz (5)
	A_ODR_952HZ = 6	 // 952 Hz (6)
};

// accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
enum accel_abw
{
	A_ABW_408HZ = 0, // 408 Hz (0)
	A_ABW_211HZ = 1, // 211 Hz (1)
	A_ABW_105HZ = 2, // 105 Hz (2)
	A_ABW_50HZ  = 3, //  50 Hz (3)
};

// mag_odr defines all possible output data rates of the magnetometer:
enum mag_odr
{
	M_ODR_0625HZ = 0, // 0.625 Hz (0)
	M_ODR_125HZ  = 1, // 1.25 Hz (1)
	M_ODR_250HZ  = 2, // 2.5 Hz (2)
	M_ODR_5HZ    = 3, // 5 Hz (3)
	M_ODR_10HZ   = 4, // 10 Hz (4)
	M_ODR_20HZ   = 5, // 20 Hz (5)
	M_ODR_40HZ   = 6, // 40 Hz (6)
	M_ODR_80HZ   = 7  // 80 Hz (7)
};

enum interrupt_select
{
	INTERRUPT_SELECT_INT1 = INT1_CTRL,
	INTERRUPT_SELECT_INT2 = INT2_CTRL
};

enum interrupt_generators
{
	INT_DRDY_XL = (1 << 0),	// Accelerometer data ready (INT1 & INT2)
	INT_DRDY_G = (1 << 1),	 // Gyroscope data ready (INT1 & INT2)
	INT1_BOOT = (1 << 2),	  // Boot status (INT1)
	INT2_DRDY_TEMP = (1 << 2), // Temp data ready (INT2)
	INT_FTH = (1 << 3),		   // FIFO threshold interrupt (INT1 & INT2)
	INT_OVR = (1 << 4),		   // Overrun interrupt (INT1 & INT2)
	INT_FSS5 = (1 << 5),	   // FSS5 interrupt (INT1 & INT2)
	INT_IG_XL = (1 << 6),	  // Accel interrupt generator (INT1)
	INT1_IG_G = (1 << 7),	  // Gyro interrupt enable (INT1)
	INT2_INACT = (1 << 7),	 // Inactivity interrupt output (INT2)
};

enum accel_interrupt_generator
{
	XLIE_XL = (1 << 0),
	XHIE_XL = (1 << 1),
	YLIE_XL = (1 << 2),
	YHIE_XL = (1 << 3),
	ZLIE_XL = (1 << 4),
	ZHIE_XL = (1 << 5),
	GEN_6D = (1 << 6)
};

enum gyro_interrupt_generator
{
	XLIE_G = (1 << 0),
	XHIE_G = (1 << 1),
	YLIE_G = (1 << 2),
	YHIE_G = (1 << 3),
	ZLIE_G = (1 << 4),
	ZHIE_G = (1 << 5)
};

enum mag_interrupt_generator
{
	ZIEN = (1 << 5),
	YIEN = (1 << 6),
	XIEN = (1 << 7)
};

enum h_lactive
{
	INT_ACTIVE_HIGH,
	INT_ACTIVE_LOW
};

enum pp_od
{
	INT_PUSH_PULL,
	INT_OPEN_DRAIN
};

enum fifoMode_type
{
	FIFO_OFF = 0,
	FIFO_THS = 1,
	FIFO_CONT_TRIGGER = 3,
	FIFO_OFF_TRIGGER = 4,
	FIFO_CONT = 6
};

struct gyroSettings
{
	// Gyroscope settings:
	bool enabled;
	gyro_scale scale;
	gyro_odr sampleRate;
	// New gyro stuff:
	uint8_t bandwidth;
	bool lowPowerEnable;
	bool HPFEnable;
	uint8_t HPFCutoff;
	uint8_t HPFReference;
	bool flipX;
	bool flipY;
	bool flipZ;
	bool enableX;
	bool enableY;
	bool enableZ;
	bool latchInterrupt;
};

struct deviceSettings
{
	uint8_t commInterface; // Can be I2C, SPI 4-wire or SPI 3-wire
	uint8_t agAddress;	 // I2C address or SPI CS pin
	uint8_t mAddress;	  // I2C address or SPI CS pin
  TwoWire* i2c;    // pointer to an instance of I2C interface
};

struct accelSettings
{
	// Accelerometer settings:
	bool enabled;
	accel_scale scale;
	accel_odr sampleRate;
	// New accel stuff:
	bool enableX;
	bool enableY;
	bool enableZ;
	int8_t bandwidth;
	bool highResEnable;
	uint8_t highResBandwidth;
};

struct magSettings
{
	// Magnetometer settings:
	bool enabled;
	mag_scale scale;
	mag_odr sampleRate;
	// New mag stuff:
	bool tempCompensationEnable;
	uint8_t XYPerformance;
	uint8_t ZPerformance;
	bool lowPowerEnable;
	uint8_t operatingMode;
};

struct temperatureSettings
{
	// Temperature settings
	bool enabled;
};

struct IMUSettings
{
	deviceSettings device;

	gyroSettings gyro;
	accelSettings accel;
	magSettings mag;

	temperatureSettings temp;
};

#endif
