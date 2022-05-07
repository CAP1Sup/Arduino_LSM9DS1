/******************************************************************************
SFE_LSM9DS1.cpp
SFE_LSM9DS1 Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 27, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file implements all functions of the LSM9DS1 class. Functions here range
from higher level stuff, like reading/writing LSM9DS1 registers to low-level,
hardware reads and writes. Both SPI and I2C handler functions can be found
towards the bottom of this file.

Development environment specifics:
	IDE: Arduino 1.6
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"
#include <Wire.h> // Wire library is used for I2C
#include <SPI.h>  // SPI library is used for...SPI.

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

// Sensor Sensitivity Constants
// Values set according to the typical specifications provided in
// table 3 of the LSM9DS1 datasheet. (pg 12)
#define SENSITIVITY_ACCELEROMETER_2G  0.0000610352F
#define SENSITIVITY_ACCELEROMETER_4G  0.0001220703F
#define SENSITIVITY_ACCELEROMETER_8G  0.0002441406F
#define SENSITIVITY_ACCELEROMETER_16G 0.0004882813F
#define SENSITIVITY_GYROSCOPE_245DPS  0.0074768066F
#define SENSITIVITY_GYROSCOPE_500DPS  0.0152587891F
#define SENSITIVITY_GYROSCOPE_2000DPS 0.0610351563F
#define SENSITIVITY_MAGNETOMETER_4G   SENSITIVITY_ACCELEROMETER_4G
#define SENSITIVITY_MAGNETOMETER_8G   SENSITIVITY_ACCELEROMETER_8G
#define SENSITIVITY_MAGNETOMETER_12G  0.0003662109F
#define SENSITIVITY_MAGNETOMETER_16G  SENSITIVITY_ACCELEROMETER_16G

LSM9DS1::LSM9DS1()
{
	init(); //Initialise settings
}

void LSM9DS1::init()
{
	settings.gyro.enabled = true;
	settings.gyro.enableX = true;
	settings.gyro.enableY = true;
	settings.gyro.enableZ = true;
	settings.gyro.scale = G_SCALE_245DPS;
	settings.gyro.sampleRate = G_ODR_952HZ;
	// gyro cutoff frequency: value between 0-3
	// Actual value of cutoff frequency depends
	// on sample rate.
	settings.gyro.bandwidth = 0;
	settings.gyro.lowPowerEnable = false;
	settings.gyro.HPFEnable = false;
	// Gyro HPF cutoff frequency: value between 0-9
	// Actual value depends on sample rate. Only applies
	// if gyroHPFEnable is true.
	settings.gyro.HPFCutoff = 0;
	// Gyro HPF reference frequency: value between 0-255
	settings.gyro.HPFReference = 0;

	// Orientation adjustments
	settings.gyro.flipX = false;
	settings.gyro.flipY = false;
	settings.gyro.flipZ = false;
	settings.gyro.latchInterrupt = true;

	settings.accel.enabled = true;
	settings.accel.enableX = true;
	settings.accel.enableY = true;
	settings.accel.enableZ = true;
	settings.accel.scale = A_SCALE_2G;
	settings.accel.sampleRate = A_ODR_952HZ;
	settings.accel.filterBandwidth = A_FBW_SAMPLE_RATE;
	settings.accel.highResEnable = false;
	settings.accel.highResBandwidth = A_FHBW_ODR_DIV_50;

	settings.mag.enabled = true;
	settings.mag.scale = M_SCALE_4GS;
	settings.mag.sampleRate = M_ODR_80HZ;
	settings.mag.tempCompensationEnable = false;
	settings.mag.XYPerformance = MAG_ULTRA_HIGH_PERFORMANCE_MODE;
	settings.mag.ZPerformance = MAG_ULTRA_HIGH_PERFORMANCE_MODE;
	settings.mag.lowPowerEnable = false;
	settings.mag.operatingMode = MAG_CONTINUOUS_MODE;

	settings.temp.enabled = true;

	// Zero the biases of each of the sensors
	for (int i=0; i<3; i++)
	{
		// "Zero" the slope biases (note that the normal operation is one input unit is one output unit)
		gSlopeBias[i] = 1;
		aSlopeBias[i] = 1;
		mSlopeBias[i] = 1;

		// Zero the offset biases
		gOffsetBias[i] = 0;
		aOffsetBias[i] = 0;
		mOffsetBias[i] = 0;
	}
}


uint16_t LSM9DS1::begin(uint8_t agAddress, uint8_t mAddress, TwoWire &wirePort)
{
	// Set device settings, they are used in many other places
	settings.device.commInterface = IMU_MODE_I2C;
	settings.device.agAddress = agAddress;
	settings.device.mAddress = mAddress;
	settings.device.i2c = &wirePort;

	constrainScales();
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable

	// Initialize I2C library
	settings.device.i2c->begin();

	// Configure the communication speed (if it can)
	#if defined(ARDUINO_ARCH_MBED)
		settings.device.i2c->setClock(400000);
	#endif

	// Reboot device and settings to default
	xgWriteByte(CTRL_REG8, 0x05);
	mWriteByte(CTRL_REG2_M, 0x0c);

	// Allow the IMU time to reboot
	delay(10);

	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t mTest = mReadByte(WHO_AM_I_M);		// Read the gyro WHO_AM_I
	uint8_t xgTest = xgReadByte(WHO_AM_I_XG);	// Read the accel/mag WHO_AM_I
	uint16_t whoAmICombined = (xgTest << 8) | mTest;

	if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
		return 0;

	// Gyro initialization stuff:
	initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.

	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.

	// Magnetometer initialization stuff:
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.

	// Give the IMU some time to accept the new settings
	delay(50);

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return whoAmICombined;
}

uint16_t LSM9DS1::beginSPI(uint8_t ag_CS_pin, uint8_t m_CS_pin)
{
	// Set device settings, they are used in many other places
	settings.device.commInterface = IMU_MODE_SPI;
	settings.device.agAddress = ag_CS_pin;
	settings.device.mAddress = m_CS_pin;

	constrainScales();
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable

	// Now, initialize our hardware interface.
	initSPI();	// Initialize SPI

	// Reboot device and settings to default
	xgWriteByte(CTRL_REG8, 0x05);
	mWriteByte(CTRL_REG2_M, 0x0c);

	// Allow the IMU time to reboot
	delay(10);

	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t mTest = mReadByte(WHO_AM_I_M);		// Read the gyro WHO_AM_I
	uint8_t xgTest = xgReadByte(WHO_AM_I_XG);	// Read the accel/mag WHO_AM_I
	uint16_t whoAmICombined = (xgTest << 8) | mTest;

	if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
		return 0;

	// Gyro initialization stuff:
	initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.

	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.

	// Magnetometer initialization stuff:
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return whoAmICombined;
}

void LSM9DS1::initGyro()
{
	uint8_t tempRegValue = 0;

	// CTRL_REG1_G (Default value: 0x00)
	// [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	// ODR_G[2:0] - Output data rate selection
	// FS_G[1:0] - Gyroscope full-scale selection
	// BW_G[1:0] - Gyroscope bandwidth selection

	// To disable gyro, set sample rate bits to 0. We'll only set sample
	// rate if the gyro is enabled.
	if (settings.gyro.enabled)
	{
		tempRegValue = (settings.gyro.sampleRate << 5);
	}
	tempRegValue |= (settings.gyro.scale << 3);
	tempRegValue |= (settings.gyro.bandwidth & 0x3);
	xgWriteByte(CTRL_REG1_G, tempRegValue);

	// CTRL_REG2_G (Default value: 0x00)
	// [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
	// INT_SEL[1:0] - INT selection configuration
	// OUT_SEL[1:0] - Out selection configuration
	xgWriteByte(CTRL_REG2_G, 0x00);

	// CTRL_REG3_G (Default value: 0x00)
	// [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	// LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	// HP_EN - HPF enable (0:disabled, 1: enabled)
	// HPCF_G[3:0] - HPF cutoff frequency
	tempRegValue = settings.gyro.lowPowerEnable ? (1<<7) : 0;
	if (settings.gyro.HPFEnable)
	{
		tempRegValue |= (1<<6) | (settings.gyro.HPFCutoff & 0x0F);
	}
	xgWriteByte(CTRL_REG3_G, tempRegValue);

	// CTRL_REG4 (Default value: 0x38)
	// [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	// Zen_G - Z-axis output enable (0:disable, 1:enable)
	// Yen_G - Y-axis output enable (0:disable, 1:enable)
	// Xen_G - X-axis output enable (0:disable, 1:enable)
	// LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	// 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	tempRegValue = 0;
	if (settings.gyro.enableZ) tempRegValue |= (1<<5);
	if (settings.gyro.enableY) tempRegValue |= (1<<4);
	if (settings.gyro.enableX) tempRegValue |= (1<<3);
	if (settings.gyro.latchInterrupt) tempRegValue |= (1<<1);
	xgWriteByte(CTRL_REG4, tempRegValue);

	// ORIENT_CFG_G (Default value: 0x00)
	// [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
	// SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
	// Orient [2:0] - Directional user orientation selection
	tempRegValue = 0;
	if (settings.gyro.flipX) tempRegValue |= (1<<5);
	if (settings.gyro.flipY) tempRegValue |= (1<<4);
	if (settings.gyro.flipZ) tempRegValue |= (1<<3);
	// TODO: Add orientation config
	xgWriteByte(ORIENT_CFG_G, tempRegValue);

	// Set the reference G value for the filter
	xgWriteByte(REFERENCE_G, settings.gyro.HPFReference);
}

void LSM9DS1::initAccel()
{
	uint8_t tempRegValue = 0;

	//	CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//	Zen_XL - Z-axis output enabled
	//	Yen_XL - Y-axis output enabled
	//	Xen_XL - X-axis output enabled
	if (settings.accel.enableZ) tempRegValue |= (1<<5);
	if (settings.accel.enableY) tempRegValue |= (1<<4);
	if (settings.accel.enableX) tempRegValue |= (1<<3);

	xgWriteByte(CTRL_REG5_XL, tempRegValue);

	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	// ODR_XL[2:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_SCAL_ODR - Bandwidth selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	tempRegValue = 0;
	// To disable the accel, set the sampleRate bits to 0.
	if (settings.accel.enabled)
	{
		tempRegValue |= (settings.accel.sampleRate << 5);
	}
	// Set the scale value
	tempRegValue |= (settings.accel.scale << 3);
	if (settings.accel.filterBandwidth != A_FBW_SAMPLE_RATE)
	{
		tempRegValue |= (1<<2); // Set BW_SCAL_ODR
		tempRegValue |= settings.accel.filterBandwidth;
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);

	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	tempRegValue = 0;
	if (settings.accel.highResEnable)
	{
		tempRegValue |= (1<<7); // Set HR bit
		tempRegValue |= (settings.accel.highResBandwidth << 5);
	}
	xgWriteByte(CTRL_REG7_XL, tempRegValue);
}

/**
 * This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data,
 * average them, scales them to  gs and deg/s, respectively, and then passes the biases to the
 * main sketch for subtraction from all subsequent data. There are no gyro and accelerometer bias
 * registers to store the data as there are in the ADXL345, a precursor to the LSM9DS0, or the
 * MPU-9150, so we have to subtract the biases ourselves. This results in a more accurate measurement
 * in general and can remove errors due to imprecise or varying initial placement. Calibration of
 * sensor data in this manner is good practice.
 *
 * @param autoCalc If the calibration should be automatically applied to future readings
*/
void LSM9DS1::calibrate()
{
	/*
	uint8_t sampleCount = 0;
	int32_t aBiasRawTemp[3] = {0, 0, 0};
	int32_t gBiasRawTemp[3] = {0, 0, 0};

	// Turn on FIFO and set threshold to 32 samples
	enableFIFO(true);
	setFIFO(FIFO_THS, 0x1F);

	// Wait for FIFO to fill
	while (sampleCount < 0x1F)
	{
		sampleCount = (xgReadByte(FIFO_SRC) & 0x3F); // Read number of stored samples
	}

	// Read the device until the FIFO is empty
	for(uint8_t sample = 0; sample < sampleCount; sample++)
	{
		readRawGyro();
		gBiasRawTemp[0] += raw_gx;
		gBiasRawTemp[1] += raw_gy;
		gBiasRawTemp[2] += raw_gz;
		readRawAccel();
		aBiasRawTemp[0] += raw_ax;
		aBiasRawTemp[1] += raw_ay;
		aBiasRawTemp[2] += raw_az - (int16_t)(1./aRes); // Assumes sensor facing up!
	}

	// Calculate the biases for each axis
	for (uint8_t axis = 0; axis < 3; axis++)
	{
		gBiasRaw[axis] = gBiasRawTemp[axis] / sampleCount;
		gBias[axis] = gBiasRaw[axis] * gRes;
		aBiasRaw[axis] = aBiasRawTemp[axis] / sampleCount;
		aBias[axis] = aBiasRaw[axis] * aRes;
	}

	// No need for FIFO anymore, disable it
	enableFIFO(false);
	setFIFO(FIFO_OFF, 0x00);
	*/
}

void LSM9DS1::calibrateMag()
{
	/*
	int i, j;
	int16_t magMin[3] = {0, 0, 0};
	int16_t magMax[3] = {0, 0, 0};

	for (i=0; i<128; i++)
	{
		while (!magAvailable())
			;
		readRawMag();
		int16_t magTemp[3] = {0, 0, 0};
		magTemp[0] = raw_mx;
		magTemp[1] = raw_my;
		magTemp[2] = raw_mz;
		for (j = 0; j < 3; j++)
		{
			if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
			if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
		}
	}
	for (j = 0; j < 3; j++)
	{
		mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
		mBias[j] = mBiasRaw[j] * mRes;
		if (loadIn)
			magOffset(j, mBiasRaw[j]);
	}
	*/
}

void LSM9DS1::initMag()
{
	uint8_t tempRegValue = 0;

	// CTRL_REG1_M (Default value: 0x10)
	// [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
	// TEMP_COMP - Temperature compensation
	// OM[1:0] - X & Y axes op mode selection
	//	00:low-power, 01:medium performance
	//	10: high performance, 11:ultra-high performance
	// DO[2:0] - Output data rate selection
	// ST - Self-test enable
	if (settings.mag.tempCompensationEnable) tempRegValue |= (1<<7);
	tempRegValue |= (settings.mag.XYPerformance << 5);
	tempRegValue |= (settings.mag.sampleRate << 2);
	mWriteByte(CTRL_REG1_M, tempRegValue);

	// CTRL_REG2_M (Default value 0x00)
	// [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
	// FS[1:0] - Full-scale configuration
	// REBOOT - Reboot memory content (0:normal, 1:reboot)
	// SOFT_RST - Reset config and user registers (0:default, 1:reset)
	tempRegValue = (settings.mag.scale << 5);
	mWriteByte(CTRL_REG2_M, tempRegValue);

	// CTRL_REG3_M (Default value: 0x03)
	// [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
	// I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
	// LP - Low-power mode cofiguration (1:enable)
	// SIM - SPI mode selection (0:write-only, 1:read/write enable)
	// MD[1:0] - Operating mode
	//	00:continuous conversion, 01:single-conversion,
	//  10,11: Power-down
	tempRegValue = 0;
	if (settings.mag.lowPowerEnable) tempRegValue |= (1<<5);
	tempRegValue |= settings.mag.operatingMode;
	mWriteByte(CTRL_REG3_M, tempRegValue); // Continuous conversion mode

	// CTRL_REG4_M (Default value: 0x00)
	// [0][0][0][0][OMZ1][OMZ0][BLE][0]
	// OMZ[1:0] - Z-axis operative mode selection
	//	00:low-power mode, 01:medium performance
	//	10:high performance, 10:ultra-high performance
	// BLE - Big/little endian data
	tempRegValue = 0;
	tempRegValue = (settings.mag.ZPerformance << 2);
	mWriteByte(CTRL_REG4_M, tempRegValue);

	// CTRL_REG5_M (Default value: 0x00)
	// [0][BDU][0][0][0][0][0][0]
	// BDU - Block data update for magnetic data
	//	0:continuous, 1:not updated until MSB/LSB are read
	tempRegValue = 0;
	mWriteByte(CTRL_REG5_M, tempRegValue);
}


// Taken from https://github.com/FemmeVerbeek/Arduino_LSM9DS1/blob/master/src/LSM9DS1.cpp
void LSM9DS1::setAccelOffset(float x, float y, float z)
{  aOffsetBias[0] = x / aSlopeBias[0];
   aOffsetBias[1] = y / aSlopeBias[1];
   aOffsetBias[2] = z / aSlopeBias[2];
}

//Slope is already dimensionless, so it can be stored as is.
void LSM9DS1::setAccelSlope(float x, float y, float z)
{  if (x==0) x=1;  //zero slope not allowed
   if (y==0) y=1;
   if (z==0) z=1;
   aSlopeBias[0] = x;
   aSlopeBias[1] = y;
   aSlopeBias[2] = z;
}

void LSM9DS1::setGyroOffset(float x, float y, float z)
{  gOffsetBias[0] = x / gSlopeBias[0];
   gOffsetBias[1] = y / gSlopeBias[1];
   gOffsetBias[2] = z / gSlopeBias[2];
}

// Slope is already dimensionless, so it can be stored as is.
void LSM9DS1::setGyroSlope(float x, float y, float z)
{  if (x==0) x=1;  //zero slope not allowed
   if (y==0) y=1;
   if (z==0) z=1;
   gSlopeBias[0] = x;
   gSlopeBias[1] = y;
   gSlopeBias[2] = z;
}

void LSM9DS1::setMagOffset(float x, float y, float z)
{  mOffsetBias[0] = x / mSlopeBias[0];
   mOffsetBias[1] = y / mSlopeBias[1];
   mOffsetBias[2] = z / mSlopeBias[2];
}

//Slope is already dimensionless, so it can be stored as is.
void LSM9DS1::setMagSlope(float x, float y, float z)
{  if (x==0) x=1;  //zero slope not allowed
   if (y==0) y=1;
   if (z==0) z=1;
   mSlopeBias[0] = x;
   mSlopeBias[1] = y;
   mSlopeBias[2] = z;
}

bool LSM9DS1::accelAvailable()
{
	return READ_BIT(xgReadByte(STATUS_REG_1), 0);
}

bool LSM9DS1::gyroAvailable()
{
	return READ_BIT(xgReadByte(STATUS_REG_1), 1);
}

bool LSM9DS1::tempAvailable()
{
	return READ_BIT(xgReadByte(STATUS_REG_1), 2);
}

bool LSM9DS1::magAvailable(lsm9ds1_axis axis)
{
	return READ_BIT(mReadByte(STATUS_REG_M), axis);
}

void LSM9DS1::readRawAccel()
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp
	if ( xgReadBytes(OUT_X_L_XL, temp, 6) == 6 ) // Read 6 bytes, beginning at OUT_X_L_XL
	{
		raw_ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
		raw_ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
		raw_az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
	}
}

void LSM9DS1::readAccel(bool biasCorrection) {
	readRawAccel();

	// Apply bias and scaling (if specified)
	if (biasCorrection) {
		ax = ((raw_ax * aRes) - aOffsetBias[0]) * aSlopeBias[0];
		ay = ((raw_ay * aRes) - aOffsetBias[1]) * aSlopeBias[1];
		az = ((raw_az * aRes) - aOffsetBias[2]) * aSlopeBias[2];
	}
	else {
		ax = raw_ax * aRes;
		ay = raw_ay * aRes;
		az = raw_az * aRes;
	}
}

int16_t LSM9DS1::readRawAccel(lsm9ds1_axis axis)
{
	uint8_t temp[2];
	int16_t value;
	if ( xgReadBytes(OUT_X_L_XL + (2 * axis), temp, 2) == 2)
	{
		value = (temp[1] << 8) | temp[0];
		return value;
	}
	return 0;
}

float LSM9DS1::readAccel(lsm9ds1_axis axis, bool biasCorrection) {
	if (biasCorrection) {
		return ((readRawAccel(axis) * aRes) - aOffsetBias[axis]) * aSlopeBias[axis];
	}
	else {
		return readRawAccel(axis) * aRes;
	}
}

void LSM9DS1::readRawMag()
{
	uint8_t temp[6]; // We'll read six bytes from the mag into temp
	if ( mReadBytes(OUT_X_L_M, temp, 6) == 6) // Read 6 bytes, beginning at OUT_X_L_M
	{
		raw_mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
		raw_my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
		raw_mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
	}
}

void LSM9DS1::readMag(bool biasCorrection) {
	readRawMag();

	// Apply bias and scaling (if specified)
	if (biasCorrection) {
		mx = ((raw_mx * mRes) - mOffsetBias[0]) * mSlopeBias[0];
		my = ((raw_my * mRes) - mOffsetBias[1]) * mSlopeBias[1];
		mz = ((raw_mz * mRes) - mOffsetBias[2]) * mSlopeBias[2];
	}
	else {
		mx = raw_mx * mRes;
		my = raw_my * mRes;
		mz = raw_mz * mRes;
	}
}

int16_t LSM9DS1::readRawMag(lsm9ds1_axis axis)
{
	uint8_t temp[2];
	if ( mReadBytes(OUT_X_L_M + (2 * axis), temp, 2) == 2)
	{
		return (temp[1] << 8) | temp[0];
	}
	return 0;
}

float LSM9DS1::readMag(lsm9ds1_axis axis, bool biasCorrection) {
	if (biasCorrection) {
		return ((readRawMag(axis) * mRes) - mOffsetBias[axis]) * mSlopeBias[axis];
	}
	else {
		return readRawMag(axis) * mRes;
	}
}

typedef union
{
	struct
	{
		int16_t data :12;
		int16_t Reserved :4;
	};
	int16_t word;
} int12_t;

void LSM9DS1::readTemp()
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp
	if ( xgReadBytes(OUT_TEMP_L, temp, 2) == 2 ) // Read 2 bytes, beginning at OUT_TEMP_L
	{
		int12_t degree;
		degree.word = ( ( int16_t )temp[ 1 ] << 8 ) | ( int16_t ) temp[ 0 ];

		int16_t offset = 25;  // Per datasheet sensor outputs 0 typically @ 25 degrees centigrade
		temperature = offset + degree.data / 16;
	}
}

void LSM9DS1::readRawGyro()
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	if ( xgReadBytes(OUT_X_L_G, temp, 6) == 6) // Read 6 bytes, beginning at OUT_X_L_G
	{
		raw_gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
		raw_gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
		raw_gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
	}
}

void LSM9DS1::readGyro(bool biasCorrection) {
	readRawGyro();

	// Apply bias and scaling (if specified)
	if (biasCorrection) {
		gx = ((raw_gx * gRes) - gOffsetBias[0]) * gSlopeBias[0];
		gy = ((raw_gy * gRes) - gOffsetBias[1]) * gSlopeBias[1];
		gz = ((raw_gz * gRes) - gOffsetBias[2]) * gSlopeBias[2];
	}
	else {
		gx = raw_gx * gRes;
		gy = raw_gy * gRes;
		gz = raw_gz * gRes;
	}
}

int16_t LSM9DS1::readRawGyro(lsm9ds1_axis axis)
{
	uint8_t temp[2];
	int16_t value;

	if ( xgReadBytes(OUT_X_L_G + (2 * axis), temp, 2) == 2)
	{
		value = (temp[1] << 8) | temp[0];
		return value;
	}
	return 0;
}

void LSM9DS1::setGyroScale(gyro_scale gScl)
{
	// Read current value of CTRL_REG1_G:
	uint8_t ctrl1RegValue = xgReadByte(CTRL_REG1_G);

	// Mask out scale bits (3 & 4):
	ctrl1RegValue &= 0b11100111;
	ctrl1RegValue |= (gScl << 3);
	xgWriteByte(CTRL_REG1_G, ctrl1RegValue);

	// Save the new scaling
	settings.gyro.scale = gScl;

	// Calculate the new LSB to deg/s ratio
	calcgRes();
}

void LSM9DS1::setAccelScale(accel_scale aScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t tempRegValue = xgReadByte(CTRL_REG6_XL);

	// Mask out accel scale bits:
	tempRegValue &= 0b11100111;
	tempRegValue |= (aScl << 3);
	xgWriteByte(CTRL_REG6_XL, tempRegValue);

	// Save the new scaling
	settings.accel.scale = aScl;

	// Calculate the new LSB to gs ratio
	calcaRes();
}

void LSM9DS1::setMagScale(mag_scale mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t ctrl2Reg = mReadByte(CTRL_REG2_M);

	// We can just zero the entire register, as the other bits are only set
	// when rebooting and are immediately cleared
	ctrl2Reg = 0;

	// Set the new scale value
	ctrl2Reg |= (mScl << 5);

	// And write the new register value back into CTRL_REG6_XM:
	mWriteByte(CTRL_REG2_M, ctrl2Reg);

	// Update the LSB to gauss ratio
	calcmRes();
}

void LSM9DS1::setGyroODR(gyro_odr gRate)
{
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG1_G);

	// Clear the ODR bits
	temp &= 0b00011111;

	// Set the new rate
	temp |= (gRate << 5);

	// Update our settings struct
	settings.gyro.sampleRate = gRate;

	// And write the new register value back into CTRL_REG1_G:
	xgWriteByte(CTRL_REG1_G, temp);
}

void LSM9DS1::setAccelODR(accel_odr aRate)
{
	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG6_XL);

	// Then mask out the accel ODR bits:
	temp &= 0b00011111;

	// Then shift in our new ODR bits:
	temp |= (aRate << 5);

	// Save the new sample rate
	settings.accel.sampleRate = aRate;

	// And write the new register value back into CTRL_REG1_XM:
	xgWriteByte(CTRL_REG6_XL, temp);
}

void LSM9DS1::setMagODR(mag_odr mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG1_M);

	// Then mask out the mag ODR bits:
	temp &= 0b11100011;

	// Then shift in our new ODR bits:
	temp |= (mRate << 2);

	// Save the new sample rate
	settings.mag.sampleRate = mRate;

	// And write the new register value back into CTRL_REG5_XM:
	mWriteByte(CTRL_REG1_M, temp);
}

void LSM9DS1::calcgRes()
{
	switch (settings.gyro.scale)
	{
	case G_SCALE_245DPS:
		gRes = SENSITIVITY_GYROSCOPE_245DPS;
		break;
	case G_SCALE_500DPS:
		gRes = SENSITIVITY_GYROSCOPE_500DPS;
		break;
	case G_SCALE_2000DPS:
		gRes = SENSITIVITY_GYROSCOPE_2000DPS;
		break;
	default:
		break;
	}
}

void LSM9DS1::calcaRes()
{
	switch (settings.accel.scale)
	{
	case A_SCALE_2G:
		aRes = SENSITIVITY_ACCELEROMETER_2G;
		break;
	case A_SCALE_4G:
		aRes = SENSITIVITY_ACCELEROMETER_4G;
		break;
	case A_SCALE_8G:
		aRes = SENSITIVITY_ACCELEROMETER_8G;
		break;
	case A_SCALE_16G:
		aRes = SENSITIVITY_ACCELEROMETER_16G;
		break;
	default:
		break;
	}
}

void LSM9DS1::calcmRes()
{
	switch (settings.mag.scale)
	{
	case M_SCALE_4GS:
		mRes = SENSITIVITY_MAGNETOMETER_4G;
		break;
	case M_SCALE_8GS:
		mRes = SENSITIVITY_MAGNETOMETER_8G;
		break;
	case M_SCALE_12GS:
		mRes = SENSITIVITY_MAGNETOMETER_12G;
		break;
	case M_SCALE_16GS:
		mRes = SENSITIVITY_MAGNETOMETER_16G;
		break;
	}
}

void LSM9DS1::configInt(interrupt_select interrupt, uint8_t generator,
	                     h_lactive activeLow, pp_od pushPull)
{
	// Write to INT1_CTRL or INT2_CTRL. [interupt] should already be one of
	// those two values.
	// [generator] should be an OR'd list of values from the interrupt_generators enum
	xgWriteByte(interrupt, generator);

	// Configure CTRL_REG8
	uint8_t temp;
	temp = xgReadByte(CTRL_REG8);

	if (activeLow) temp |= (1<<5);
	else temp &= ~(1<<5);

	if (pushPull) temp &= ~(1<<4);
	else temp |= (1<<4);

	xgWriteByte(CTRL_REG8, temp);
}

void LSM9DS1::configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn)
{
	uint8_t temp = 0;

	temp = threshold & 0x7F;
	if (sleepOn) temp |= (1<<7);
	xgWriteByte(ACT_THS, temp);

	xgWriteByte(ACT_DUR, duration);
}

uint8_t LSM9DS1::getInactivity()
{
	uint8_t temp = xgReadByte(STATUS_REG_0);
	temp &= (0x10);
	return temp;
}

void LSM9DS1::configAccelInt(uint8_t generator, bool andInterrupts)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (andInterrupts) temp |= 0x80;
	xgWriteByte(INT_GEN_CFG_XL, temp);
}

void LSM9DS1::configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	// Write threshold value to INT_GEN_THS_?_XL.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	xgWriteByte(INT_GEN_THS_X_XL + axis, threshold);

	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	xgWriteByte(INT_GEN_DUR_XL, temp);
}

uint8_t LSM9DS1::getAccelIntSrc()
{
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_XL);

	// Check if the IA_XL (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}

	return 0;
}

void LSM9DS1::configGyroInt(uint8_t generator, bool aoi, bool latch)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (aoi) temp |= 0x80;
	if (latch) temp |= 0x40;
	xgWriteByte(INT_GEN_CFG_G, temp);
}

void LSM9DS1::configGyroThs(int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	uint8_t buffer[2];
	buffer[0] = (threshold & 0x7F00) >> 8;
	buffer[1] = (threshold & 0x00FF);
	// Write threshold value to INT_GEN_THS_?H_G and  INT_GEN_THS_?L_G.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	xgWriteByte(INT_GEN_THS_XH_G + (axis * 2), buffer[0]);
	xgWriteByte(INT_GEN_THS_XH_G + 1 + (axis * 2), buffer[1]);

	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	xgWriteByte(INT_GEN_DUR_G, temp);
}

uint8_t LSM9DS1::getGyroIntSrc()
{
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_G);

	// Check if the IA_G (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}

	return 0;
}

void LSM9DS1::configMagInt(uint8_t generator, h_lactive activeLow, bool latch)
{
	// Mask out non-generator bits (0-4)
	uint8_t config = (generator & 0xE0);
	// IEA bit is 0 for active-low, 1 for active-high.
	if (activeLow == INT_ACTIVE_HIGH) config |= (1<<2);
	// IEL bit is 0 for latched, 1 for not-latched
	if (!latch) config |= (1<<1);
	// As long as we have at least 1 generator, enable the interrupt
	if (generator != 0) config |= (1<<0);

	mWriteByte(INT_CFG_M, config);
}

void LSM9DS1::configMagThs(uint16_t threshold)
{
	// Write high eight bits of [threshold] to INT_THS_H_M
	mWriteByte(INT_THS_H_M, uint8_t((threshold & 0x7F00) >> 8));
	// Write low eight bits of [threshold] to INT_THS_L_M
	mWriteByte(INT_THS_L_M, uint8_t(threshold & 0x00FF));
}

uint8_t LSM9DS1::getMagIntSrc()
{
	uint8_t intSrc = mReadByte(INT_SRC_M);

	// Check if the INT (interrupt active) bit is set
	if (intSrc & (1<<0))
	{
		return (intSrc & 0xFE);
	}

	return 0;
}

void LSM9DS1::sleepGyro(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);
	if (enable) temp |= (1<<6);
	else temp &= ~(1<<6);
	xgWriteByte(CTRL_REG9, temp);
}

void LSM9DS1::enableFIFO(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);
	if (enable) temp |= (1<<1);
	else temp &= ~(1<<1);
	xgWriteByte(CTRL_REG9, temp);
}

void LSM9DS1::setFIFO(fifoMode_type fifoMode, uint8_t fifoThs)
{
	// Limit threshold - 0x1F (31) is the maximum. If more than that was asked
	// limit it to the maximum.
	uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
	xgWriteByte(FIFO_CTRL, ((fifoMode << 5) | (threshold & 0b00011111)));
}

uint8_t LSM9DS1::getFIFOSamples()
{
	return (xgReadByte(FIFO_SRC) & 0x3F);
}

void LSM9DS1::constrainScales()
{
	if ((settings.gyro.scale != G_SCALE_245DPS) && (settings.gyro.scale != G_SCALE_500DPS) &&
		(settings.gyro.scale != G_SCALE_2000DPS))
	{
		settings.gyro.scale = G_SCALE_245DPS;
	}

	if ((settings.accel.scale != A_SCALE_2G) && (settings.accel.scale != A_SCALE_4G) &&
		(settings.accel.scale != A_SCALE_8G) && (settings.accel.scale != A_SCALE_16G))
	{
		settings.accel.scale = A_SCALE_2G;
	}

	if ((settings.mag.scale != M_SCALE_4GS) && (settings.mag.scale != M_SCALE_8GS) &&
		(settings.mag.scale != M_SCALE_12GS) && (settings.mag.scale != M_SCALE_16GS))
	{
		settings.mag.scale = M_SCALE_4GS;
	}
}

void LSM9DS1::xgWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		I2CwriteByte(settings.device.agAddress, subAddress, data);
	else if (settings.device.commInterface == IMU_MODE_SPI)
		SPIwriteByte(settings.device.agAddress, subAddress, data);
}

void LSM9DS1::mWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		return I2CwriteByte(settings.device.mAddress, subAddress, data);
	else if (settings.device.commInterface == IMU_MODE_SPI)
		return SPIwriteByte(settings.device.mAddress, subAddress, data);
}

uint8_t LSM9DS1::xgReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadByte(settings.device.agAddress, subAddress);
	else if (settings.device.commInterface == IMU_MODE_SPI)
		return SPIreadByte(settings.device.agAddress, subAddress);
	return -1;
}

uint8_t LSM9DS1::xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// gyro-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadBytes(settings.device.agAddress, subAddress, dest, count);
	else if (settings.device.commInterface == IMU_MODE_SPI)
		return SPIreadBytes(settings.device.agAddress, subAddress, dest, count);
	return -1;
}

uint8_t LSM9DS1::mReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadByte(settings.device.mAddress, subAddress);
	else if (settings.device.commInterface == IMU_MODE_SPI)
		return SPIreadByte(settings.device.mAddress, subAddress);
	return -1;
}

uint8_t LSM9DS1::mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadBytes(settings.device.mAddress, subAddress, dest, count);
	else if (settings.device.commInterface == IMU_MODE_SPI)
		return SPIreadBytes(settings.device.mAddress, subAddress, dest, count);
	return -1;
}

void LSM9DS1::initSPI()
{
	pinMode(settings.device.agAddress, OUTPUT);
	digitalWrite(settings.device.agAddress, HIGH);
	pinMode(settings.device.mAddress, OUTPUT);
	digitalWrite(settings.device.mAddress, HIGH);

	SPI.begin();
	// Maximum SPI frequency is 10MHz
	// Data is read and written MSb first.
	// Data is read and written MSb first.
	// Data is captured on rising edge of clock (CPHA = 0)
	// Base value of the clock is HIGH (CPOL = 1)
	SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
}

void LSM9DS1::SPIwriteByte(uint8_t csPin, uint8_t subAddress, uint8_t data)
{
	digitalWrite(csPin, LOW); // Initiate communication

	// If write, bit 0 (MSB) should be 0
	// If single write, bit 1 should be 0
	SPI.transfer(subAddress & 0x3F); // Send Address
	SPI.transfer(data); // Send data

	digitalWrite(csPin, HIGH); // Close communication
}

uint8_t LSM9DS1::SPIreadByte(uint8_t csPin, uint8_t subAddress)
{
	uint8_t temp;
	// Use the multiple read function to read 1 byte.
	// Value is returned to `temp`.
	SPIreadBytes(csPin, subAddress, &temp, 1);
	return temp;
}

uint8_t LSM9DS1::SPIreadBytes(uint8_t csPin, uint8_t subAddress,
							uint8_t * dest, uint8_t count)
{
	// To indicate a read, set bit 0 (msb) of first byte to 1
	uint8_t rAddress = 0x80 | (subAddress & 0x3F);
	// Mag SPI port is different. If we're reading multiple bytes,
	// set bit 1 to 1. The remaining six bytes are the address to be read
	if ((csPin == settings.device.mAddress) && count > 1)
		rAddress |= 0x40;

	digitalWrite(csPin, LOW); // Initiate communication
	SPI.transfer(rAddress);
	for (int i=0; i<count; i++)
	{
		dest[i] = SPI.transfer(0x00); // Read into destination array
	}
	digitalWrite(csPin, HIGH); // Close communication

	return count;
}

// Wire.h read and write protocols
void LSM9DS1::I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	settings.device.i2c->beginTransmission(address);  // Initialize the Tx buffer
	settings.device.i2c->write(subAddress);           // Put slave register address in Tx buffer
	settings.device.i2c->write(data);                 // Put data in Tx buffer
	settings.device.i2c->endTransmission();           // Send the Tx buffer
}

uint8_t LSM9DS1::I2CreadByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data

	settings.device.i2c->beginTransmission(address);         // Initialize the Tx buffer
	settings.device.i2c->write(subAddress);	                 // Put slave register address in Tx buffer
	settings.device.i2c->endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	settings.device.i2c->requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address

	data = settings.device.i2c->read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

uint8_t LSM9DS1::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	byte retVal;
	settings.device.i2c->beginTransmission(address);      // Initialize the Tx buffer
	// Next send the register to be read. OR with 0x80 to indicate multi-read.
	settings.device.i2c->write(subAddress | 0x80);        // Put slave register address in Tx buffer
	retVal = settings.device.i2c->endTransmission(false); // Send Tx buffer, send a restart to keep connection alive
	if (retVal != 0) // endTransmission should return 0 on success
		return 0;

	retVal = settings.device.i2c->requestFrom(address, count);  // Read bytes from slave register address
	if (retVal != count)
		return 0;

	for (int i=0; i<count;)
		dest[i++] = settings.device.i2c->read();

	return count;
}
