/******************************************************************************
SFE_LSM9DS1.h
SFE_LSM9DS1 Library Header File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 27, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file prototypes the LSM9DS1 class, implemented in SFE_LSM9DS1.cpp. In
addition, it defines every register in the LSM9DS1 (both the Gyro and Accel/
Magnetometer registers).

Development environment specifics:
	IDE: Arduino 1.6.0
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/
#ifndef __SparkFunLSM9DS1_H__
#define __SparkFunLSM9DS1_H__

//#define DEBUGGING

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
#endif

#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

#define LSM9DS1_AG_ADDR(sa0)	((sa0) == 0 ? 0x6A : 0x6B)
#define LSM9DS1_M_ADDR(sa1)		((sa1) == 0 ? 0x1C : 0x1E)

#define READ_BIT(raw, bitPos) ((raw >> bitPos) & 0x01)

enum lsm9ds1_axis {
	X_AXIS   = 0,
	Y_AXIS   = 1,
	Z_AXIS   = 2,
	ALL_AXIS = 3,
};

class LSM9DS1
{
public:
	IMUSettings settings;

	// We'll store the gyro, accel, and magnetometer readings in a series of
	// public class variables. Each sensor gets three variables -- one for each
	// axis. Call readGyro(), readAccel(), and readMag() first, before using
	// these variables!
	// These values are the RAW signed 16-bit readings from the sensors.
	int16_t raw_gx, raw_gy, raw_gz; // x, y, and z axis readings of the gyroscope
	int16_t raw_ax, raw_ay, raw_az; // x, y, and z axis readings of the accelerometer
	int16_t raw_mx, raw_my, raw_mz; // x, y, and z axis readings of the magnetometer
    int16_t raw_temperature; // Chip temperature

	// These values are the actual readings from the sensors.
	float gx, gy, gz; // x, y, and z axis readings of the gyroscope
	float ax, ay, az; // x, y, and z axis readings of the accelerometer
	float mx, my, mz; // x, y, and z axis readings of the magnetometer
    float temperature; // Chip temperature

	// Slope biases
	float gSlopeBias[3]; // Slope of the gyroscope bias
	float aSlopeBias[3]; // Slope of the accelerometer bias
	float mSlopeBias[3]; // Slope of the magnetometer bias

	// Offset biases
	float gOffsetBias[3]; // Offset of the gyroscope bias
	float aOffsetBias[3]; // Offset of the accelerometer bias
	float mOffsetBias[3]; // Offset of the magnetometer bias

	// LSM9DS1 -- LSM9DS1 class constructor
	// The constructor will set up a handful of private variables, and set the
	// communication mode as well.
	// Input:
	//	- interface = Either IMU_MODE_SPI or IMU_MODE_I2C, whichever you're using
	//				to talk to the IC.
	//	- xgAddr = If IMU_MODE_I2C, this is the I2C address of the accel/gyroscope.
	// 				If IMU_MODE_SPI, this is the chip select pin of the gyro (CS_AG)
	//	- mAddr = If IMU_MODE_I2C, this is the I2C address of the magnetometer.
	//				If IMU_MODE_SPI, this is the cs pin of the magnetometer (CS_M)
	LSM9DS1();

	// begin() and beginSPI() -- The values set in the IMUSettings struct will take
	// effect after calling this function.
	// INPUTS:
	// - agAddress - Sets either the I2C address of the accel/gyro or SPI chip
	//   select pin connected to the CS_XG pin.
	// - mAddress - Sets either the I2C address of the magnetometer or SPI chip
	//   select pin connected to the CS_M pin.
	// - i2C port (Note, only on "begin()" funtion, for use with I2C com interface)
	//   defaults to Wire, but if hardware supports it, can use other TwoWire ports.
	//   **For SPI use "beginSPI()", and only send first two address arguments.
	uint16_t begin(uint8_t agAddress = LSM9DS1_AG_ADDR(1), uint8_t mAddress = LSM9DS1_M_ADDR(1), TwoWire& wirePort = Wire); //By default use the default I2C addres, and use Wire port
	uint16_t beginSPI(uint8_t ag_CS_pin, uint8_t m_CS_pin);

	// Bias corrections (slope)
	void setAccelSlopeBiases(float x, float y, float z);
	void setGyroSlopeBiases(float x, float y, float z);
	void setMagSlopeBiases(float x, float y, float z);

	void setAccelSlopeBiases(float biases[3]);
	void setGyroSlopeBiases(float biases[3]);
	void setMagSlopeBiases(float biases[3]);

	// Bias corrections (offsets)
	void setAccelOffsetBiases(float x, float y, float z);
	void setGyroOffsetBiases(float x, float y, float z);
	void setMagOffsetBiases(float x, float y, float z);

	void setAccelOffsetBiases(float biases[3]);
	void setGyroOffsetBiases(float biases[3]);
	void setMagOffsetBiases(float biases[3]);

	// accelAvailable() -- Polls the accelerometer status register to check
	// if new data is available.
	// Output:	1 - New data available
	//			0 - No new data available
	bool accelAvailable();

	// gyroAvailable() -- Polls the gyroscope status register to check
	// if new data is available.
	// Output:	1 - New data available
	//			0 - No new data available
	bool gyroAvailable();

	// tempAvailable() -- Polls the temperature status register to check
	// if new data is available.
	// Output:	1 - New data available
	//			0 - No new data available
	bool tempAvailable();

	// magAvailable() -- Polls the accelerometer status register to check
	// if new data is available.
	// Input:
	//	- axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
	//	  on one specific axis. Or ALL_AXIS (default) to check for new data
	//	  on all axes.
	// Output:	1 - New data available
	//			0 - No new data available
	bool magAvailable(lsm9ds1_axis axis = ALL_AXIS);

	// readRawGyro() -- Read the gyroscope output registers.
	// This function will read all six gyroscope output registers.
	// The readings are stored in the class' gx, gy, and gz variables. Read
	// those _after_ calling readGyro().
	void readRawGyro();
	void readGyro(bool biasCorrection = true);

	// int16_t readRawGyro(axis) -- Read a specific axis of the gyroscope.
	// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
	// Input:
	//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
	// Output:
	//	A 16-bit signed integer with sensor data on requested axis.
	int16_t readRawGyro(lsm9ds1_axis axis);
	float readGyro(lsm9ds1_axis axis, bool biasCorrection = true);

	// readRawAccel() -- Read the accelerometer output registers.
	// This function will read all six accelerometer output registers.
	// The readings are stored in the class' ax, ay, and az variables. Read
	// those _after_ calling readAccel().
	void readRawAccel();
	void readAccel(bool biasCorrection = true);

	// int16_t readRawAccel(axis) -- Read a specific axis of the accelerometer.
	// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
	// Input:
	//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
	// Output:
	//	A 16-bit signed integer with sensor data on requested axis.
	int16_t readRawAccel(lsm9ds1_axis axis);
	float readAccel(lsm9ds1_axis axis, bool biasCorrection = true);

	// readRawMag() -- Read the magnetometer output registers.
	// This function will read all six magnetometer output registers.
	// The readings are stored in the class' mx, my, and mz variables. Read
	// those _after_ calling readMag().
	void readRawMag();
	void readMag(bool biasCorrection = true);

	// int16_t readRawMag(axis) -- Read a specific axis of the magnetometer.
	// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
	// Input:
	//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
	// Output:
	//	A 16-bit signed integer with sensor data on requested axis.
	int16_t readRawMag(lsm9ds1_axis axis);
	float readMag(lsm9ds1_axis axis, bool biasCorrection = true);

	// readRawTemp() -- Read the temperature output register.
	// This function will read two temperature output registers.
	// The combined readings are stored in the class' temperature variables. Read
	// those _after_ calling readTemp().
	void readRawTemp();
	void readTemp();

	// setGyroScale() -- Set the full-scale range of the gyroscope.
	// This function can be called to set the scale of the gyroscope to
	// 245, 500, or 200 degrees per second.
	// Input:
	// 	- gScl = The desired gyroscope scale. Must be one of three possible
	//		values from the gyro_scale.
	void setGyroScale(gyro_scale gScl);
	uint16_t getGyroScaleInt();

	// setAccelScale() -- Set the full-scale range of the accelerometer.
	// This function can be called to set the scale of the accelerometer to
	// 2, 4, 6, 8, or 16 g's.
	// Input:
	// 	- aScl = The desired accelerometer scale. Must be one of five possible
	//		values from the accel_scale.
	void setAccelScale(accel_scale aScl);
	uint8_t getAccelScaleInt();

	// setMagScale() -- Set the full-scale range of the magnetometer.
	// This function can be called to set the scale of the magnetometer to
	// 2, 4, 8, or 12 Gs.
	// Input:
	// 	- mScl = The desired magnetometer scale. Must be one of four possible
	//		values from the mag_scale.
	void setMagScale(mag_scale mScl);
	uint8_t getMagScaleInt();

	// setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
	// Input:
	//	- gRate = The desired output rate and cutoff frequency of the gyro.
	void setGyroODR(gyro_odr gRate);
	float getGyroODRFloat();

	// setAccelODR() -- Set the output data rate of the accelerometer
	// Input:
	//	- aRate = The desired output rate of the accel.
	void setAccelODR(accel_odr aRate);
	float getAccelODRFloat();

	// setMagODR() -- Set the output data rate of the magnetometer
	// Input:
	//	- mRate = The desired output rate of the mag.
	void setMagODR(mag_odr mRate);
	float getMagODRFloat();

	// configInactivity() -- Configure inactivity interrupt parameters
	// Input:
	//	- duration = Inactivity duration - actual value depends on gyro ODR
	//	- threshold = Activity Threshold
	//	- sleepOn = Gyroscope operating mode during inactivity.
	//	  true: gyroscope in sleep mode
	//	  false: gyroscope in power-down
	void configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn);

	// configAccelInt() -- Configure Accelerometer Interrupt Generator
	// Input:
	//	- generator = Interrupt axis/high-low events
	//	  Any OR'd combination of ZHIE_XL, ZLIE_XL, YHIE_XL, YLIE_XL, XHIE_XL, XLIE_XL
	//	- andInterrupts = AND/OR combination of interrupt events
	//	  true: AND combination
	//	  false: OR combination
	void configAccelInt(uint8_t generator, bool andInterrupts = false);

	// configAccelThs() -- Configure the threshold of an accelereomter axis
	// Input:
	//	- threshold = Interrupt threshold. Possible values: 0-255.
	//	  Multiply by 128 to get the actual raw accel value.
	//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
	//	- duration = Duration value must be above or below threshold to trigger interrupt
	//	- wait = Wait function on duration counter
	//	  true: Wait for duration samples before exiting interrupt
	//	  false: Wait function off
	void configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration = 0, bool wait = 0);

	// configGyroInt() -- Configure Gyroscope Interrupt Generator
	// Input:
	//	- generator = Interrupt axis/high-low events
	//	  Any OR'd combination of ZHIE_G, ZLIE_G, YHIE_G, YLIE_G, XHIE_G, XLIE_G
	//	- aoi = AND/OR combination of interrupt events
	//	  true: AND combination
	//	  false: OR combination
	//	- latch: latch gyroscope interrupt request.
	void configGyroInt(uint8_t generator, bool aoi, bool latch);

	// configGyroThs() -- Configure the threshold of a gyroscope axis
	// Input:
	//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
	//	  Value is equivalent to raw gyroscope value.
	//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
	//	- duration = Duration value must be above or below threshold to trigger interrupt
	//	- wait = Wait function on duration counter
	//	  true: Wait for duration samples before exiting interrupt
	//	  false: Wait function off
	void configGyroThs(int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait);

	// configInt() -- Configure INT1 or INT2 (Gyro and Accel Interrupts only)
	// Input:
	//	- interrupt = Select INT1 or INT2
	//	  Possible values: XG_INT1 or XG_INT2
	//	- generator = Or'd combination of interrupt generators.
	//	  Possible values: INT_DRDY_XL, INT_DRDY_G, INT1_BOOT (INT1 only), INT2_DRDY_TEMP (INT2 only)
	//	  INT_FTH, INT_OVR, INT_FSS5, INT_IG_XL (INT1 only), INT1_IG_G (INT1 only), INT2_INACT (INT2 only)
	//	- activeLow = Interrupt active configuration
	//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
	//	- pushPull =  Push-pull or open drain interrupt configuration
	//	  Can be either INT_PUSH_PULL or INT_OPEN_DRAIN
	void configInt(interrupt_select interupt, uint8_t generator,
				   h_lactive activeLow = INT_ACTIVE_LOW, pp_od pushPull = INT_PUSH_PULL);

	// configMagInt() -- Configure Magnetometer Interrupt Generator
	// Input:
	//	- generator = Interrupt axis/high-low events
	//	  Any OR'd combination of ZIEN, YIEN, XIEN
	//	- activeLow = Interrupt active configuration
	//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
	//	- latch: latch gyroscope interrupt request.
	void configMagInt(uint8_t generator, h_lactive activeLow, bool latch = true);

	// configMagThs() -- Configure the threshold of a gyroscope axis
	// Input:
	//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
	//	  Value is equivalent to raw magnetometer value.
	void configMagThs(uint16_t threshold);

	// getGyroIntSrc() -- Get contents of Gyroscope interrupt source register
	uint8_t getGyroIntSrc();

	// getGyroIntSrc() -- Get contents of accelerometer interrupt source register
	uint8_t getAccelIntSrc();

	// getGyroIntSrc() -- Get contents of magnetometer interrupt source register
	uint8_t getMagIntSrc();

	// getGyroIntSrc() -- Get status of inactivity interrupt
	uint8_t getInactivity();

	// sleepGyro() -- Sleep or wake the gyroscope
	// Input:
	//	- enable: True = sleep gyro. False = wake gyro.
	void sleepGyro(bool enable = true);

	// enableFIFO() - Enable or disable the FIFO
	// Input:
	//	- enable: true = enable, false = disable.
	void enableFIFO(bool enable = true);

	// setFIFO() - Configure FIFO mode and Threshold
	// Input:
	//	- fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
	//	  Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
	//	- fifoThs: FIFO threshold level setting
	//	  Any value from 0-0x1F is acceptable.
	void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs);

	// getFIFOSamples() - Get number of FIFO samples
	uint8_t getFIFOSamples();

	// Calibration procedure
	void openGyroCalibration();
	void openAccelCalibration();
	void openMagCalibration();

// Nothing is sacred in debugging mode
#if !defined(DEBUGGING)
protected:
#endif

	// gRes, aRes, and mRes store the current resolution for each sensor.
	// Units of these values would be DPS (or g's or Gs's) per ADC tick.
	// This value is calculated as (sensor scale) / (2^15).
	float gRes, aRes, mRes;

	uint8_t acceMMlOK=0; // bit 0..2 maxXYZ bit 3..5 minXYZ
	float maxAX = 1, maxAY=1, maxAZ=1, minAX=-1, minAY=-1, minAZ=-1; // Accel Slope
	float zeroAX1 =0,zeroAX2 =0,zeroAY1 =0,zeroAY2 =0,zeroAZ1 =0,zeroAZ2 =0;  //Accel Offset

	bool magnetOK=false;

	// initSettings() -- Sets up gyro, accel, and mag settings to default.
	// to set com interface and/or addresses see begin() and beginSPI().
	void initSettings();

	// initGyro() -- Sets up the gyroscope to begin reading.
	// This function steps through all five gyroscope control registers.
	// Upon exit, the following parameters will be set:
	//	- CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled.
	//		95 Hz ODR, 12.5 Hz cutoff frequency.
	//	- CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
	//		set to 7.2 Hz (depends on ODR).
	//	- CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
	//		active high). Data-ready output enabled on DRDY_G.
	//	- CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
	//		address. Scale set to 245 DPS. SPI mode set to 4-wire.
	//	- CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
	void initGyro();

	// initAccel() -- Sets up the accelerometer to begin reading.
	// This function steps through all accelerometer related control registers.
	// Upon exit these registers will be set as:
	//	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
	//	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
	//		all axes enabled.
	//	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
	//	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
	void initAccel();

	// initMag() -- Sets up the magnetometer to begin reading.
	// This function steps through all magnetometer-related control registers.
	// Upon exit these registers will be set as:
	//	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
	//	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
	//		requests don't latch. Temperature sensor disabled.
	//	- CTRL_REG6_XM = 0x00:  2 Gs scale.
	//	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
	//	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
	void initMag();

	// gReadByte() -- Reads a byte from a specified gyroscope register.
	// Input:
	// 	- subAddress = Register to be read from.
	// Output:
	// 	- An 8-bit value read from the requested address.
	uint8_t mReadByte(uint8_t subAddress);

	// gReadBytes() -- Reads a number of bytes -- beginning at an address
	// and incrementing from there -- from the gyroscope.
	// Input:
	// 	- subAddress = Register to be read from.
	// 	- * dest = A pointer to an array of uint8_t's. Values read will be
	//		stored in here on return.
	//	- count = The number of bytes to be read.
	// Output: No value is returned, but the `dest` array will store
	// 	the data read upon exit.
	uint8_t mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

	// gWriteByte() -- Write a byte to a register in the gyroscope.
	// Input:
	//	- subAddress = Register to be written to.
	//	- data = data to be written to the register.
	void mWriteByte(uint8_t subAddress, uint8_t data);

	// xmReadByte() -- Read a byte from a register in the accel/mag sensor
	// Input:
	//	- subAddress = Register to be read from.
	// Output:
	//	- An 8-bit value read from the requested register.
	uint8_t xgReadByte(uint8_t subAddress);

	// xmReadBytes() -- Reads a number of bytes -- beginning at an address
	// and incrementing from there -- from the accelerometer/magnetometer.
	// Input:
	// 	- subAddress = Register to be read from.
	// 	- * dest = A pointer to an array of uint8_t's. Values read will be
	//		stored in here on return.
	//	- count = The number of bytes to be read.
	// Output: No value is returned, but the `dest` array will store
	// 	the data read upon exit.
	uint8_t xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

	// xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
	// Input:
	//	- subAddress = Register to be written to.
	//	- data = data to be written to the register.
	void xgWriteByte(uint8_t subAddress, uint8_t data);

	// calcgRes() -- Calculate the resolution of the gyroscope.
	// This function will set the value of the gRes variable. gScale must
	// be set prior to calling this function.
	void calcgRes();

	// calcmRes() -- Calculate the resolution of the magnetometer.
	// This function will set the value of the mRes variable. mScale must
	// be set prior to calling this function.
	void calcmRes();

	// calcaRes() -- Calculate the resolution of the accelerometer.
	// This function will set the value of the aRes variable. aScale must
	// be set prior to calling this function.
	void calcaRes();

	//////////////////////
	// Helper Functions //
	//////////////////////
	void constrainScales();

	///////////////////
	// SPI Functions //
	///////////////////
	// initSPI() -- Initialize the SPI hardware.
	// This function will setup all SPI pins and related hardware.
	void initSPI();

	// SPIwriteByte() -- Write a byte out of SPI to a register in the device
	// Input:
	//	- csPin = The chip select pin of the slave device.
	//	- subAddress = The register to be written to.
	//	- data = Byte to be written to the register.
	void SPIwriteByte(uint8_t csPin, uint8_t subAddress, uint8_t data);

	// SPIreadByte() -- Read a single byte from a register over SPI.
	// Input:
	//	- csPin = The chip select pin of the slave device.
	//	- subAddress = The register to be read from.
	// Output:
	//	- The byte read from the requested address.
	uint8_t SPIreadByte(uint8_t csPin, uint8_t subAddress);

	// SPIreadBytes() -- Read a series of bytes, starting at a register via SPI
	// Input:
	//	- csPin = The chip select pin of a slave device.
	//	- subAddress = The register to begin reading.
	// 	- * dest = Pointer to an array where we'll store the readings.
	//	- count = Number of registers to be read.
	// Output: No value is returned by the function, but the registers read are
	// 		all stored in the *dest array given.
	uint8_t SPIreadBytes(uint8_t csPin, uint8_t subAddress,
							uint8_t * dest, uint8_t count);

	///////////////////
	// I2C Functions //
	///////////////////

	// I2CwriteByte() -- Write a byte out of I2C to a register in the device
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to be written to.
	//	- data = Byte to be written to the register.
	void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);

	// I2CreadByte() -- Read a single byte from a register over I2C.
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to be read from.
	// Output:
	//	- The byte read from the requested address.
	uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);

	// I2CreadBytes() -- Read a series of bytes, starting at a register via SPI
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to begin reading.
	// 	- * dest = Pointer to an array where we'll store the readings.
	//	- count = Number of registers to be read.
	// Output: No value is returned by the function, but the registers read are
	// 		all stored in the *dest array given.
	uint8_t I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);

	// Calibration methods
	// In calibration.cpp
	void calibrateGyroOffset(uint16_t sampleCount);
	void calibrateGyroSlope(uint8_t turnangle);
	void readGyroForAvg(int16_t sampleCount, float &averX, float &averY, float &averZ);

	void calibrateAccel(uint16_t sampleCount);
	void readAccelForAvg(uint16_t sampleCount, float &averX, float &averY, float &averZ);

	void calibrateMagneto(float earthMagneticStrength);
	void readMagnetoForAvg(uint16_t sampleCount, float &averX, float &averY, float &averZ);
};

#endif // SFE_LSM9DS1_H //
