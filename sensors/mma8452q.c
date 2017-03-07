/*
 * mma8452q.c
 *
 *  Created on: Feb 10, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Function definitions for initialization
 *			of the MMA8452Q accelerometer.
 */

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"

#include "utils/uartstdio.h"

#include "mma8452q.h"
#include "i2c_driver.h"

//*****************************************************************************
//
// This function will initialize the MMA8452Q Sensor.
// I2C_Base: Base address of the I2C module.
// AccelConfig: Range of the accelerometer.
// GyroRate: Update rate of the gyroscope.
// GyroAccuracy: degrees per second measurement rate of the gyroscope.
// MagRate: Update rate of the magnetometer.
//
// The device expects the following format:
// DEVICE_ADDRESS ACK REGISTER_ADDRESS ACK DATA STOP
//
// Note: The BMI160 can only be written to in one byte chunks.
// Also, the BMI160 boots up initially in suspend mode for the accel and
// gyro. In supsend mode, the device cannot receive burst writes.
//
//*****************************************************************************

#define DEBUG true

void InitMMA8452Q(uint32_t I2C_base, uint8_t AccelRange, uint8_t AccelRate)
{
	 uint8_t txBuffer[2];
	#if DEBUG
	uint8_t state[5] = {0, 0, 0, 0, 0};
#endif

	//
	// First, reset the device
	txBuffer[0] = MMA8452Q_CTRL_REG2;
	txBuffer[1] = MMA8452Q_RESET;
	I2CBurstWrite(I2C_base, MMA8452Q_DEVICE_ADDRESS, 2, txBuffer);

	SysCtlDelay(1000000);

#if DEBUG
	//
	// Check the value written.
    I2CRead(I2C_base, MMA8452Q_DEVICE_ADDRESS, MMA8452Q_CTRL_REG2, 1, state);
    UARTprintf("RESET: 0x%x\n\r", state[0]);
#endif

	//
	// Configure the device.
	txBuffer[0] = MMA8452Q_CTRL_REG1;
	txBuffer[1] = AccelRate | MMA8452Q_WAKE_MODE;
	I2CBurstWrite(I2C_base, MMA8452Q_DEVICE_ADDRESS, 2, txBuffer);

#if DEBUG
	//
	// Check the value written.
    I2CRead(I2C_base, MMA8452Q_DEVICE_ADDRESS, MMA8452Q_CTRL_REG1, 1, state);
    UARTprintf("CTRL_REG_1: 0x%x\n\r", state[0]);
#endif

	//
	// Enable the data ready interrupt.
	txBuffer[0] = MMA8452Q_CTRL_REG4;
	txBuffer[1] = MMA8452Q_INT_DDRY_EN;
	I2CBurstWrite(I2C_base, MMA8452Q_DEVICE_ADDRESS, 2, txBuffer);

#if DEBUG
	//
	// Check the value written.
    I2CRead(I2C_base, MMA8452Q_DEVICE_ADDRESS, MMA8452Q_CTRL_REG4, 1, state);
    UARTprintf("CTRL_REG_4: 0x%x\n\r", state[0]);
#endif

	//
	// Configure the interrupt as INT1.
	txBuffer[0] = MMA8452Q_CTRL_REG5;
	txBuffer[1] = MMA8452Q_INT_DDRY_INT1;
	I2CBurstWrite(I2C_base, MMA8452Q_DEVICE_ADDRESS, 2, txBuffer);

#if DEBUG
	//
	// Check the value written.
    I2CRead(I2C_base, MMA8452Q_DEVICE_ADDRESS, MMA8452Q_CTRL_REG5, 1, state);
    UARTprintf("CTRL_REG_5: 0x%x\n\r", state[0]);
#endif

	//
	// Configure the range of the device.
	txBuffer[0] = MMA8452Q_RANGE;
	txBuffer[1] = MMA8452Q_RATE_2G;
	I2CBurstWrite(I2C_base, MMA8452Q_DEVICE_ADDRESS, 2, txBuffer);

#if DEBUG
	//
	// Check the value written.
    I2CRead(I2C_base, MMA8452Q_DEVICE_ADDRESS, MMA8452Q_RATE_2G, 1, state);
    UARTprintf("CTRL_REG_5: 0x%x\n\r", state[0]);
#endif
}
