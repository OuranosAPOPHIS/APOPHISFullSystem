/*
 * bmi160.c
 *
 *  Created on: Jan 31, 2017
 *      Author: Brandon Klefman
 *      Purpose: Contains the initialization,
 *      and operational functions for the
 *      BMI 160 sensor.
 */

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"

#include "utils/uartstdio.h"

#include "bmi160.h"
#include "i2c_driver.h"

#define DEBUG true

//*****************************************************************************
//
// This function will initialize the BMI160 Sensor by configuring the
// accelerometer, gyroscope and magnetometer of the device with the given values.
//
// I2C_BASE: Base address of the I2C module.
// AccelRate: Update rate of the accelerometer.
// AccelAccuracy: g accuracy of the accelerometer.
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
void InitBMI160(uint32_t I2C_base, uint8_t AccelRate, uint8_t AccelAccuracy, uint8_t GyroRate,
                uint8_t GyroAccuracy, uint8_t MagRate, uint8_t *offsetValues, uint32_t sysClockSpeed)
{
    uint8_t txBuffer[2];
#if DEBUG
    uint8_t state[5] = {0, 0, 0, 0, 0};
#endif

    //
    // First initiate a softreset to boot the device fresh.
    txBuffer[0] = BMI160_CMD;
    txBuffer[1] = BMI160_SOFT_RESET;

    //
    // First Reset the device.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

    //
    // Wait 3.8 ms at least, before booting the accel after the reset.
    SysCtlDelay(sysClockSpeed / 1000 * 10);

    //
    // Next, boot up the accel in normal mode.
    txBuffer[0] = BMI160_CMD;
    txBuffer[1] = BMI160_ACC_NORMAL_MODE_SET;
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

    //
    // 450us delay is required after each send in suspend mode.
    SysCtlDelay(sysClockSpeed / 1000 / 2);

    //
    // Next, boot up the gyro in normal mode.
    txBuffer[0] = BMI160_CMD;
    txBuffer[1] = BMI160_GYR_NORMAL_MODE_SET;
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

    //
    // 80 ms delay after boot up of gyro
    SysCtlDelay(sysClockSpeed / 1000 * 100);

    //
    // Next, boot up mag in normal mode.
    txBuffer[0] = BMI160_CMD;
    txBuffer[1] = BMI160_MAG_NORMAL_MODE_SET;
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

    //
    // 0.5 ms delay after boot up of mag
    SysCtlDelay(sysClockSpeed / 1000 * 2);

#if DEBUG
    //
    // Check the PMU configuration.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_PMU_STATUS, 1, state);
    UARTprintf("PMU Status: 0x%x\n\r", state[0]);
#endif

    //
    // Set the configuration parameters for the accel rate.
    txBuffer[0] = BMI160_ACC_CONFIG;
    txBuffer[1] = AccelRate;

    //
    // Configure the accelerometer rate.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

#if DEBUG
    //
    // Check the accel rate configuration.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_ACC_CONFIG, 1, state);
    UARTprintf("Accel Rate: 0x%x\n\r", state[0]);
#endif

    //
    // Set the configuration parameters of the accel range.
    txBuffer[0] = BMI160_ACC_RANGE;
    txBuffer[1] = AccelAccuracy;

    //
    // Configure the accelerometer range.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

#if DEBUG
    //
    // Check the accel g' configuration.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_ACC_RANGE, 1, state);
    UARTprintf("Accel G Setting: 0x%x\n\r", state[0]);
#endif

    //
    // Set the configuration parameters of the gyro rate.
    txBuffer[0] = BMI160_GYR_CONFIG;
    txBuffer[1] = GyroRate;

    //
    // Configure the gyro rate.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

#if DEBUG
    //
    // Check the gyro rate configuration.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_GYR_CONFIG, 1, state);
    UARTprintf("Gyro Rate: 0x%x\n\r", state[0]);
#endif

    //
    // Set the configuration parameters for the gyro range.
    txBuffer[0] = BMI160_GYR_RANGE;
    txBuffer[1] = GyroAccuracy;

    //
    // Configure the gyro range.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

#if DEBUG
    //
    // Check the gyro accuracy configuration.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_GYR_RANGE, 1, state);
    UARTprintf("Gyro Accuracy: 0x%x\n\r", state[0]);

    //
    // Check the Fast Offset Compensation values.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_FOC_CONFIG, 1, state);
    UARTprintf("(FOC CONF): 0x%x\n\r", state[0]);
#endif

    //
    // Initialize the magnetometer.
    // Set up the mag address.
    txBuffer[0] = BMI160_MAG_IF;
    txBuffer[1] = BMI160_MAG_ADDRESS << 1;
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

    //
    // Set up direct writes.
    txBuffer[0] = BMI160_MAG_IF + 0x01;
    txBuffer[1] = BMI160_MAG_DIRECT_ENABLE;
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

    SysCtlDelay(sysClockSpeed / 500);

    //
    // Reset the magnetometer.
    txBuffer[0] = BMI160_MAG_IF + 0x04;
    txBuffer[1] = BMM150_SOFT_RESET | BMM150_SLEEP_STARTUP;
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

    SysCtlDelay(sysClockSpeed / 500);

    txBuffer[0] = BMI160_MAG_IF + 0x03;
    txBuffer[1] = BMM150_RESET_REG;
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

    SysCtlDelay(sysClockSpeed / 500);

    //
    // Configure the magnetometer for 25 Hz operation and Normal mode.
    txBuffer[0] = BMI160_MAG_IF + 0x04;
    txBuffer[1] = BMM150_DATA_RATE_30_HZ | BMM150_NORMAL_MODE;
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

    SysCtlDelay(sysClockSpeed / 500);

    txBuffer[0] = BMI160_MAG_IF + 0x03;
    txBuffer[1] = BMM150_CONFIG_REG;
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

    SysCtlDelay(sysClockSpeed / 500);

    //
    // Set up the interface with the magnetometer.
    txBuffer[0] = BMI160_MAG_IF + 0x01;
    txBuffer[1] = BMI160_MAG_BURST_READ;
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

    txBuffer[0] = BMI160_MAG_IF + 0x02;
    txBuffer[1] = BMM150_DATA_REG;
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

    SysCtlDelay(sysClockSpeed / 500);

#if DEBUG
    //
    // Check the configuration.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_MAG_IF, 5, state);
    UARTprintf("(MAG IF): 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n\r", state[0], state[1], state[2], state[3], state[4]);
#endif

    //
    // Set up the device interface as I2C primary and Mag on.
    txBuffer[0] = BMI160_IF_CONFIG;
    txBuffer[1] = BMI160_AUTO_MAG_ON;

    //
    // Write the setting to the device.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

#if DEBUG
    //
    // Check the configuration.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_IF_CONFIG, 1, state);
    UARTprintf("(IF CONF): 0x%x\n\r", state[0]);
#endif

    //
    // Set the configuration parameters for the mag rate.
    txBuffer[0] = BMI160_MAG_CONFIG;
    txBuffer[1] = MagRate;

    //
    // Configure the mag rate.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);

#if DEBUG
    //
    // Check the configuration.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_MAG_CONFIG, 1, state);
    UARTprintf("Mag Rate: 0x%x\n\r", state[0]);
#endif

    //
    // Enable INT1 on the device as the data ready pin.
    txBuffer[0] = BMI160_INT_ENABLE;
    txBuffer[1] = 0x00; // 0x00 to register 0x50

    //
    // Write these values to the device.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 4, txBuffer);
    SysCtlDelay(sysClockSpeed / 500);

    //
    // Enable INT1 on the device as the data ready pin.
    txBuffer[0] = BMI160_INT_ENABLE + 0x01;
    txBuffer[1] = BMI160_INT_EN_DDRDY; // 0x10 to register 0x51

    //
    // Write these values to the device.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);
    SysCtlDelay(sysClockSpeed / 500);

    //
    // Enable INT1 on the device as the data ready pin.
    txBuffer[0] = BMI160_INT_ENABLE + 0x02;
    txBuffer[1] = 0x00; // 0x00 to register 0x52

    //
    // Write these values to the device.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);
    SysCtlDelay(sysClockSpeed / 500);

#if DEBUG
    //
    // Check the interrupt configuration.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_INT_ENABLE, 3, state);
    UARTprintf("Interrupt Configuration:\n\r(INT_EN): 0x%x, 0x%x, 0x%x\n\r", state[0], state[1], state[2]);
#endif

    //
    // Configure INT1.
    txBuffer[0] = BMI160_INT_MAP;
    txBuffer[1] = 0x00; // 0x00 to register 0x55

    //
    // Write these values to the device.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);
    SysCtlDelay(sysClockSpeed / 500);

    //
    // Configure INT1.
    txBuffer[0] = BMI160_INT_MAP + 0x01;
    txBuffer[1] = BMI160_INT1_DDRY; // 0x80 to register 0x56

    //
    // Write these values to the device.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);
    SysCtlDelay(sysClockSpeed / 500);

    //
    // Configure INT1.
    txBuffer[0] = BMI160_INT_MAP + 0x02;
    txBuffer[1] = 0x00; // 0x00 to register 0x57

    //
    // Write these values to the device.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);
    SysCtlDelay(sysClockSpeed / 500);

#if DEBUG
    //
    // Check the interrupt configuration.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_INT_MAP, 3, state);
    UARTprintf("(INT_MAP): 0x%x, 0x%x, 0x%x\n\r", state[0], state[1], state[2]);
#endif

    //
    // Configure INT1.
    txBuffer[0] = BMI160_INT_OUT_CTRL;
    txBuffer[1] = BMI160_INT1_OUT_EN; // 0x08 to register 0x53

    //
    // Write these values to the device.
    I2CBurstWrite(I2C_base, BMI160_ADDRESS, 2, txBuffer);
    SysCtlDelay(sysClockSpeed / 500);

    //
    // Now get the offset values from the device for proper calibration.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_OFFSET, BMI160_OFFSET_SIZE, offsetValues);

#if DEBUG
    //
    // Check the interrupt configuration
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_INT_OUT_CTRL, 1, state);
    UARTprintf("(INT_OUT_CTRL): 0x%x\n\r", state[0]);

    //
    // Check the error register for errors.
    I2CRead(I2C_base, BMI160_ADDRESS, BMI160_ERR_REG, 1, state);
    UARTprintf("(ERR_REG): 0x%x\n\r", state[0]);
#endif
}
