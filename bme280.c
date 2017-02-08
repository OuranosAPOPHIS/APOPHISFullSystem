/*
 * bme280.c
 *
 *  Created on: Feb 5, 2017
 *      Author: Brandon Klefman
 *      Purpose: Initialization function
 *      and other functions for the BME280
 *      environmental sensor.
 */

#include <stdint.h>

#include "i2c_driver.h"
#include "bme280.h"


/*
 * Initialization function for the BME280.
 *
 *
 * Note: IMU must be enabled, as the BME280
 * and IMU share the same I2C bus.
 *
 * I2C_base: the base address of the I2C module.
 *
 * Configuration is currently set to the recommended
 * for indoor navigation. Pressure Oversampling: 16x,
 * Temperature Oversampling: 2x, Humidity oversampling 1x,
 * standby time 0.5ms, and IIR Filter coefficient 16.
 * At this setting, data is output at 25 Hz.
 */
void InitBME280(uint32_t I2C_base)
{
    uint8_t txBuffer[2] = {0};

    //
    // Reset the device.
    txBuffer[0] = BME280_RESET_REG;
    txBuffer[1] = BME280_RESET;
    I2CBurstWrite(I2C_base, BME280_DEVICE_ADDRESS, 2, txBuffer);

    //
    // Configure the device.
    txBuffer[0] = BME280_CONFIG;
    txBuffer[1] = BME280_STNDBY_0_5 | BME280_FLTR_16;
    I2CBurstWrite(I2C_base, BME280_DEVICE_ADDRESS, 2, txBuffer);

    //
    // Configure the oversampling of the humidity sensor.
    txBuffer[0] = BME280_CTRL_HUM;
    txBuffer[1] = BME280_HUM_OS_X1;
    I2CBurstWrite(I2C_base, BME280_DEVICE_ADDRESS, 2, txBuffer);

    //
    // Configure the oversampling of the sensors.
    txBuffer[0] = BME280_CTRL_MEAS;
    txBuffer[1] = BME280_PRES_OS_X16 | BME280_TEMP_OS_X2 | BME280_NORMAL_MODE;
    I2CBurstWrite(I2C_base, BME280_DEVICE_ADDRESS, 2, txBuffer);
}

/*
 * Get the data from all of the sensors on the BME 280.
 *
 * I2C_base: the base address for the I2C module.
 * *rxBuffer: pointer to the pressure, temperature and humidity data.
 */
void GetBME280RawData(uint32_t I2Cbase, uint8_t *rxBuffer)
{
    //
    // Perform the I2C read. The pressure and temperature
    // are stored as 20 bit unsigned ints and the humidity
    // data is stored as a 16 bit int.
    I2CRead(I2Cbase, BME280_DEVICE_ADDRESS, BME280_PRES_RAW, 7, rxBuffer);
}


