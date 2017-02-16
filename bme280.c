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

#include "utils/uartstdio.h"

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
void InitBME280(uint32_t I2C_base, int8_t *offsetValues)
{
    uint8_t txBuffer[2] = {0};
    uint8_t state[2];

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
    // Check the write.
    I2CRead(I2C_base, BME280_DEVICE_ADDRESS, BME280_CONFIG, 1, state);
    UARTprintf("CONFIG: 0x%x\n\r", state[0]);

    //
    // Configure the oversampling of the humidity sensor.
    txBuffer[0] = BME280_CTRL_HUM;
    txBuffer[1] = BME280_HUM_OS_X1;
    I2CBurstWrite(I2C_base, BME280_DEVICE_ADDRESS, 2, txBuffer);

    //
    // Check the write.
    I2CRead(I2C_base, BME280_DEVICE_ADDRESS, BME280_CTRL_HUM, 1, state);
    UARTprintf("CTRL_HUM: 0x%x\n\r", state[0]);

    //
    // Configure the oversampling of the sensors.
    txBuffer[0] = BME280_CTRL_MEAS;
    txBuffer[1] = BME280_PRES_OS_X16 | BME280_TEMP_OS_X2 | BME280_NORMAL_MODE;
    I2CBurstWrite(I2C_base, BME280_DEVICE_ADDRESS, 2, txBuffer);

    //
    // Check the write.
    I2CRead(I2C_base, BME280_DEVICE_ADDRESS, BME280_CTRL_MEAS, 1, state);
    UARTprintf("CTRL_MEAS: 0x%x\n\r", state[0]);

    //
    // Get the offset values from the sensor.
    I2CRead(I2C_base, BME280_DEVICE_ADDRESS, BME280_COMP_VALUES, 12, offsetValues);
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

//
// Provided by BME280
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t BME280_compensate_T_int32(int32_t adc_T, int32_t *t_fine, int8_t *offsetValues, uint8_t *dig_T1)
{
    int32_t var1, var2, T;
    int8_t dig_T2 = offsetValues[0];
    int8_t dig_T3 = offsetValues[1];

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = ((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3) >> 14;

    *t_fine = var1 + var2;

    T = (*t_fine * 5 + 128) >> 8;

    return T;
}

//
// Provided by BME280
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280_compensate_P_int64(int32_t adc_P, int32_t *p_t_fine, int8_t *offsetValues, uint8_t *dig_P1)
{
    int64_t var1, var2, p;
    int32_t t_fine = *p_t_fine;
    int8_t dig_P2 = offsetValues[4];
    int8_t dig_P3 = offsetValues[5];
    int8_t dig_P4 = offsetValues[6];
    int8_t dig_P5 = offsetValues[7];
    int8_t dig_P6 = offsetValues[8];
    int8_t dig_P7 = offsetValues[9];
    int8_t dig_P8 = offsetValues[10];
    int8_t dig_P9 = offsetValues[11];

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
    var2 = var2 + (((int64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;

    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }

    p = 1048576 - adc_P;

    p = (((p<<31)-var2)*3125)/var1;

    var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);

    return (uint32_t)p;
}

