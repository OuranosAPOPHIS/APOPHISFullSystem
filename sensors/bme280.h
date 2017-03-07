/*
 * bme280.h
 *
 *  Created on: Feb 5, 2017
 *      Author: Brandon Klefman
 *      Purpose: Header file for the BME280
 *      environmental sensor. This file contains
 *      the mapping of all the registers
 *      in the BME280.
 */

#ifndef BME280_H_
#define BME280_H_

#include <stdint.h>

//
// Maximum value in a 20 bit variable.
#define BME280_MAX_PT_VALUE 524288
#define BME280_MAX_HUM_VALUE 32768

//
// Device address as configured by TI.
#define BME280_DEVICE_ADDRESS 0x77

//
// Register map.
#define BME280_CHIP_ID 0xD0
#define BME280_RESET_REG 0xE0
#define BME280_CTRL_HUM 0xF2
#define BME280_STATUS 0xF3
#define BME280_CTRL_MEAS 0xF4
#define BME280_CONFIG 0xF5
#define BME280_PRES_RAW 0xF7 // 3 bytes
#define BME280_TEMP_RAW 0xFA // 3 bytes
#define BME280_HUM_RAW 0xFD // 2 bytes
#define BME280_COMP_VALUES 0x88 // 18 bytes, read up to 3 bytes for temp, 12 bytes for temp and pres.

/*
 * Register: Reset
 * The reset value that must be written
 * to the reset register to intitiate
 * a reset. All other values are ignored.
 */
#define BME280_RESET 0xB6

/*
 * Register: Ctrl Hum
 * The configuration settings of the
 * oversampling for the humidity sensor.
 */
#define BME280_HUM_OS_SKIP 0x00
#define BME280_HUM_OS_X1 0x01
#define BME280_HUM_OS_X2 0x02
#define BME280_HUM_OS_X4 0x03
#define BME280_HUM_OS_X8 0x04
#define BME280_HUM_OS_X16 0x05

/*
 * Register: Status
 * Status of the device.
 */
#define BME280_MEASURING 0x08
#define BME280_IM_UPDATE 0x01

/*
 * Register: Ctrl Measure
 * Sets the data aquisition settings
 * for the pressure and temperature.
 * This must be written to after a
 * write to ctrl_hum.
 */
#define BME280_PRES_OS_SKIP 0x00
#define BME280_PRES_OS_X1 0x04
#define BME280_PRES_OS_X2 0x08
#define BME280_PRES_OS_X4 0x0C
#define BME280_PRES_OS_X8 0x10
#define BME280_PRES_OS_X16 0x14

#define BME280_TEMP_OS_SKIP 0x00
#define BME280_TEMP_OS_X1 0x20
#define BME280_TEMP_OS_X2 0x40
#define BME280_TEMP_OS_X4 0x60
#define BME280_TEMP_OS_X8 0x80
#define BME280_TEMP_OS_X16 0xA0

#define BME280_SLEEP_MODE 0x00
#define BME280_FORCE_MODE 0x01
#define BME280_NORMAL_MODE 0x03

/*
 * Register: Config
 * Sets the rate, filter and
 * interface options of the
 * device.
 * Note: Writes to this
 * register may be ignored
 * in normal mode.
 * In sleep mode, writes
 * are not ignored.
 *
 * Write to the config register
 * before setting the operational
 * mode.
 */
#define BME280_STNDBY_0_5 0x00
#define BME280_STNDBY_62 0x20
#define BME280_STNDBY_125 0x40
#define BME280_STNDBY_250_ 0x60
#define BME280_STNDBY_500 0x80
#define BME280_STNDBY_1000 0xA0
#define BME280_STNDBY_10 0xC0
#define BME280_STNDBY_20 0xE0

#define BME280_FLTR_OFF 0x00
#define BME280_FLTR_2 0x02
#define BME280_FLTR_4 0x04
#define BME280_FLTR_8 0x06
#define BME280_FLTR_16 0x08

#define BME280_SPI_EN 0x01

/*
 * Function Prototypes
 */
void InitBME280(uint32_t I2C_base, int8_t *offsetValues);
void GetBME280RawData(uint32_t I2C_base, uint8_t *rxBuffer);
int32_t BME280_compensate_T_int32(int32_t adc_T, int32_t *t_fine, int8_t *offsetValues, uint8_t *dig_P2);
uint32_t BME280_compensate_P_int64(int32_t adc_P, int32_t *t_fine, int8_t *offsetValues, uint8_t *dig_P1);

#endif /* BME280_H_ */
