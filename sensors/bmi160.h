/*
 * bm160.h
 *
 *  Created on: Jan 31, 2017
 *      Author: Brandon Klefman
 *      Purpose: Define the registers addresses
 *      for the bmi160 6-axis sensor unit.
 */

#ifndef BMI160_H_
#define BMI160_H_

//
// Useful definitions for converting from int16_t to
// float for computations.
#define BMI160_LSBG_2G 16384
#define BMI160_LSBG_4G 8192
#define BMI160_LSBG_8G 4096
#define BMI160_LSBG_16G 2048

//
// Define the device address.
// Default address is 0x68, but TI
// set this up as 0x69.
#define BMI160_ADDRESS 0x69
#define BMI160_MAG_ADDRESS 0x13

/*
 * Define the relevant register addresses.
 */
#define BMI160_CHIPID 0x00
#define BMI160_ERR_REG 0x02
#define BMI160_PMU_STATUS 0x03
#define BMI160_DATA 0x04
#define BMI160_DATA_SIZE 20 // bytes
#define BMI160_SENSORTIME 0x18 // 3 bytes
#define BMI160_STATUS 0x1B
#define BMI160_INT_STATUS 0x1C // 3 bytes
#define BMI160_ACC_CONFIG 0x40
#define BMI160_ACC_RANGE 0x41
#define BMI160_GYR_CONFIG 0x42
#define BMI160_GYR_RANGE 0x43
#define BMI160_MAG_CONFIG 0x44
#define BMI160_MAG_IF 0x4B // 5 bytes
#define BMI160_INT_ENABLE 0x50 // 3 bytes
#define BMI160_INT_OUT_CTRL 0x53
#define BMI160_INT_LATCH 0x54
#define BMI160_INT_MAP 0x55 // 3 bytes
#define BMI160_INT_DATA 0x58 // 2 bytes
#define BMI160_INT_FLAT_CONFIG 0x67 // 2 bytes
#define BMI160_FOC_CONFIG 0x69
#define BMI160_IF_CONFIG 0x6B
#define BMI160_OFFSET 0x71 // 7 bytes
#define BMI160_OFFSET_SIZE 7
#define BMI160_CMD 0x7E

/*
 * Register: ERR_REG
 * Possible values to read from ERR_REG.
 */
#define BMI160_NO_ERR 0x0
#define BMI160_ERR1 0x1
#define BMI160_ERR2 0x2

/*
 * Register: PMU_STATUS
 * Possible values in register PMU_STATUS.
 * This register shows the current power
 * mode of the sensor.
 */
#define BMI160_SUSP_MODE 0x80
#define BMI160_ACC_NORMAL_MODE 0x90
#define BMI160_ACC_LOW_POWER 0xA0
#define BMI160_GYR_NORMAL_MODE 0x84
#define BMI160_GYR_FAST_START 0x8C
#define BMI160_MAG_NORMAL_MODE 0x81
#define BMI160_MAG_LOW_POWER 0x82

/*
 * Register: DATA
 * Possible values in data registers.
 * Each x,y,z data piece takes up
 * one byte of data.
 */
#define BMI160_MAG_X 0x04
#define BMI160_MAG_Y 0x06
#define BMI160_MAG_Z 0x08
#define BMI160_MAG_RHALL 0x0A
#define BMI160_GYRO_X 0x0C
#define BMI160_GYRO_Y 0x0E
#define BMI160_GYRO_Z 0x10
#define BMI160_ACC_X 0x12
#define BMI160_ACC_Y 0x14
#define BMI160_ACC_Z 0x16

/*
 * Possible data in the SENSORTIME Register.
 * Right now, not going to be used.
 */

/*
 * Register: STATUS
 * Possible values in the STATUS register.
 * This is used when an interrupt is called
 * to find out which data is fresh.
 */
#define BMI160_ACC_RDY 0x80
#define BMI160_GYR_RDY 0x40
#define BMI160_MAG_RDY 0x20
#define BMI160_NVM_RDY 0x10

/*
 * Register: INT_STATUS
 * Possible values in the INT_STATUS register.
 * Many of these will not be used.
 * Dump first byte, check second byte for
 * data ready pin. Dump third byte.
 */
#define BMI160_INT_DATA_RDY 0x10

/*
 * Register: ACC_CONF
 * Possible values in the ACC_CONF register.
 * This sets the output data rate, bandwidth
 * and read mode of the accelerometer.
 */
#define BMI160_ACC_0_8_HZ 0x21
#define BMI160_ACC_1_5_HZ 0x22
#define BMI160_ACC_3_1_HZ 0x23
#define BMI160_ACC_6_3_HZ 0x24
#define BMI160_ACC_12_5_HZ 0x25
#define BMI160_ACC_25_HZ 0x26
#define BMI160_ACC_50_HZ 0x27
#define BMI160_ACC_100_HZ 0x28
#define BMI160_ACC_200_HZ 0x29
#define BMI160_ACC_400_HZ 0x2A
#define BMI160_ACC_800_HZ 0x2B
#define BMI160_ACC_1600_HZ 0x2C

/*
 * Possible values in the ACC_RANGE register.
 * Allows for selection of accelerometer g
 * range.
 */
#define BMI160_ACC_RANGE_2G 0x03
#define BMI160_ACC_RANGE_4G 0x05
#define BMI160_ACC_RANGE_8G 0x08
#define BMI160_ACC_RANGE_16G 0x0C

/*
 * Register: GYR_CONF
 * Possible values for the GYR_CONF register.
 * This sets the output data rate, the bandwidth
 * and the read mode of the gyroscope.
 * Note: The gyroscope cannot operate at a
 * rate slower than 25 HZ.
 * Normal mode is defined as 0b010 for gyr_bwp
 * value.
 */
#define BMI160_GYR_25_HZ 0x26
#define BMI160_GYR_50_HZ 0x27
#define BMI160_GYR_100_HZ 0x28
#define BMI160_GYR_200_HZ 0x29
#define BMI160_GYR_400_HZ 0x2A
#define BMI160_GYR_800_HZ 0x2B
#define BMI160_GYR_1600_HZ 0x2C
#define BMI160_GYR_3200_HZ 0x2D

/*
 * Register: GYR_RANGE
 * Possible values for the GYR_RANGE register.
 * This sets the angular rate measurement range
 * in degrees per second.
 */
#define BMI160_GYR_RATE_2000 0x00
#define BMI160_GYR_RATE_1000 0x01
#define BMI160_GYR_RATE_500 0x02
#define BMI160_GYR_RATE_250 0x03
#define BMI160_GYR_RATE_125 0x04

/*
 * Register: MAG_CONF
 * Possible values for the MAG_CONF register.
 * This register sets the output data rate of
 * the magnetometer.
 */
#define BMI160_MAG_1_HZ 0x01
#define BMI160_MAG_2_HZ 0x02
#define BMI160_MAG_4_HZ 0x03
#define BMI160_MAG_8_HZ 0x04
#define BMI160_MAG_16_HZ 0x05
#define BMI160_MAG_31_HZ 0x06
#define BMI160_MAG_63_HZ 0x07
#define BMI160_MAG_125_HZ 0x08
#define BMI160_MAG_250_HZ 0x09
#define BMI160_MAG_500_HZ 0x0A
#define BMI160_MAG_1000_HZ 0x0B

/*
 * Register: MAG_IF
 *
 */
#define BMI160_MAG_DIRECT_ENABLE 0x80
#define BMI160_MAG_BURST_READ 0x03

#define BMM150_DATA_REG 0x42
#define BMM150_RESET_REG 0x4B
#define BMM150_CONFIG_REG 0x4C

#define BMM150_SOFT_RESET 0x82
#define BMM150_SUSPEND_STARTUP 0x00
#define BMM150_SLEEP_STARTUP 0x01

#define BMM150_DATA_RATE_10_HZ 0x00
#define BMM150_DATA_RATE_8_HZ 0x18
#define BMM150_DATA_RATE_25_HZ 0x30
#define BMM150_DATA_RATE_30_HZ 0x38
#define BMM150_NORMAL_MODE 0x00
#define BMM150_SLEEP_MODE 0x03

/*
 * Register: INT_EN
 * Possible values for the INT_EN register.
 * This register controls which interrupts
 * will fire. This register has a total of
 * 3 bytes. For the data ready bit, the
 * data must be written to register 0x51.
 */
//
// Values to be written to register 0x50.
#define BMI160_INT_EN_FLAT 0x80
#define BMI160_INT_EN_ORIENT 0x40
#define BMI160_INT_EN_STAP 0x20
#define BMI160_INT_EN_DTAP 0x10
#define BMI160_INT_EN_ANY_Z 0x04
#define BMI160_INT_EN_ANY_Y 0x02
#define BMI160_INT_EN_ANY_X 0x01

//
// Values to be written to register 0x51.
#define BMI160_INT_EN_FIFOWATER 0x40
#define BMI160_INT_EN_FIFOFULL 0x20
#define BMI160_INT_EN_DDRDY 0x10
#define BMI160_INT_EN_LOWG 0x08
#define BMI160_INT_EN_HIGHG_Z 0x04
#define BMI160_INT_EN_HIGHG_Y 0x02
#define BMI160_INT_EN_HIGHG_X 0x01

//
// Values to be written to register 0x52.
#define BMI160_INT_EN_STEP 0x08
#define BMI160_INT_EN_SLOWMO_Z 0x04
#define BMI160_INT_EN_SLOWMO_Y 0x02
#define BMI160_INT_EN_SLOWMO_X 0x01

/*
 * Register: INT_OUT_CTRL
 * Possible values for the INT_OUT_CTRL
 * register. This register contains the
 * behavioral configuration of the interrupt
 * pins.
 */
#define BMI160_INT2_OUT_EN 0x80
#define BMI160_INT2_OD 0x40
#define BMI160_INT2_LVL 0x20
#define BMI160_INT2_EDGE_CTRL 0x10
#define BMI160_INT1_OUT_EN 0x08
#define BMI160_INT1_OD 0x04
#define BMI160_INT1_LVL 0x02
#define BMI160_INT1_EDGE_CTRL 0x01

/*
 * Register: INT_LATCH
 * Possible values for the INT_LATCH
 * register. This register contains the
 * raw interrupt reset bit and interrupt
 * mode selection.
 */
#define BMI160_NON_LATCHED 0x00
#define BMI160_LATCHED 0x0F

/*
 * Register: INT_MAP
 * Possible values for the INT_MAP register.
 * This register controls which interrupt
 * signals are mapped to the INT1 and INT2
 * pins. This register takes up 3 bytes.
 */
//
// Values for register 0x55. Interrupts to
// be mapped to INT1.
#define BMI160_INT1_FLAT 0x80
#define BMI160_INT1_ORIENT 0x40
#define BMI160_INT1_STAP 0x20
#define BMI160_INT1_DTAP 0x10
#define BMI160_INT1_NOMO 0x08
#define BMI160_INT1_ANYMOT 0x04
#define BMI160_INT1_HIGHG 0x02
#define BMI160_INT1_LOWG 0x01

//
// Values for register 0x56. Interrupts to
// be mapped to INT1 and INT2.
#define BMI160_INT1_DDRY 0x80
#define BMI160_INT1_FIFOWATER 0x40
#define BMI160_INT1_FIFOFULL 0x20
#define BMI160_INT1_PMUTRIG 0x10
#define BMI160_INT2_DDRY 0x08
#define BMI160_INT2_FIFOWATER 0x04
#define BMI160_INT2_FIFOFULL 0x02
#define BMI160_INT2_MPUTRIG 0x01

//
// Values for register 0x57. Interrupts to
// be mapped to INT2.
#define BMI160_INT2_FLAT 0x80
#define BMI160_INT2_ORIENT 0x40
#define BMI160_INT2_STAP 0x20
#define BMI160_INT2_DTAP 0x10
#define BMI160_INT2_NOMO 0x08
#define BMI160_INT2_ANYMOT 0x04
#define BMI160_INT2_HIGHG 0x02
#define BMI160_INT2_LOWG 0x01

/*
 * Register: INT_DATA
 * Possible values for INT_DATA register.
 * This register contains data source
 * definitions for the two interrupt groups.
 * Not used at this time.
 */

/*
 * Register: INT_LOW_HIGH
 * Contains the configuration for the
 * low g interrupt.
 * Not used at this time.
 */

/*
 * Register: INT_MOTION
 * Contains the configuration for the anymotion
 * or nomotion interrupts.
 * Not used at this time.
 */

/*
 * Register: INT_TAP
 * Contains the configuration for the tap
 * interrupts.
 * Not used at this time.
 */

/*
 * Register: INT_ORIENT
 * Contains the configuration for the
 * orientation interrupt.
 * Not used at this time.
 */

/*
 * Register: INT_FLAT
 * Contains the configuration for the flat
 * interrupt.
 * Not used at this time.
 */

/*
 * Register: FOC_CONF
 * Contains the configuration settings for
 * the fast offset compensation for the
 * accelerometer and the gyroscope.
 */
#define BMI160_GYR_FOC_ON 0x40
#define BMI160_ACC_FOC_X_1G 0x10
#define BMI160_ACC_FOC_X_NEG_1G 0x20
#define BMI160_ACC_FOC_X_0G 0x30
#define BMI160_ACC_FOC_Y_1G 0x04
#define BMI160_ACC_FOC_Y_NEG_1G 0x08
#define BMI160_ACC_FOC_Y_0G 0x0C
#define BMI160_ACC_FOC_Z_1G 0x01
#define BMI160_ACC_FOC_Z_NEG_1G 0x02
#define BMI160_ACC_FOC_Z_0G 0x03


/*
 * Register: CONF
 * Contains the configuration of the sensor.
 * Not used at this time.
 */

/*
 * Register: IF_CONF
 * Contains the settings for the digital
 * interface.
 */
#define BMI160_PRIMARY_AUTO 0x00
#define BMI160_I2C_OIS 0x10
#define BMI160_AUTO_MAG_ON 0x20

/*
 * Register: PMU_TRIGGER
 * Used to set the trigger conditions to
 * change the gyro power modes.
 * Not used at this time.
 */

/*
 * Register: SELF_TEST
 * Contains settings for the sensor self-test
 * configuration and trigger.
 * Not used at this time.
 */

/*
 * Register: NV_CONF
 * Contains the settings for the digital
 * interface.
 * Not used at this time.
 */

/*
 * Register: OFFSET
 * Contains the offset compensation values
 * for the accelerometer and gyroscope.
 * This program will not be writing to this
 * register.
 */

/*
 * Register: STEP_CNT
 * Contains the number of steps.
 * Not used at this time.
 */

/*
 * Register: STEP_CONF
 * Contains the configuraton of the step
 * detector.
 * Not used at this time.
 */

/*
 * Register: CMD
 * Command register triggers operations like
 * softreset, NVM programming, etc.
 * Note: If a soft reset is issued to the
 * device, it must be completely reconfigured.
 */
#define BMI160_SOFT_RESET 0xB1
#define BMI160_ACC_NORMAL_MODE_SET 0x11
#define BMI160_GYR_NORMAL_MODE_SET 0x15
#define BMI160_MAG_NORMAL_MODE_SET 0x19

//*****************************************************************************
//
// Function Prototypes.
//
//*****************************************************************************
//
// Initialization Functions
void InitBMI160(uint32_t I2C_base, uint8_t AccelRate, uint8_t AccelAccuracy, uint8_t GyroRate,
                uint8_t GyroAccuracy, uint8_t MagRate, uint8_t *offsetValues, uint32_t sysClockSpeed);

/*
void InitBMM150(uint32_t I2C_base); */


#endif /* BMI160_H_ */
