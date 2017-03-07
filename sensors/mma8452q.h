/*
 * mma8452q.h
 *
 *  Created on: Feb 10, 2017
 *      Author: Brandon Klefman
 *      Purpose: Define the registers addresses
 *      for the MMA8452Q accelerometer. 
 */

#ifndef MMA8452Q_H
#define MMA8452Q_H

//
// MMA8452Q Device address over I2C.
#define MMA8452Q_DEVICE_ADDRESS 0x1D

//
// Registers.
#define MMA8452Q_STATUS 0x00
#define MMA8452Q_DATA_X 0x01
#define MMA8452Q_DATA_Y 0x03
#define MMA8452Q_DATA_Z 0x05
#define MMA8452Q_SYSMOD 0x0B
#define MMA8452Q_INT_SOURCE 0x0C
#define MMA8452Q_WHO_AM_I 0x0D
#define MMA8452Q_RANGE 0x0E
#define MMA8452Q_HI_PASS_FILTER 0x0F
#define MMA8452Q_PL_STATUS 0x10
#define MMA8452Q_PL_CONFIG 0x11
#define MMA8452Q_PL_COUNT 0x12
#define MMA8452Q_PL_BF_ZCOMP 0x13
#define MMA8452Q_PL_THS_REG 0x14
#define MMA8452Q_FF_MT_CFG 0x15
#define MMA8452Q_FF_MT_SRC 0x16
#define MMA8452Q_FF_MT_THS 0x17
#define MMA8452Q_FF_MT_COUNT 0x18
#define MMA8452Q_TRANSIENT_CFG 0x1D
#define MMA8452Q_TRANSIENT_SRC 0x1E
#define MMA8452Q_TRANSIENT_THS 0x1F
#define MMA8452Q_TRANSIENT_COUNT 0x20
#define MMA8452Q_PULSE_CFG 0x21
#define MMA8452Q_PULSE_SRC 0x22
#define MMA8452Q_THS_X 0x23
#define MMA8452Q_THS_Y 0x24
#define MMA8452Q_THS_Z 0x25
#define MMA8452Q_PULSE_TMLT 0x26
#define MMA8452Q_PULSE_LTCY 0x27
#define MMA8452Q_PULSE_WIND 0x28
#define MMA8452Q_ALP_COUNT 0x29
#define MMA8452Q_CTRL_REG1 0x2A
#define	MMA8452Q_CTRL_REG2 0x2B
#define MMA8452Q_CTRL_REG3 0x2C
#define MMA8452Q_CTRL_REG4 0x2D
#define MMA8452Q_CTRL_REG5 0x2E
#define MMA8452Q_OFF_X 0x2F
#define MMA8452Q_OFF_Y 0x30
#define MMA8452Q_OFF_Z 0x31

//
// Register: Status
#define MMA8452Q_DATA_RDY 0x08

//
// Register: SYSMOD
#define MMA8452Q_NORMAL_MODE 0x01

//
// Register: INT_SOURCE
#define MMA8452Q_INT_DDRY 0x01

//
// Register: XYZ_DATA_CFG
#define MMA8452Q_RATE_2G 0x00
#define MMA8452Q_RATE_4G 0x01
#define MMA8452Q_RATE_8G 0x02
#define MMA8452Q_HIGH_PASS_ON 0x10

//
// Register: HP_FILTER_CUTOFF

//
// Register: PL_STATUS

//
// Register: PL_CFG

//
// Register: PL_COUNT

//
// Register: PL_BF_ZCOMP

//
// Register: PL_THS_REG

//
// Register: FF_MT_CFG

//
// Register: FF_MT_SRC

//
// Register: FF_MT_THS

//
// Register: FF_MT_COUNT

//
// Register: TRANSIENT_CFG

//
// Register: TRANSIENT_SRC

//
// Register: TRANSIENT_THS

//
// Register: TRANSIENT_COUNT

//
// Register: CTRL_REG_1
#define MMA8452Q_50_HZ 0x20
#define MMA8452Q_WAKE_MODE 0x01

//
// Register: CTRL_REG_2
#define MMA8452Q_RESET 0x40

//
// Register: CTRL_REG_4
#define MMA8452Q_INT_DDRY_EN 0x01

//
// Register CTRL_REG_5
#define MMA8452Q_INT_DDRY_INT1 0x01

//
// Function Prototypes
void InitMMA8452Q(uint32_t I2C_base, uint8_t AccelRange, uint8_t AccelRate);


#endif /* MMA8452Q_H */
