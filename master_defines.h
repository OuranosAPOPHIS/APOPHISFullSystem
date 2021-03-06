/*
 * master_defines.h
 *
 *  Created on: Apr 1, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Universal defines for all .h or .c files
 *      		 within the APOPHIS project.
 */

#ifndef MASTER_DEFINES_H_
#define MASTER_DEFINES_H_

//
// Controls whether the output to the console is
// on or off.
#define DEBUG false

//
// These defines determine what sensors/motors/etc.
// are turned on or off in the program.
#define RADIO_ACTIVATED true
#define GPS_ACTIVATED false
#define GNDMTRS_ACTIVATED true
#define SECONDARY_ATTITUDE false
#define ULTRASONIC_ACTIVATED false
#define PAYLOAD_DEPLOY true
#define IMU_ACTIVATED true
#define ALTIMETER_ACTIVATED false
#define AIRMTRS_ACTIVATED true

//
// This define determines the clock speed.
// If this is false, the system will default to 16MHz.
#define SPEEDIS120MHZ true

#if SPEEDIS120MHZ
#define SYSCLOCKSPEED 120000000
#else
#define SYSCLOCKSPEED 16000000
#endif

//
// Sensor-Boosterpack defines.
// This allows alternation between the three
// different sensor packs and their associated
// calibration data. Only the first true value of
// the three defines will be processed, the others
// will be skipped. It is safe to leave one as true
// and the other two as false.
#define BOOSTERPACK1 false
#define BOOSTERPACK2 true
#define BOOSTERPACK3 false

//
// Parameter which selects whether to use the DCM
// created by TI or the custom DCM created by us.
// True - use custom DCM created by us.
#define CUSTOM_ATTITUDE false

#define STABILITY false

#endif /* MASTER_DEFINES_H_ */
