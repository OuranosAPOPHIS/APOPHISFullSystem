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

#define DEBUG true

#define RADIO_ACTIVATED true
#define GPS_ACTIVATED false
#define GNDMTRS_ACTIVATED true
#define SECONDARY_ATTITUDE false
#define ULTRASONIC_ACTIVATED false
#define PAYLOAD_DEPLOY true
#define IMU_ACTIVATED true
#define ALTIMETER_ACTIVATED false
#define AIRMTRS_ACTIVATED false

#define SPEEDIS120MHZ true

#if SPEEDIS120MHZ
#define SYSCLOCKSPEED 120000000
#else
#define SYSCLOCKSPEED 16000000
#endif

#endif /* MASTER_DEFINES_H_ */
