/*
 * initializations.h
 *
 *  Created on: Jan 20, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Function prototypes for the initializations.c source file. *
 */

#ifndef INITIALIZATIONS_H_
#define INITIALIZATIONS_H_

#include <stdint.h>

/*
 * LED Initializations.
 */
void InitLED(void);

/*
 * UART Initializations
 */
void InitConsole(void);
void InitRadio(void);
void InitGPS(void);
void InitGndMotors(void);


/*
 * ADC Initializations for Solar Panels.
 */
void InitSolarPanels(void);

/*
 * Ultrasonic Sensor Initializations
 */
void InitUltraSonicSensor(void);

/*
 * Solenoid Enable Pins Initializations
 */
void InitSolenoidEnablePins(void);

/*
 * BoostXL-Sensors Initializations
 */
void InitIMU(uint8_t *offsetCompensation);
void InitAltimeter(int8_t *offsetValues);

/*
 * PWM air motor initializations.
 */
uint32_t InitAirMtrs(uint32_t zeroThrottle);
uint32_t InitServoMtrs(void);

#endif /* INITIALIZATIONS_H_ */
