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
void InitLED(uint32_t SysClockSpeed);

/*
 * UART Initializations
 */
void InitConsole(void);
void InitRadio(uint32_t SysClockSpeed);
void InitGPS(uint32_t SysClockSpeed);
void InitGndMotors(uint32_t SysClockSpeed);


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
void InitSolenoidEnablePins(uint32_t SysClockSpeed);

/*
 * BoostXL-Sensors Initializations
 */
void InitIMU(uint32_t SysClockSpeed, uint8_t *offsetCompensation);
void InitAltimeter(uint32_t SysClockSpeed, int8_t *offsetValues);

/*
 * PWM air motor initializations.
 */
uint32_t InitAirMtrs(uint32_t sysClockSpeed, uint32_t zeroThrottle);
uint32_t InitServoMtrs(uint32_t sysClockSpeed);

#endif /* INITIALIZATIONS_H_ */
