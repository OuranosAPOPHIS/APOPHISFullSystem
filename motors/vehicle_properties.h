/*
 * vehicle_properties.c
 *
 *  Created on: Apr 14, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Defines for mass matrices and
 *              other matrices for quick C math.
 *              All "hard" comutation will be done
 *              in MATLAB first.
 */

//
// Zero and maximum cmd values for the air motors.
#define ZEROTHROTTLE 2200
#define MAXTHROTTLE 3500

//
// Mass moment of inertia matrix, I.
#define IXX 0.0
#define IXY 0.0
#define IXZ 0.0
#define IYX 0.0
#define IYY 0.0
#define IYZ 0.0
#define IZX 0.0
#define IZY 0.0
#define IZZ 0.0

//
// B matrix inverse (4x4), to relate motor thrusts to moments.
#define B11 0.0
#define B12 0.0
#define B13 0.0
#define B14 0.0
#define B21 0.0
#define B22 0.0
#define B23 0.0
#define B24 0.0
#define B31 0.0
#define B32 0.0
#define B33 0.0
#define B34 0.0
#define B41 0.0
#define B42 0.0
#define B43 0.0
#define B44 0.0

//
// Thrust to command ratio based on motor testing. 60N is the
// maximum (ish) thrust achievable by each motor.
#define THRUST_CMD_RATIO ((MAXTHROTTLE - ZEROTHROTTLE) / 60)


