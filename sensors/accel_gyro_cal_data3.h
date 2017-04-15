/*
 * accel_gyro_cal_data3.h
 *
 *  Created on: Apr 12, 2017
 *      Author: Brandon Klefman,
 *              Abby Couto
 *
 *      Purpose: The gyro and accel misalignment and scale
 *               factor terms inverted, for faster operation
 *               in C. These values were found using the orignal
 *               scale factor and misalignment factors and
 *               calculating the inverse using MATLAB.
 *
 *               NOTE: This calibration data is for boosterpack 2.
 */

#ifndef SENSORS_ACCEL_GYRO_CAL_DATA_H_
#define SENSORS_ACCEL_GYRO_CAL_DATA_H_

// NOTE: THESE VALUES ARE THE INVERSE OF THE Mg or Ma MATRICES
// THIS IS DESIGNED TO BE FASTER COMPUTATIONALLY BY PERFOMRING
// THE INVERSE IN MATLAB INSTEAD OF C.

//
// INVERSE OF the Scale Factor and Misalignment terms for the Gyro.
#define SGX 0.959148004220482
#define MGXY 0.0560401313807686
#define MGXZ 0.00859332810519781
#define MGYX 0.058436088280073
#define SGY 0.993226880196119
#define MGYZ -0.00292809065419914
#define MGZX -0.0992920339120276
#define MGZY -0.110378653548624
#define SGZ 0.972913269411589

//
// Bias for the gyro.
#define BGX 0.648820000000001
#define BGY 0.2334225
#define BGZ 1.109945

//
// INVERSE OF the Scale Factor and Misalignment terms for the Accel.
#define SAX 9.79301105733608
#define MAXY -0.0790078431333715
#define MAXZ 0.0592668344835615
#define MAYX -0.0942717860606006
#define SAY 9.72162386114186
#define MAYZ 0.0814474244427025
#define MAZX -0.421080292955642
#define MAZY -0.0771768192364095
#define SAZ 9.54388274720904

//
// Bias for the accel.
#define BAX 0.00178499999999993
#define BAY 0.029113825
#define BAZ 0.0230717499999999

#endif /* SENSORS_ACCEL_GYRO_CAL_DATA_H_ */
