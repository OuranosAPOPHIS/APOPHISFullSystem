/*
 * accel_gyro_cal_data2.h
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
#define SGX 0.967936321357228
#define MGXY -0.0198665646222568
#define MGXZ 0.0129867246184958
#define MGYX 0.0786732099333824
#define SGY 0.994569924093963
#define MGYZ 0.0319310221833324
#define MGZX -0.127166928760744
#define MGZY -0.0536945296237856
#define SGZ 1.00682862380582

//
// Bias for the gyro.
#define BGX -0.14958
#define BGY 0.357280000000001
#define BGZ -1.1499625

//
// INVERSE OF the Scale Factor and Misalignment terms for the Accel.
#define SAX 9.72593551921637
#define MAXY 0.0213406951775807
#define MAXZ 0.215722388315399
#define MAYX -0.0949114617763137
#define SAY 9.77644497434834
#define MAYZ 0.182867309999372
#define MAZX -0.468245849023067
#define MAZY -0.199592107096355
#define SAZ 9.69830410634976

//
// Bias for the accel.
#define BAX -0.01272575
#define BAY 0.0103455
#define BAZ -0.0156252250000001

#endif /* SENSORS_ACCEL_GYRO_CAL_DATA_H_ */
