/*
 * accel_gyro_cal_data.h
 *
 *  Created on: Apr 1, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: The gyro and accel misalignment and scale
 *      		 factor terms inverted, for faster operation
 *      		 in C. These values were found using the orignal
 *      		 scale factor and misalignment factors and
 *      		 calculating the inverse using MATLAB.
 */

#ifndef SENSORS_ACCEL_GYRO_CAL_DATA_H_
#define SENSORS_ACCEL_GYRO_CAL_DATA_H_

// NOTE: THESE VALUES ARE THE INVERSE OF THE Mg or Ma MATRICES
// THIS IS DESIGNED TO BE FASTER COMPUTATIONALLY BY PERFOMRING
// THE INVERSE IN MATLAB INSTEAD OF C.

//
// INVERSE OF the Scale Factor and Misalignment terms for the Gyro.
#define	SGX 0.961570687260114
#define	MGXY 0.00909649132638421
#define	MGXZ -0.00156111340088961
#define MGYX 0.0706589410224352
#define SGY 1.00281414151422
#define MGYZ -0.0303132184597918
#define MGZX -0.076811679711435
#define MGZY 0.122405069820481
#define SGZ 0.999855876147308

//
// Bias for the gyro.
#define BGX  0.043825
#define BGY 0.0943224999999988
#define BGZ -0.200075

//
// INVERSE OF the Scale Factor and Misalignment terms for the Accel.
#define	SAX  9.75971310644998
#define	MAXY -0.0073638359438152
#define	MAXZ -0.314722379253915
#define MAYX -0.118337835298923
#define SAY 9.81322196479023
#define MAYZ -0.0629470137174567
#define MAZX 0.0454315080282665
#define MAZY 0.0661366299916046
#define SAZ 9.6935373819151

//
// Bias for the accel.
#define BAX -0.038635275
#define BAY 0.05346685
#define BAZ -0.031036375

#endif /* SENSORS_ACCEL_GYRO_CAL_DATA_H_ */
