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
#define	SGX 0.97626628641425f
#define	MGXY 0.00185734724959686f
#define	MGXZ -0.0121570927846085f
#define MGYX 0.00893397192873571f
#define SGY 1.01122447867606f
#define MGYZ -0.0302731537981324f
#define MGZX 0.126466876599477f
#define MGZY 0.0306767129943194f
#define SGZ 1.01398056573882f

//
// Bias for the gyro.
#define BGX -0.915587500000001f
#define BGY 1.5186725f
#define BGZ -0.53449f

//
// INVERSE OF the Scale Factor and Misalignment terms for the Gyro.
#define	SAX -0.897372313143673f
#define	MAXY -7.00158050372183f * 1e-05
#define	MAXZ 0.00589377969817477f
#define MAYX 0.000819963546446416f
#define SAY -0.898218723870705f
#define MAYZ 0.000875976343428164f
#define MAZX 0.00271350463954318f
#define MAZY -0.000807516314877129f
#define SAZ -0.896963110533293f

//
// Bias for the gyro.
#define BAX -0.036071775f
#define BAY 0.0494690250000001f
#define BAZ -0.030288725f

#endif /* SENSORS_ACCEL_GYRO_CAL_DATA_H_ */
