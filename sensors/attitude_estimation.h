/*
 * attitude_estimation.h
 *
 *  Created on: Apr 2, 2017
 *      Author: Brandon Klefman
 */

#ifndef ATTITUDE_ESTIMATION_H_
#define ATTITUDE_ESTIMATION_H_

#define DT 0.01
#define DEG2RAD 0.01745329251
#define RAD2DEG 57.2957795131

#define M_PI 3.14159265358979323846

//****************************************************************************************
//
// Structure to hold all the sensor data.
//
//****************************************************************************************
typedef struct {
    float fDCM[3][3];
	float fAccelX;
	float fAccelY;
	float fAccelZ;
	float fGyroX;
	float fGyroY;
	float fGyroZ;
	float fMagX;
	float fMagY;
	float fMagZ;
	float fPitch;
	float fRoll;
	float fYaw;
	float fGyroWeight;
} sAttitudeData;

//****************************************************************************************
//
// Function Prototypes.
//
//****************************************************************************************
void InitAttitude(sAttitudeData *sAttData, float fGyroWeight);
void UpdateAccel(sAttitudeData *sAttData, float fAccelX, float fAccelY, float fAccelZ);
void UpdateGyro(sAttitudeData *sAttData, float fGyroX, float fGyroY, float fGyroZ);
void UpdateMag(sAttitudeData *sAttData, float fMagX, float fMagY, float fMagZ);
void InitHeading(sAttitudeData *sAttData);
void UpdateYaw(sAttitudeData *sAttData);
void StaticUpdateAttitude(sAttitudeData *sAttData);
void DynamicUpdateAttitude(sAttitudeData *sAttData);
void UpdateEulers(sAttitudeData *sAttData);
void ComputeSkewMatrix(float fGyroVecNorm[3], float K[3][3]);
void MatrixMultiply3x3(float fProduct[3][3], float fA[3][3], float fB[3][3]);
void MatrixAdd3x3(float fSum[3][3], float fA[3][3], float fB[3][3]);
void MatrixScale3x3(float fProduct[3][3], float fMatrix[3][3], float fScalar);

#endif /* ATTITUDE_ESTIMATION_H_ */
