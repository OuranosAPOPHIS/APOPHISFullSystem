/*
 * attitude_estimation.c
 *
 *  Created on: Apr 2, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Custom attitude estimation algorithm
 *
 *      Note: These functions assume a time step of DT
 *      	  which is defined in the associated header
 *      	  file. The default setting for DT is 0.01.
 *
 *      Portions of this code were derived from TI's
 *      sensorlib compDCM functions. Additionally, a
 *      portion of this code was derived from github user Skadi15 - Christian Moncada.
 *      https://github.com/Skadi15/Overlord_System/blob/master/sensors/windrose_module.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "sensorlib/vector.h"

#include "attitude_estimation.h"
//
// Time step for the control law.
#define DT 0.01
/*
 * Initialize the attitude.
 * Parameter(s):
 *  *sAttData - pointer to the attitude data structure as defined above.
 * 	*fAccel - pointer to the accel data.
 * 	*fGyro - pointer to the gyro data.
 * 	*fMag - pointer to the mag data.
 * 	*fEuler - pointer to the Euler angles (RPY).
 * 	fGyroWeight - weighting of the gyro.
 */
void InitAttitude(sAttitudeData *sAttData, float fGyroWeight)
{
    //
    // Initialize the DCM as an identity matrix.
    sAttData->fDCM[0][0] = 1.0;
    sAttData->fDCM[0][1] = 0.0;
    sAttData->fDCM[0][2] = 0.0;
    sAttData->fDCM[1][0] = 0.0;
    sAttData->fDCM[1][1] = 1.0;
    sAttData->fDCM[1][2] = 0.0;
    sAttData->fDCM[2][0] = 0.0;
    sAttData->fDCM[2][1] = 0.0;
    sAttData->fDCM[2][2] = 1.0;


	//
	// Set the structure values.
	sAttData->fGyroWeight = fGyroWeight;
}

/*
 * Update the accelerometer values.
 * Parameter(s):
 *  *sAttData - pointer to the attitude data structure as defined above.
 *  fAccelX,Y,Z - current x,y,z values for the accelerometer.
 */
void UpdateAccel(sAttitudeData *sAttData, float fAccelX, float fAccelY, float fAccelZ)
{
	//
	// Update the accelerometer values in the struct.
	sAttData->fAccelX = fAccelX;
	sAttData->fAccelY = fAccelY;
	sAttData->fAccelZ = fAccelZ;
}

/*
 * Update the gyroscope values.
 * Parameter(s):
 *  *sAttData - pointer to the attitude data structure as defined above.
 *  fGyroX,Y,Z - current x,y,z values for the gyroscope.
 */
void UpdateGyro(sAttitudeData *sAttData, float fGyroX, float fGyroY, float fGyroZ)
{
	//
	// Update the gyroscope values in the struct.
	sAttData->fGyroX = fGyroX;
	sAttData->fGyroY = fGyroY;
	sAttData->fGyroZ = fGyroZ;
}

/*
 * Update the magnetometer values.
 * Parameter(s):
 *  *sAttData - pointer to the attitude data structure as defined above.
 *  fMagX,Y,Z - current x,y,z values for the magnetometer.
 */
void UpdateMag(sAttitudeData *sAttData, float fMagX, float fMagY, float fMagZ)
{
	//
	// Update the magnetometer values in the struct.
	sAttData->fMagX = fMagX;
	sAttData->fMagY = fMagY;
	sAttData->fMagZ = fMagZ;
}

/*
 * InitHeading
 * Parameter(s):
 * *sAttData - Pointer to the sAttitudeData struct defined above.
 * Purpose:
 *  Calculates the absolute 2D heading based on the magnetometer reading.
 */
void InitHeading(sAttitudeData *sAttData)
{
    /*
     * Normally, initialize Yaw as magnetometer reading.
     * sAttData->fYaw = atan2(-sAttData->fMagY, sAttData->fMagX);
     * For purpose of IMU validation on 3-axis rotation table, initialize yaw as 0.
     */
    sAttData->fYaw = 0.0f;
}

/*
 * UpdateYaw
 * Parameter(s):
 *  *sAttData - Pointer to the sAttitudeData struct defined above.
 * Purpose:
 *  Updates the current heading based on magetometer and gyroscope readings.
 *
 * Note: This function must use calibrated gyro and mag data.
 */
void UpdateYaw(sAttitudeData *sAttData)
{
    float fGyroHeading;
    float fMagHeading;

    //
    // Check what reading the gyro is giving and convert to radians.
    fGyroHeading = sAttData->fYaw * DEG2RAD;

    //
    // Get the mag heading in radians.
    /*
     * Normally, update Yaw based on magnetometer reading.
     * fMagHeading = atan2(-sAttData->fMagY, sAttData->fMagX);
     * For purpose of IMU validation on 3-axis rotation table, update yaw using DynamicUpdateAttitude
     */
    fMagHeading = 0.0f;

    //
    // Check for negative mag values.
    if (fMagHeading < 0)
        fMagHeading += 2 * M_PI;

    //
    // Keep the values between +M_PI and -M_PI.
    if ((fMagHeading - fGyroHeading) < -M_PI)
        fMagHeading += 2 * M_PI;
    else if ((fMagHeading - fGyroHeading) > M_PI)
        fGyroHeading += 2 * M_PI;

    //
    // Update the yaw.
    sAttData->fYaw = (fMagHeading * (1 - (sAttData->fGyroWeight)) +
            fGyroHeading * (sAttData->fGyroWeight)) * RAD2DEG;

    //
    // Keep yaw between 0 and 360 degrees.
    if (sAttData->fYaw > 360.0f)
        sAttData->fYaw -= 360.0f;
}

/*
 * StaticUpdateAttitude
 * Update the attitude based on the sensor readings.
 * Parameter(s):
 *  *sAttData - pointer to the attitude data structure as defined above.
 *
 *  Call this function only when the vehicle is not moving
 *  Use the DynamicUpdateAttitude if moving.
 */
void StaticUpdateAttitude(sAttitudeData *sAttData)
{
    float pfI[3], pfJ[3], pfK[3];

    //
    // The magnetometer reading forms the initial I vector, pointing north.
    /*
     * Normally, initialize Yaw as magnetometer reading.
     * pfI[0] = sAttData->fMagX;
     * pfI[1] = sAttData->fMagY;
     * pfI[2] = sAttData->fMagZ;
     * For purpose of IMU validation on 3-axis rotation table, initialize I as [1;0;0].
     */
    pfI[0] = 1.0f;
    pfI[1] = 0.0f;
    pfI[2] = 0.0f;

    //
    // The accelerometer reading forms the initial K vector, pointing down.
    pfK[0] = sAttData->fAccelX;
    pfK[1] = sAttData->fAccelY;
    pfK[2] = sAttData->fAccelZ;

    //
    // Compute the initial J vector, which is the cross product of the K and I
    // vectors.
    VectorCrossProduct(pfJ, pfK, pfI);

    //
    // Recompute the I vector from the cross product of the J and K vectors.
    // This makes it fully orthogonal, which it wasn't before since magnetic
    // north points inside the Earth in many places.
    VectorCrossProduct(pfI, pfJ, pfK);

    //
    // Normalize the I, J, and K vectors.
    VectorScale(pfI, pfI, 1 / sqrtf(VectorDotProduct(pfI, pfI)));
    VectorScale(pfJ, pfJ, 1 / sqrtf(VectorDotProduct(pfJ, pfJ)));
    VectorScale(pfK, pfK, 1 / sqrtf(VectorDotProduct(pfK, pfK)));

    //
    // Initialize the DCM matrix from the I, J, and K vectors.
    sAttData->fDCM[0][0] = pfI[0];
    sAttData->fDCM[0][1] = pfI[1];
    sAttData->fDCM[0][2] = pfI[2];
    sAttData->fDCM[1][0] = pfJ[0];
    sAttData->fDCM[1][1] = pfJ[1];
    sAttData->fDCM[1][2] = pfJ[2];
    sAttData->fDCM[2][0] = pfK[0];
    sAttData->fDCM[2][1] = pfK[1];
    sAttData->fDCM[2][2] = pfK[2];
}

/*
 * DynamicUpdateAttitude
 * Parameter(s):
 * 	*sAttData - Pointer to the sAttitudeData struct defined above.
 * Purpose:
 * 	Updates the current DCM based on gyroscope readings and updates the Euler Angles.
 *
 * Note: This function must use calibrated accel, gyro and mag data.
 * This function was adapted from Dr. Stephen Bruder's MATLAB code
 * available on his website.
 * http://mercury.pr.erau.edu/~bruders/teaching/2017_spring/EE440/lectures/lecture%2016_Inertial_Nav_ECEF.pdf
 */
void DynamicUpdateAttitude(sAttitudeData *sAttData)
{
    float C[3][3] = { 0 };
    float fGyroVecNorm[3] = { 0 };
    float AMatrix[3][3] = { 0 };
    float IMatrix[3][3] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };

    //
	// Skew symmetric matrix.
	float K[3][3] = { 0 };

	//
	// Square of skew symmetric matrix.
	float K2[3][3] = { 0 };

	//
	// Create vector of gyroscope readings.
	float fGyroVec[3] = {DEG2RAD*((sAttData->fGyroX)),
	                     DEG2RAD*((sAttData->fGyroY)),
	                     DEG2RAD*((sAttData->fGyroZ))};
    //
    // Compute sin(theta)
    float fAScalar = sin(DT * sqrtf(VectorDotProduct(fGyroVec, fGyroVec)));

	//
	// Normalized vector of gyroscope data.
	VectorScale(fGyroVecNorm, fGyroVec,1 / sqrtf(VectorDotProduct(fGyroVec, fGyroVec)));

	//
	// Compute the skew-symmetric matrix based on the gyroscope data.
	ComputeSkewMatrix(fGyroVecNorm, K);

	//
	// Compute sin(theta)*K
	MatrixScale3x3(AMatrix, K, fAScalar);

    //
    // Identity + sin(Theta)*K.
	MatrixAdd3x3(C, AMatrix, IMatrix);

    //
    // 1 - cos(Theta).
    float Bscalar = (1 - cos(DT * sqrtf(VectorDotProduct(fGyroVec, fGyroVec))));

	//
	// Compute the square of the skew-symmetric matrix.
	MatrixMultiply3x3(K2, K, K);

	//
	// Compute the Rodrigues Formula
	//C = I + Sin(theta) K + (1-Cos(theta)) K^2  = Rodrigues formula
	MatrixScale3x3(K2, K2, Bscalar);
	MatrixAdd3x3(C, C, K2);

	//
	// Take the DCM update matrix and update the current DCM.
	MatrixMultiply3x3(sAttData->fDCM, sAttData->fDCM, C);
}

/*
 * UpdateEulers
 * Parameter(s):
 *  *sAttData - Pointer to the sAttitudeData struct defined above.
 * Purpose:
 *  Update the Euler Angles based on the new DCM matrix.
 */
void UpdateEulers(sAttitudeData *sAttData)
{
    //
    // Compute the roll, pitch, and yaw as required.
    sAttData->fRoll = atan2f(sAttData->fDCM[2][1], sAttData->fDCM[2][2]) * RAD2DEG;
    sAttData->fPitch = -asinf(sAttData->fDCM[2][0]) * RAD2DEG;
    sAttData->fYaw = atan2f(sAttData->fDCM[1][0], sAttData->fDCM[0][0]) * RAD2DEG;

    //
    // Keep pitch between -90 and +90
    if (sAttData->fPitch > 90.0f)
        sAttData->fPitch -= 180.0f;
    else if (sAttData->fPitch < -90.0f)
        sAttData->fPitch += 180.0f;

    //
    // Check yaw angle -> 0 to 360.
    if (sAttData->fYaw < 0)
        sAttData->fYaw += 360.0f;
}

/*
 * ComputeSkewMatrix
 * Parameter(s):
 *  fGyroVecNorm[3] INPUT 3x1 normalized vector of Gyro Measurements
 *  K[3][3] OUTPUT 3x3 skew matrix of normalized gyro readings
 * Purpose:
 *  Computes skew matrix for DCM propagation as a function of gyro readings
 
 * Note: This function must use calibrated accel and gyro data.
 * This function is adapted from Dr. Bruder's MATLAB code, vec2ss
 * http://mercury.pr.erau.edu/~bruders/teaching/2017_spring/EE440/
 */
void ComputeSkewMatrix(float fGyroVecNorm[3], float K[3][3])
{
	K[0][0] = 0;
	K[0][1] = -fGyroVecNorm[2];
    K[0][2] = fGyroVecNorm[1];
    K[1][0] = fGyroVecNorm[2];
    K[1][1] = 0;
    K[1][2] = -fGyroVecNorm[0];
    K[2][0] = -fGyroVecNorm[1];
    K[2][1] = fGyroVecNorm[0];
    K[2][2] = 0;
}

/*
 * MatrixMultiply3x3
 * Parameter(s):
 *  fProduct[3][3] OUTPUT 3x3 matrix product
 *  fA[3][3] INPUT 3x3 matrix on the left
 *  fB[3][3] INPUT 3x3 matrix on the right
 *  
 * Purpose:
 *  Matrix mulitplication of two matrices that are both 3x3
 *
 */
void MatrixMultiply3x3(float fProduct[3][3], float fA[3][3], float fB[3][3])
{

	fProduct[0][0] = fA[0][0] * fB[0][0] + fA[0][1] * fB[1][0] + fA[0][2] * fB[2][0];
    fProduct[0][1] = fA[0][0] * fB[0][1] + fA[0][1] * fB[1][1] + fA[0][2] * fB[2][1];
    fProduct[0][2] = fA[0][0] * fB[0][2] + fA[0][1] * fB[1][2] + fA[0][2] * fB[2][2];
    fProduct[1][0] = fA[1][0] * fB[0][0] + fA[1][1] * fB[1][0] + fA[1][2] * fB[2][0];
    fProduct[1][1] = fA[1][0] * fB[0][1] + fA[1][1] * fB[1][1] + fA[1][2] * fB[2][1];
    fProduct[1][2] = fA[1][0] * fB[0][2] + fA[1][1] * fB[1][2] + fA[1][2] * fB[2][2];
    fProduct[2][0] = fA[2][0] * fB[0][0] + fA[2][1] * fB[1][0] + fA[2][2] * fB[2][0];
    fProduct[2][1] = fA[2][0] * fB[0][1] + fA[2][1] * fB[1][1] + fA[2][2] * fB[2][1];
    fProduct[2][2] = fA[2][0] * fB[0][2] + fA[2][1] * fB[1][2] + fA[2][2] * fB[2][2];
}

/*
 * 3x3 Matrix Addition.
 * Parameter(s):
 *  fSum[3][3] OUTPUT 3x3 matrix sum
 *  fA[3][3] INPUT 3x3 matrix on the left
 *  fB[3][3] INPUT 3x3 matrix on the right 
 *  Purpose:
 *  Performs matrix addition of two matrices that are both 3x3
 */
void MatrixAdd3x3(float fSum[3][3], float fA[3][3], float fB[3][3])
{
    fSum[0][0] = fA[0][0] + fB[0][0];
    fSum[0][1] = fA[0][1] + fB[0][1];
    fSum[0][2] = fA[0][2] + fB[0][2];
    fSum[1][0] = fA[1][0] + fB[1][0];
    fSum[1][1] = fA[1][1] + fB[1][1];
    fSum[1][2] = fA[1][2] + fB[1][2];
    fSum[2][0] = fA[2][0] + fB[2][0];
    fSum[2][1] = fA[2][1] + fB[2][1];
    fSum[2][2] = fA[2][2] + fB[2][2];
}

/*
 * 3x3 Matrix Scalar Multiplication.
 * Parameter(s):
 *  fProduct[3][3] OUTPUT 3x3 matrix product
 *  fMatrix[3][3] INPUT 3x3 matrix 
 *  fScalar INPUT scalar matrix multiplier
 *  
 * Purpose:
 *  Scalar mulitplication of a 3x3 matrix
 *
 */
void MatrixScale3x3(float fProduct[3][3], float fMatrix[3][3], float fScalar)
{
    fProduct[0][0] = fMatrix[0][0] * fScalar;
    fProduct[0][1] = fMatrix[0][1] * fScalar;
    fProduct[0][2] = fMatrix[0][2] * fScalar;
    fProduct[1][0] = fMatrix[1][0] * fScalar;
    fProduct[1][1] = fMatrix[1][1] * fScalar;
    fProduct[1][2] = fMatrix[1][2] * fScalar;
    fProduct[2][0] = fMatrix[2][0] * fScalar;
    fProduct[2][1] = fMatrix[2][1] * fScalar;
    fProduct[2][2] = fMatrix[2][2] * fScalar;
}
