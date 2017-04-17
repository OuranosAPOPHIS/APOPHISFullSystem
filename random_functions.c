/*
 * random_functions.c
 *
 *  Created on: Apr 14, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Extra functions, which we may
 *          never end up getting around to finishing.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

//*****************************************************************************
//
// This function will update the trajectory of the platform.
//
//*****************************************************************************
void AutoFlyUpdate(void) {
/*
    float fXdotDes;
    float fYdotDes;
    float fZdotDes;
    float fXdotdotDes;
    float fYdotdotDes;
    float fZdotdotDes;
    float fXdotCurrent;
    float fYdotCurrent;
    float fZdotCurrent;
    float fKpX;
    float fKpY;
    float fKpZ;
    float fKdX;
    float fKdY;
    float fKdZ;
    float fKpR;
    float fKpP;
    float fKpYa;
    float fKdR;
    float fKdP;
    float fKdYa;
    float fFx;
    float fFy;
    float fFz;
    float fFzSat;
    float fFMax;
    float fThrust;
    float fRollDes;
    float fPitchDes;
    float fYawDes;
    float fRolldotDes;
    float fPitchdotDes;
    float fYawdotDes;
    float fRolldotdotDes;
    float fPitchdotdotDes;
    float fYawdotdotDes;
    float fMatThdotdot[3];
    float fMatBinv[4][4];
    float fMatTorque[4];
    float fThrustDes[4];

    //
    // TODO: This is where the control law and stuff will go.
    //
    // Check if radio is sending good data.
    if (sStatus.bTargetSet) {
        //
        // Radio data is good, calculate a trajectory.

        //
        // TODO: Calculate a trajectory.
        sStatus.fRoll = sAttData.fRoll;
        sStatus.fPitch = sAttData.fPitch;
        sStatus.fYaw = sAttData.fYaw;
        if (sSensStatus.fThrust ==0){
        StaticUpdateAttitude(&sAttData);
        }else{
        DynamicUpdateAttitude(&sAttData);
        }
        sAttData.fRoll = sStatus.fRoll;
        sAttData.fPitch = sStatus.fPitch;
        sAttData.fYaw = sStatus.fYaw;
        if ((sStatus.fCurrentAlt > sStatus.fTargetAlt) && (sStatus.fCurrentAlt - sStatus.fTargetAlt > 0.03)) {
                sStatus.fTargetAlt = sStatus.fCurrentAlt - 0.03;
        }

        fKpX = 0.1;
        fKpY = 1;
        fKpZ = 0.075;

        fXdotDes = (sStatus.fCurrentLat - sStatus.fTargetLat) * (111.2 / 0.001) * fKpX; //Each 0.001 degrees of latitude equates to 111.2 meters in Embry-Riddle Aereonatical University, Prescott AZ, Lowwer Fields
        fYdotDes = (sStatus.fCurrentLong - sStatus.fTargetLong) * (91.51 / 0.001) * fKpY; //Each 0.001 degrees of latitude equates to 91.51 meters in Embry-Riddle Aereonatical University, Prescott AZ, Lowwer Fields
        fZdotDes = (-sStatus.fCurrentAlt - -sStatus.fTargetAlt) * fKpZ;

        fXdotCurrent = sSensStatus.fPrevVelX + (DT * sSensStatus.fPreviousAccelX);
        sSensStatus.fPrevVelX = fXdotCurrent;
        fYdotCurrent = sSensStatus.fPrevVelY + (DT * sSensStatus.fPreviousAccelY);
        sSensStatus.fPrevVelY = fYdotCurrent;
        fZdotCurrent = sSensStatus.fPrevVelZ + (DT * sSensStatus.fPreviousAccelZ);
        sSensStatus.fPrevVelZ = fZdotCurrent;

        fKdX = 1;
        fKdY = 1;
        fKdZ = 2;

        fXdotdotDes = (fXdotDes - fXdotCurrent) * fKdX;
        fYdotdotDes = (fYdotDes - fYdotCurrent) * fKdY;
        fZdotdotDes = (fZdotDes - fZdotCurrent) * fKdZ;

        fFx = fXdotdotDes * sStatus.fMass;
        fFy = fYdotdotDes * sStatus.fMass;
        fFz = fZdotdotDes * sStatus.fMass;
        fFMax = 238.3101;

        if (fFz>fFMax){
            fFzSat = fFMax;
        }else if (fFz<-9.8*sStatus.fMass){
            fFzSat = -9.8*sStatus.fMass;
        }else{
            fFzSat = fFz;
        }

        fRollDes = fFy / sSensStatus.fThrust;
        fPitchDes = -fFx / sSensStatus.fThrust;
        fYawDes = 0;

        fKpR = 2.09974943882214;
        fKpP = 2.09974943882214;
        fKpYa = 2.09974943882214;

        fRolldotDes = (fRollDes - sStatus.fRoll) * fKpR;
        fPitchdotDes = (fPitchDes - sStatus.fPitch) * fKpP;
        fYawdotDes = (fYawDes - sStatus.fYaw) * fKpYa;

        fKdR = 4;
        fKdP = 4;
        fKdYa = 4;

        fRolldotdotDes = (fRolldotDes - sSensStatus.fCurrentGyroX) * fKdR;
        fPitchdotdotDes = (fPitchdotDes - sSensStatus.fCurrentGyroY) * fKdP;
        fYawdotdotDes = (fYawdotDes - sSensStatus.fCurrentGyroZ) * fKdYa;

// This matrix math is definitely wrong.
        fMatThdotdot[0] = fRolldotdotDes;
        fMatThdotdot[1] = fPitchdotdotDes;
        fMatThdotdot[2] = fYawdotdotDes;
        fMatTorque[0] = fMatI[0][0] * fMatThdotdot[0] + fMatI[0][1] * fMatThdotdot[1] + fMatI[0][2] * fMatThdotdot[2];
        fMatTorque[1] = fMatI[1][0] * fMatThdotdot[0] + fMatI[1][1] * fMatThdotdot[1] + fMatI[1][2] * fMatThdotdot[2];
        fMatTorque[2] = fMatI[2][0] * fMatThdotdot[0] + fMatI[2][1] * fMatThdotdot[1] + fMatI[2][2] * fMatThdotdot[2];
        fMatTorque[3] = fFzSat;

        fThrustDes[0] = fBinx[0][0] * fMatThdotdot[0] + fBinv[0][1] * fMatThdotdot[1] + fBinv[0][2] * fMatThdotdot[2] + Binv[0][3] * fMatThdotdot[3];
        fThrustDes[1] = fBinx[1][0] * fMatThdotdot[0] + fBinv[1][1] * fMatThdotdot[1] + fBinv[1][2] * fMatThdotdot[2] + Binv[1][3] * fMatThdotdot[3];
        fThrustDes[2] = fBinx[2][0] * fMatThdotdot[0] + fBinv[2][1] * fMatThdotdot[1] + fBinv[2][2] * fMatThdotdot[2] + Binv[2][3] * fMatThdotdot[3];
        fThrustDes[3] = fBinx[3][0] * fMatThdotdot[0] + fBinv[3][1] * fMatThdotdot[1] + fBinv[3][2] * fMatThdotdot[2] + Binv[3][3] * fMatThdotdot[3];
        sSensStatus.fThrust = fThrustDes[0] + fThrustDes[1] + fThrustDes[2] + fThrustDes[3];
    } else {
        //
        // Radio data is bad. Set the current location as the target location.
        sStatus.fTempTargetLat = sStatus.fCurrentLat;
        sStatus.fTempTargetLong = sStatus.fCurrentLong;

        //
        // TODO: Add some logic, so that if we lose radio contact, we
        // don't necessarily crash...
    }

#if DEBUG
    //
    // Reset printing loop count for debugging.
    g_PrintFlag = false;
#endif
*/
}

//*****************************************************************************
//
// This function will update the trajectory of the platform in autonomous mode
// and driving.
//
//*****************************************************************************
void AutoDriveUpdate(void)
{
    //
    // TODO: This is where the control law and stuff will go.
    //
    // Check if radio is sending good data.
    /*
    float fXdotDes;
    float fYdotDes;
    float fKpR;
    float fXdotCurrent;
    float fYdotCurrent;
    float fXdotdotDes;
    float fKdR;
    float fThDes;
    float fThdotDes;
    float fKpTh;
    float fThdotdotDes;
    float fKdTh;
    float fRadiusWheel;
    float fKt;
    float fResistanceMotor;
    float fVoltLeft;
    float fVoltRight;

    if (sStatus.bTargetSet) {
        //
        // Radio data is good, calculate a trajectory.
        fKpR = 0.08;
        fXdotDes = (sStatus.fCurrentLat - sStatus.fTargetLat) * (111.2 / 0.001) * fKpR; //Each 0.001 degrees of latitude equates to 111.2 meters in Embry-Riddle Aereonatical University, Prescott AZ, Lowwer Fields
        fYdotDes = (sStatus.fCurrentLong - sStatus.fTargetLong) * (91.51 / 0.001) * fKpR; //Each 0.001 degrees of latitude equates to 91.51 meters in Embry-Riddle Aereonatical University, Prescott AZ, Lowwer Fields
        fXdotCurrent = sSensStatus.fPrevVelX + (DT * sSensStatus.fPreviousAccelX); //Are Accelerations im body or imirtia; frame, Inirtial was assumed
        sSensStatus.fPrevVelX = fXdotCurrent;
        fYdotCurrent = sSensStatus.fPrevVelY + (DT * sSensStatus.fPreviousAccelY);
        sSensStatus.fPrevVelY = fYdotCurrent;
        fKdR = 10;
        fXdotdotDes = (fXdotDes - fXdotCurrent) * fKdR;
        if ((sStatus.fCurrentLong - sStatus.fTargetLong) * (91.51 / 0.001) > 2.5 && (sStatus.fCurrentLong - sStatus.fTargetLong) * (91.51 / 0.001) < -2.5){
            fThDes = atan(((sStatus.fCurrentLat - sStatus.fTargetLat) * (111.2 / 0.001)) / ((sStatus.fCurrentLong - sStatus.fTargetLong) * (91.51 / 0.001)));
            fKpTh = 0.8;
            fThdotDes = (fThdes - sStatus.fYaw) * fKpTh;
            fKdTh = 3;
            fThdotdotDes = (fThdotDes - sSensStatus.fCurrentGyroZ) * fKdTh;
        }else{
            fRadiusWheel = ;
            fKt = ;
            fResistanceMotor = ;
            fVoltLeft = (((fXdotdotDes / (2 * sStatus.fMass)) * fRadiusWheel) / fKt) * fResistanceMotor;
        }

    } else {
        //
        // Radio data is bad. Set the current location as the target location.
        sStatus.fTempTargetLat = sStatus.fCurrentLat;
        sStatus.fTempTargetLong = sStatus.fCurrentLong;

        //
        // TODO: Add some logic, so that if we lose radio contact, we
        // don't necessarily crash...
    }


    //
    // Set the ground motors to zero throttle.
    //
    uint8_t txBuffer[4];

    // Build the instruction packet.
    txBuffer[0] = RX24_WRITE_DATA;
    txBuffer[1] = RX24_REG_MOVING_VEL_LSB;
    txBuffer[2] = 0x00;
    txBuffer[3] = 0x00;

    //
    // Send the command to the LW motor.
    Rx24FWrite(GNDMTR1_UART, GNDMTR1_DIRECTION_PORT, GMDMTR1_DIRECTION, 4,
            txBuffer);

    //
    // Send the command to the RW motor.
    Rx24FWrite(GNDMTR2_UART, GNDMTR2_DIRECTION_PORT, GMDMTR2_DIRECTION, 4,
            txBuffer);

#if DEBUG
    //
    // Reset printing loop count for debugging.
    g_PrintFlag = false;
#endif */
}

//*****************************************************************************
//
// This function will update the trajectory of the platform in manual mode
// and flying.
//
//*****************************************************************************
void ManualFlyUpdate(void)
{
/*
    float fDesiredRoll = 0.0f;
    float fDesiredPitch = 0.0f;
    //
    // TODO: Figure out a good yaw rate.

    //
    // We are flying. Set the parameters sent from the radio.
    // Get the throttle.
    int32_t ui32Throttle = (int32_t) (g_sRxPack.sControlPacket.throttle);

#if DEBUG
    UARTprintf("Throttle: %d\r\n", ui32Throttle);
#endif

    //
    // Check to make sure throttle isn't negative.
    if (ui32Throttle < 0)
        ui32Throttle = 0;

    sThrottle.fAirMtr1Throttle = (ui32Throttle
            * g_ui32ThrottleIncrement) + ZEROTHROTTLE;//HOVERTHROTTLE1;
    sThrottle.fAirMtr2Throttle = (ui32Throttle
            * g_ui32ThrottleIncrement) + ZEROTHROTTLE;//HOVERTHROTTLE2;
    sThrottle.fAirMtr3Throttle = (ui32Throttle
            * g_ui32ThrottleIncrement) + ZEROTHROTTLE;//HOVERTHROTTLE3;
    sThrottle.fAirMtr4Throttle = (ui32Throttle
            * g_ui32ThrottleIncrement) + ZEROTHROTTLE;//HOVERTHROTTLE4;

        //
        // Get the yaw value.
        int32_t i32Yaw = (int32_t) (g_sRxPack.sControlPacket.yaw);
#if DEBUG
        //
        // Get the roll, pitch, yaw for printing to the console.
        int32_t i32Roll = (int32_t) (g_sRxPack.sControlPacket.roll) * 25
                / 100;
        int32_t i32Pitch = (int32_t) (g_sRxPack.sControlPacket.pitch
                * 25 / 100);

        if (g_PrintFlag) {
            UARTprintf("Throttle: %d\r\n", ui32Throttle);
            UARTprintf(
                    "Desired Roll: %d\r\nDesired Pitch: %d\r\nYaw: %d\r\n",
                    i32Roll, i32Pitch, i32Yaw);
        }
#endif
        //
        // Calculate the roll, pitch and yaw.
        fDesiredRoll = g_sRxPack.sControlPacket.roll / 100.0f * 25.0f;
        fDesiredPitch = g_sRxPack.sControlPacket.pitch / 100.0f * 25.0f; */
/*
        //
        // Check if the pitch error is less than 0.5 or -0.5.
        if (((sStatus.fPitch - fDesiredPitch) > 0.5f)
                || ((sStatus.fPitch - fDesiredPitch) < -0.5f)) {
            //
            // Check the pitch.
            // Pitch is less than desired and negative.
            if ((sStatus.fPitch < fDesiredPitch)
                    && (sStatus.fPitch < 0)) {
#if APOPHIS
                //
                // "Pull Up", increase front motor throttle and decrease back motor throttle.
                sThrottle.fAirMtr1Throttle += g_ui32ThrottleIncrement;
                sThrottle.fAirMtr3Throttle -= g_ui32ThrottleIncrement;
#else
                sThrottle.fAirMtr1Throttle += g_ui32ThrottleIncrement;
                sThrottle.fAirMtr3Throttle -= g_ui32ThrottleIncrement;
                sThrottle.fAirMtr4Throttle -= g_ui32ThrottleIncrement;
                sThrottle.fAirMtr6Throttle += g_ui32ThrottleIncrement;
#endif
                if (g_PrintFlag) {
                    UARTprintf("Neg Pitch and Pull Up\r\n");
                }
            }
            //
            // Pitch is greater than desired and negative.
            else if ((sStatus.fPitch > fDesiredPitch)
                    && (sStatus.fPitch < 0)) {
#if APOPHIS
                //
                // "Pull Down", decrease front motor throttle and increase back motor throttle.
                sThrottle.fAirMtr1Throttle -= g_ui32ThrottleIncrement;
                sThrottle.fAirMtr3Throttle += g_ui32ThrottleIncrement;
#else
                sThrottle.fAirMtr1Throttle -= g_ui32ThrottleIncrement;
                sThrottle.fAirMtr3Throttle += g_ui32ThrottleIncrement;
                sThrottle.fAirMtr4Throttle += g_ui32ThrottleIncrement;
                sThrottle.fAirMtr6Throttle -= g_ui32ThrottleIncrement;
#endif
                if (g_PrintFlag) {
                    UARTprintf("Neg Pitch and Pull Down\r\n");
                }
            }
            //
            // Pitch is less than desired and positive.
            else if ((sStatus.fPitch < fDesiredPitch)
                    && (sStatus.fPitch > 0)) {
#if APOPHIS
                //
                // "Pull Up", increase front motor throttle and decrease back motor throttle.
                sThrottle.fAirMtr1Throttle += g_ui32ThrottleIncrement;
                sThrottle.fAirMtr3Throttle -= g_ui32ThrottleIncrement;
#else
                sThrottle.fAirMtr1Throttle += g_ui32ThrottleIncrement;
                sThrottle.fAirMtr3Throttle -= g_ui32ThrottleIncrement;
                sThrottle.fAirMtr4Throttle -= g_ui32ThrottleIncrement;
                sThrottle.fAirMtr6Throttle += g_ui32ThrottleIncrement;
#endif
                if (g_PrintFlag) {
                    UARTprintf("Pos Pitch and Pull Up\r\n");
                }
            }
            //
            // Pitch is greater than desired and positive.
            else if ((sStatus.fPitch > fDesiredPitch)
                    && (sStatus.fPitch > 0)) {
#if APOPHIS
                //
                // "Pull Down", decrease front motor throttle and increase back motor throttle.
                sThrottle.fAirMtr1Throttle -= g_ui32ThrottleIncrement;
                sThrottle.fAirMtr3Throttle += g_ui32ThrottleIncrement;
#else
                sThrottle.fAirMtr1Throttle -= g_ui32ThrottleIncrement;
                sThrottle.fAirMtr3Throttle += g_ui32ThrottleIncrement;
                sThrottle.fAirMtr4Throttle += g_ui32ThrottleIncrement;
                sThrottle.fAirMtr6Throttle -= g_ui32ThrottleIncrement;
#endif
                if (g_PrintFlag) {
                    UARTprintf("Pos Pitch and Pull Down\r\n");
                }
            }
        }

        //
        // Check if roll error is greater than 0.5 degrees.
        if ((sStatus.fRoll - fDesiredRoll > 0.5f)
                || (sStatus.fRoll - fDesiredRoll < -0.5f)) {
            //
            // Check the roll.
            // Roll is less than desired and negative.
            if ((sStatus.fRoll < fDesiredRoll) && (sStatus.fRoll < 0)) {
#if APOPHIS
                //
                // "Roll right", increase left motor throttle and decrease right motor throttle.
                sThrottle.fAirMtr2Throttle -= g_ui32ThrottleIncrement;
                sThrottle.fAirMtr4Throttle += g_ui32ThrottleIncrement;
#else
                sThrottle.fAirMtr2Throttle -= g_ui32ThrottleIncrement;
                sThrottle.fAirMtr5Throttle += g_ui32ThrottleIncrement;
#endif
                if (g_PrintFlag) {
                    UARTprintf("Neg Roll and Roll Right\r\n");
                }
            }
            //
            // Roll is greater than desired and negative.
            else if ((sStatus.fRoll > fDesiredRoll)
                    && (sStatus.fRoll < 0)) {
#if APOPHIS
                //
                // "Roll Left", increase right motor throttle and decrease left motor throttle.
                sThrottle.fAirMtr2Throttle += g_ui32ThrottleIncrement;
                sThrottle.fAirMtr4Throttle -= g_ui32ThrottleIncrement;
#else
                sThrottle.fAirMtr2Throttle += g_ui32ThrottleIncrement;
                sThrottle.fAirMtr5Throttle -= g_ui32ThrottleIncrement;
#endif
                if (g_PrintFlag) {
                    UARTprintf("Neg Roll and Roll Left\r\n");
                }
            }
            //
            // Roll is less than desired and positive.
            else if ((sStatus.fRoll < fDesiredRoll)
                    && (sStatus.fRoll > 0)) {
#if APOPHIS
                //
                // "Roll Right", increase left motor throttle and decrease right motor throttle.
                sThrottle.fAirMtr2Throttle -= g_ui32ThrottleIncrement;
                sThrottle.fAirMtr4Throttle += g_ui32ThrottleIncrement;
#else
                sThrottle.fAirMtr2Throttle -= g_ui32ThrottleIncrement;
                sThrottle.fAirMtr5Throttle += g_ui32ThrottleIncrement;
#endif
                if (g_PrintFlag) {
                    UARTprintf("Pos Roll and Roll Right\r\n");
                }
            }

            //
            // Roll is greater than desired and positive.
            else if ((sStatus.fRoll > fDesiredRoll)
                    && (sStatus.fRoll > 0)) {
#if APOPHIS
                //
                // "Roll Left", decrease left motor throttle and increase right motor throttle.
                sThrottle.fAirMtr2Throttle += g_ui32ThrottleIncrement;
                sThrottle.fAirMtr4Throttle -= g_ui32ThrottleIncrement;
#else
                sThrottle.fAirMtr2Throttle += g_ui32ThrottleIncrement;
                sThrottle.fAirMtr5Throttle -= g_ui32ThrottleIncrement;
#endif
                if (g_PrintFlag) {
                    UARTprintf("Pos Roll and Roll Left\r\n");
                }
            }

            //
            // TODO: Check yaw calculations.
            if (i32Yaw == 1) {
                //
                // User is pressing right bumper. Rotate right (clockwise from above).
                sThrottle.fAirMtr1Throttle += 5 * g_ui32ThrottleIncrement;
                sThrottle.fAirMtr3Throttle += 5 * g_ui32ThrottleIncrement;
                sThrottle.fAirMtr2Throttle -= 5 * g_ui32ThrottleIncrement;
                sThrottle.fAirMtr4Throttle -= 5 * g_ui32ThrottleIncrement;

            } else if (i32Yaw == -1) {
                //
                // User is pressing left bumper. Rotate left (counter-clockwise from above).
                sThrottle.fAirMtr1Throttle -= 5 * g_ui32ThrottleIncrement;
                sThrottle.fAirMtr3Throttle -= 5 * g_ui32ThrottleIncrement;
                sThrottle.fAirMtr2Throttle += 5 * g_ui32ThrottleIncrement;
                sThrottle.fAirMtr4Throttle += 5 * g_ui32ThrottleIncrement;
            }
        }
    }

    //
    // Set the new throttles for the motors.
    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, sThrottle.fAirMtr1Throttle);
    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, sThrottle.fAirMtr2Throttle);
    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, sThrottle.fAirMtr3Throttle);
    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, sThrottle.fAirMtr4Throttle);
    */
}

//*****************************************************************************
//
// Radio interrupt handler as created by Ryan Claus.
//
//*****************************************************************************
void RadioIntHandler2(void) { /*
	static uint8_t ui8Index = 0;
	static uint8_t ui8Magic[40] = { 0 };
	static uint8_t ui8MagicCount;
	static bool bValidData = false;
	static int32_t i32RxChar;

	//
	// Get the interrupt status and clear the associated interrupt.
	uint32_t ui32Status = UARTIntStatus(RADIO_UART, true);
	UARTIntClear(RADIO_UART, ui32Status);

	//
	// Get the character received and send it to the console.
	while (UARTCharsAvail(RADIO_UART)) {
		i32RxChar = UARTCharGetNonBlocking(RADIO_UART);
		if (ui8Index >= (sizeof(uRxPack)))
			ui8Index = 0;
		if (i32RxChar != -1) {
			if (bValidData) {
				//
				// Get the chars over the UART.
				g_sRxPack.ui8Data[ui8Index++] = (uint8_t) i32RxChar;
				if (((g_sRxPack.ui8Data[3] == 'T' || g_sRxPack.ui8Data[3] == '0' || g_sRxPack.ui8Data[3] == 'A' || g_sRxPack.ui8Data[3] == 'D') && ui8Index >= sizeof(tGSTPacket))
						|| (g_sRxPack.ui8Data[3] == 'C'
								&& ui8Index >= sizeof(tGSCPacket))) {
					ui8Index = 0;
					bValidData = false;

					//
					// Good radio connection. Reset the timer and set the status.
					sStatus.bRadioConnected = true;
					TimerLoadSet(RADIO_TIMER_CHECK, TIMER_A,
							16000000 / GS_RADIO_RATE);

					//
					// Process the radio commands.
					ProcessRadio();

					break;
				}
			} else {
				ui8Magic[ui8Index] = (uint8_t) i32RxChar;
				ui8Index = (ui8Index + 1) % 4;
				if (ui8MagicCount >= 3) {
					if (ui8Magic[ui8Index % 4] == 0xFF
							&& ui8Magic[(ui8Index + 1) % 4] == 0xFF
							&& ui8Magic[(ui8Index + 2) % 4] == 0xFF
							&& (ui8Magic[(ui8Index + 3) % 4] == 'T'
									|| ui8Magic[(ui8Index + 3) % 4] == 'C'
									|| ui8Magic[(ui8Index + 3) % 4] == '0'
									|| ui8Magic[(ui8Index +3) % 4] == 'A'
									|| ui8Magic[(ui8Index + 3) % 4] == 'D')) {
						g_sRxPack.ui8Data[3] = ui8Magic[(ui8Index + 3) % 4];
						ui8Index = 4;
						bValidData = true;
						ui8MagicCount = 0;
					}
				} else {
					ui8MagicCount++;
				}
			}
		}
	}
*/}



