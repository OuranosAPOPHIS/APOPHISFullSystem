/*
 * Project: Aerial Platform for Overland Haul and Import System (APOPHIS)
 *
 *  Created On: Jan 20, 2017
 *  Last Updated: April 18, 2017
 *      Author(s): Brandon Klefman
 *
 *      Purpose: Flight computer for the APOPHIS platform.
 *
 */

//*****************************************************************************
//
// Includes
//
//*****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "inc/hw_memmap.h"

#include "driverlib/adc.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "sensorlib/comp_dcm.h"

#include "misc/buttons.h"

#include "utils/uartstdio.h"

#include "initializations.h"
#include "APOPHIS_pin_map.h"
#include "master_defines.h"
#include "packet_format.h"

#include "sensors/bmi160.h"
#include "sensors/i2c_driver.h"
#include "sensors/bme280.h"

#include "motors/vehicle_properties.h"

//
// Sensor-Boosterpack Calibration data.
#if BOOSTERPACK1
#include "sensors/accel_gyro_cal_data1.h"
#elif BOOSTERPACK2
#include "sensors/accel_gyro_cal_data2.h"
#elif BOOSTERPACK3
#include "sensors/accel_gyro_cal_data3.h"
#endif

#include "motors/gnd_mtrs.h"

//*****************************************************************************
//
// Defines
//
//*****************************************************************************
//
// Time step for the control law.
#define DT 0.01

//
// LSB conversions for +/-2g accel, 125 deg/s gyro and mag settings.
#define GYROLSB 262.4f
#define ACCELLSB 16384
#define MAGLSB 16.0f

#if IMU_ACTIVATED
#define STABBIAS true
#else
#define STABBIAS false
#endif

//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
void SysTickIntHandler(void);
void ConsoleIntHandler(void);
void RadioIntHandler(void);
void GPSIntHandler(void);
void Timer2AInterrupt(void);
void Timer2BInterrupt(void);
void Timer3AInterrupt(void);
void Timer3BInterrupt(void);
void Timer1AInterrupt(void);
void Timer1BInterrupt(void);
void SolenoidInterrupt(void);
void BMI160IntHandler(void);
void BME280IntHandler(void);
void RadioTimeoutIntHandler(void);
void TurnOnLED(uint32_t LEDNum);
void TurnOffLED(uint32_t LEDNum);
void Menu(char CharReceived);
void ProcessGPS(void);
void ProcessRadio(void);
void ProcessADC(void);
int ProcessUltraSonic(uint32_t SysClockSpeed);
void DeployPayload(void);
void SendPacket(void);
void ProcessIMUData(void);
void ProcessBME280(void);
bool WaitForButtonPress(uint8_t desiredButtonState);
void WaitForArming(void);

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************

bool bValidData = false;
uint8_t ui8Magic[4] = { 0 };
uint8_t ui8Index = 0;
uint32_t MaxIndex = 0;

//*****************************************************************************
//
// System State Structures.
//
//*****************************************************************************
//
// State of the system structure definition and variable.
typedef struct {
	uint8_t ui8Mode;    // Mode of operation. 'D' - drive, 'F' - fly, 'A' - autonomous
	bool bPayDeployed;  // True indicates the payload has already been deployed.
	bool bRadioConnected; // True indicates radio is connected.
	bool bTargetSet;    // True indicates good radio data.
	float fRoll;		// Actual platform roll (degrees).
	float fPitch;		// Actual platform pitch (degrees).
	float fYaw;			// Actual platform yaw (degrees).
	float fDesRoll;     // Desired platform roll (degrees).
	float fDesPitch;    // Desired platform pitch (degrees).
	int32_t i32DesYaw; // Desired direction of yaw. (+1 CW or -1 CCW rotation).
	float fMass;			// Actual platform mass (kg).
	float fMatI;			// Mass Moment of Inertia of the Platform
	float fCurrentLat;		// Current Latitide.
	float fCurrentLong;		// Current longitude.
	float fCurrentAlt;		// Current Altitude.
	float fTargetLat;		// Target latitude.
	float fTargetLong;		// Target longiutde.
	float fTargetAlt;		// Target Altitude.
	float fTempTargetLat;// Temporary target latitude for before the location is set by the GS.
	float fTempTargetLong; // Temporary target longitude for before the location is set by the GS.
	bool bArmed;			// boolean armed indicator. true means the system is armed. False is disarmed.
} SystemStatus;

//
// Structure for all of the sensor values. 
typedef struct {
	float fCurrentAccelX;		// Current Acceleration in X direction
	float fCurrentAccelY;		// Current Acceleration in Y direction
	float fCurrentAccelZ;		// Current Acceleration in Z direction
	float fPreviousAccelX;
	float fPreviousAccelY;
	float fPreviousAccelZ;
	float fPrevVelX;
	float fPrevVelY;
	float fPrevVelZ;
	float fPrevPosX;
	float fPrevPosY;
	float fPrevPosZ;
	float fCurrentGyroX;
	float fCurrentGyroY;
	float fCurrentGyroZ;
	float fThrust;
} SensorStatus;

//
// Structure for all of the motor throttles.
typedef struct {
	uint16_t ui16GndMtrLWThrottle;
	uint16_t ui16GndMtrRWThrottle;
	int32_t i32RWThrottlePerc;
	int32_t i32LWThrottlePerc;
	uint32_t ui32AirMtrThrottle;
	uint32_t ui32AirMtr1Throttle;
	uint32_t ui32AirMtr2Throttle;
	uint32_t ui32AirMtr3Throttle;
	uint32_t ui32AirMtr4Throttle;
} SystemThrottle;

//
// System throttle structure.
SystemThrottle sThrottle;
//
// Initialize the state of the system.
SystemStatus sStatus;

//
// Initialize the sensor state.
SensorStatus sSensStatus;

//
// Throttle values for the air motors.
uint32_t g_ui32ThrottleIncrement = 0;

//*****************************************************************************
//
// IMU global variables.
//
//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
tCompDCM g_sCompDCMInst;
bool g_bDCMStarted;

//
// Flag to indicate when to process IMU data.
bool g_IMUDataFlag = false;

//
// Offset compensation data for the accel and gyro.
uint8_t g_offsetData[7] = { 0 };

//
// Calculated bias for mag from MATLAB in uTeslas.
int16_t g_MagBias[3] = { 0 }; //{ 23.2604390452978, 4.40368720486817, 41.9678519105233 };

//
// Global storage for the gyro data and gyro bias.
float g_fGyroData[3];
int16_t g_GyroStabBias[3] = { 0 };

//
// Used as global storage for the raw mag data.
float g_fMagData[3];

//
// Used to indicate if the IMU is working, by blinking LED 1.
bool g_LED1On = false;

//
// Used to indicate when to print the accel and gyro values to the PC.
// Set to once per second. Triggered every second by the SysTickIntHandler()
bool g_PrintIMUData = false;

//
// Variable to determine whether to print the raw accel and gyro data to the terminal.
bool g_PrintRawBMIData = false;

//*****************************************************************************
//
// Radio packet global variables.
//
//*****************************************************************************
//
// Radio packet to be sent to the ground station.
uTxPack g_Pack;

//
// Radio packet to be received from ground station.
uRxPack g_sRxPack;

//*****************************************************************************
//
// Misc global variables.
//
//*****************************************************************************
//
// LED tracker.
bool g_LEDON = false;

//
// Counter for Systick printing at 2 Hz instead of 12 Hz.
uint32_t g_SysTickCount = 0;

//
// Variable to store when USER LED 4 is on.
bool g_LED4On = false;

//
// Variable to store characters received from the PC.
char g_CharConsole;

//
// Variable to trigger evaluation of received character from PC in the main
// function.
bool g_ConsoleFlag = false;

//
// Variable to indicate end of program.
bool g_Quit = false;

//
// Variable to determine when to evaluate the ADC results.
bool g_ADCFlag = false;

//
// Flag to indicate when to print for the trajectory information.
bool g_PrintFlag = false;

//*****************************************************************************
//
// GPS global variables.
//
//*****************************************************************************
//
// Variable to indicate when GPS data is available.
bool g_GPSFlag = false;

//
// Variable to indicate when LED 2 is turned on. And when GPS
// has collected good data.
bool g_GPSLEDOn = false;
bool g_GPSConnected = false;

//
// Variable to determine whether to print the raw GPS data to the terminal.
bool g_PrintRawGPS = false;

//*****************************************************************************
//
// Ultrasonic Sensor global variables.
//
//*****************************************************************************
//
// Variable to indicate first or last pass through timer interrupt.
bool g_TimerFirstPass = true;

//
// Variable to record falling and rising edge values of timer.
uint32_t g_TimerRiseValue;
uint32_t g_TimerFallValue;

//
// Variable to signal when to evaluate the timer values for the ultrasonic sensor.
bool g_UltraSonicFlag = false;

//
// Variable to represent which sensor is currently being activated. If it is
// 0, then no sensors are being activated. This value will range from 0 - 6.
uint8_t g_UltraSonicSensor = 0;

//*****************************************************************************
//
// Environmental Sensor global variables.
//
//*****************************************************************************
//
// BME280 flag for triggered data analysis in main().
bool g_BME280Ready = false;

//
// Storage for the BME280 raw data and its pointer.
int8_t g_BME280RawData[7];
int8_t *g_ptrBME280RawData = &g_BME280RawData[0];

//
// Offset values from BME280 registers.
int8_t g_BME280OffsetValues[12];
uint8_t g_BME280OffsetUnsigned[2];

//
// Floating format for pressure and temperature data.
float g_fPressure;
float g_fTemp;

//
// global temperature fine value for BME280;
int32_t g_t_fine;
int32_t *g_p_t_fine = &g_t_fine;

//*****************************************************************************
//
// Servo global variables.
//
//*****************************************************************************
//
// Starting and ending position of the deployment servo.
uint32_t g_ui32ServoStartPosition = 0;
uint32_t g_ui32ServoEndPosition = 0;

uint32_t g_ui32ServoAngle = 0;

bool g_RadioDone = false;

//*****************************************************************************
//
// Start of program.
//
//*****************************************************************************
int main(void) {

#if PAYLOAD_DEPLOY
	uint32_t ui32ServoSpeed = 0;
#endif

#if STABBIAS
	//
	// Values for the gyro stability bias calculation.
    int numCalcs = 0;
    int index, j;
    float fSum[3] = { 0 };
    float fbias[3][50] = { 0 };
#endif

	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	FPUEnable();
	FPULazyStackingEnable();

	//
	// Set the clocking to run at 120 MHz.
#if SPEEDIS120MHZ
	SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
	SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);
#else
	//
	// Set the clocking to run at 16 MHz.
	SysCtlClockFreqSet(SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ, 16000000);
#endif

	//
	// Disable interrupts during initialization period.
	IntMasterDisable();

	//
	// Before doing anything, initialize the LED.
	InitLED();

	//
	// Turn off all LEDs, in case one was left on.
	TurnOffLED(5);

	//
	// Initialization has begun. Turn on LED 1.
	TurnOnLED(1);

	//
	// Initialize the buttons.
	ButtonsInit();

	//
	// Initialize the system state.
    sStatus.bPayDeployed = false;
    sStatus.bRadioConnected = false;
    sStatus.bTargetSet = false;
    sStatus.fMass = 16;
    sStatus.bArmed = false;

    //
    // Initialize the sensor states.
    sSensStatus.fPrevVelX = 0;
    sSensStatus.fPrevVelY = 0;
    sSensStatus.fPrevVelZ = 0;
    sSensStatus.fPrevPosX = 0;
    sSensStatus.fPrevPosY = 0;
    sSensStatus.fPrevPosZ = 0;
    sSensStatus.fThrust = 0;

    //
    // Initialize the throttle of the system.
    sThrottle.ui32AirMtr1Throttle = ZEROTHROTTLE;
    sThrottle.ui32AirMtr2Throttle = ZEROTHROTTLE;
    sThrottle.ui32AirMtr3Throttle = ZEROTHROTTLE;
    sThrottle.ui32AirMtr4Throttle = ZEROTHROTTLE;

    sThrottle.ui16GndMtrRWThrottle = 0x0000;
    sThrottle.ui16GndMtrLWThrottle = 0x0000;

    //
    // Set the magic packets.
    g_Pack.pack.Magic[0] = 0xFF;
    g_Pack.pack.Magic[1] = 0xFF;
    g_Pack.pack.Magic[2] = 0xFF;

	//
	// Initialize the payload deployment pins if turned on.
#if PAYLOAD_DEPLOY
	//
	// Initialize the air motors.
	ui32ServoSpeed = InitServoMtrs();

	//
	// Calculate zero position corresponding to a 2.5% duty cycle.
	g_ui32ServoStartPosition = (uint32_t)(ui32ServoSpeed * 0.025);

	//
	// Calculate end position corresponding to a 12.5% duty cycle.
	g_ui32ServoEndPosition = g_ui32ServoStartPosition * 5;

	//
	// Initialize the angle of the system to the starting position.
	g_ui32ServoAngle = g_ui32ServoStartPosition;
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);
#endif

    //
    // Initialize the Console if debug mode is on.
#if DEBUG
    InitConsole();
    UARTprintf("Clock speed: %d\r\n", SYSCLOCKSPEED);
#endif

    //
    // Initialize the GPS if turned on.
#if GPS_ACTIVATED
    InitGPS();
#endif

    //
    // Initialize the solar panels if turned on.
#if SECONDARY_ATTITUDE
    InitSolarPanels();

    InitSecondaryAccel();
#endif

    //
    // Initialize the ultrasonic sensors if turned on.
#if ULTRASONIC_ACTIVATED
    InitUltraSonicSensor();
#endif

    //
    // Initialize the pressure sensor if enabled.
#if ALTIMETER_ACTIVATED
    //
    // Initialize the altimeter.
    InitAltimeter(g_BME280OffsetValues);

    //
    // Set the offset values.
    g_BME280OffsetUnsigned[0] = g_BME280OffsetValues[0];
    g_BME280OffsetUnsigned[1] = g_BME280OffsetValues[3];
#endif

   //
   // Initialize the BMI160 IMU if enabled.
#if IMU_ACTIVATED
   InitIMU(g_offsetData);

#if STABBIAS
   //
   // Calculate the gyro bias.
   while (numCalcs < 50) {
       uint8_t status;
       uint8_t IMUData[6] = { 0 };
       int16_t i16GyroData[3];
       float fGyroDataUnCal[3];

       //
       // First check the status for which data is ready.
       I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_STATUS, 1, &status);

       //
       // Check what status returned.
       if ((status & 0xC0) == (BMI160_GYR_RDY | BMI160_ACC_RDY)) {
           //
           // Then get the data for the gyro.
           I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_GYRO_X, 6, IMUData);

           //
           // Set the gyro data.
           i16GyroData[0] = (((int16_t) IMUData[1] << 8) + (int8_t) IMUData[0]);
           i16GyroData[1] = (((int16_t) IMUData[3] << 8) + (int8_t) IMUData[2]);
           i16GyroData[2] = (((int16_t) IMUData[5] << 8) + (int8_t) IMUData[4]);

           //
           // Convert gyro data to float.
           fGyroDataUnCal[0] = (((float) (i16GyroData[0])) / GYROLSB) - BGX;
           fGyroDataUnCal[1] = (((float) (i16GyroData[1])) / GYROLSB) - BGY;
           fGyroDataUnCal[2] = (((float) (i16GyroData[2])) / GYROLSB) - BGZ;

           //
           // Calculate the calibrated gyro data.
           fbias[0][numCalcs] = fGyroDataUnCal[0] * SGX + fGyroDataUnCal[1] * MGXY + fGyroDataUnCal[2] * MGXZ;
           fbias[1][numCalcs] = fGyroDataUnCal[0] * MGYX + fGyroDataUnCal[1] * SGY + fGyroDataUnCal[2] * MGYZ;
           fbias[2][numCalcs] = fGyroDataUnCal[0] * MGZX + fGyroDataUnCal[1] * MGZY + fGyroDataUnCal[2] * SGZ;

           numCalcs++;
       }
   }

   //
   // Calculate the bias.
   for (index = 0; index < numCalcs; index++)
       for (j = 0; j < 3; j++)
           fSum[j] += fbias[j][index];

   for (index = 0; index < 3; index++)
       g_GyroStabBias[index] = fSum[index] / numCalcs;

   IntMasterDisable();
#endif

   //
   // Initialize the DCM.
   CompDCMInit(&g_sCompDCMInst, 1.0f / DCM_UPDATE_RATE, 0.2f, 0.6f, 0.2f);
   g_bDCMStarted = false;
#endif

   //
   // Initialize the radio if turned on.
#if RADIO_ACTIVATED
   InitRadio();

   //
   // Activate the radio timers.
   TimerEnable(RADIO_TIMER, TIMER_A);
   TimerEnable(RADIO_TIMER_CHECK, TIMER_A);
#endif

   //
   // Turn on interrupts so the sensors and radio can start working.
   IntMasterEnable();

   //
   // All sensors are activated...
   // Wait to finish boot up process before initializing any motors.
   while (!WaitForButtonPress(RIGHT_BUTTON))
       { ;;; }

	//
	// Initialize the ground motors if turned on.
#if GNDMTRS_ACTIVATED
	InitGndMotors();
#endif

	//
	// Initialize the air motors if activated.
#if AIRMTRS_ACTIVATED
	//
	// Initialize the air motors.
	InitAirMtrs(ZEROTHROTTLE);

	//
	// Calculate a throttle increment.
	g_ui32ThrottleIncrement = (MAXTHROTTLE - ZEROTHROTTLE) / 100;
#endif

	//
	// Initialize more system states based on motor information.
#if	GNDMTRS_ACTIVATED
	sStatus.ui8Mode = 'D';
#elif AIRMTRS_ACTIVATED
	sStatus.ui8Mode = 'F';
#else
	sStatus.ui8Mode = 'A';
#endif

#if DEBUG
	//
	// Before starting program, wait for a button press on either switch.
	UARTprintf("Initialization Complete!\r\nPress left button to start or\r\n");
	UARTprintf("click the 'Arm System' button on the GUI.\n\r");
#endif

	//
	// Wait to start arming everything until the user arms the system.
	WaitForArming();

	TurnOffLED(5);

	//
	// Turn off LED1, and enable the systick at 25 Hz to
	// update the trajectory and blink LED 4, signifying regular operation.
	// The Systick cannot handle any value larger than 16MHz.
	TurnOffLED(1);
	SysTickPeriodSet(SYSCLOCKSPEED / UPDATE_TRAJECTORY_RATE);
	SysTickIntRegister(SysTickIntHandler);
	SysTickIntEnable();

	//
	// Print menu.
	Menu('M');

	//
	// Program start.
	while (!g_Quit) {
		//
		// First check for commands from Console.
		if (g_ConsoleFlag)
			Menu(g_CharConsole);

		//
		// Now check if GPS data is ready.
		if (g_GPSFlag)
			ProcessGPS();

		//
		// Check if ADC is finished.
		if (g_ADCFlag)
			ProcessADC();

		//
		// Check if Ultrasonic is done.
		if (g_UltraSonicFlag)
			ProcessUltraSonic(SYSCLOCKSPEED);

		//
		// Check if accel or gyro data is ready.
		if (g_IMUDataFlag)
			ProcessIMUData();

		//
		// Check if pressure or temperature data is ready.
		if (g_BME280Ready)
			ProcessBME280();

		//
		// Check if the system was disarmed.
		if (!sStatus.bArmed)
			WaitForArming();

		if (g_RadioDone)
		{
		    int n;
		    for (n = 0; n < sizeof(g_sRxPack.ui8Data); n++)
		        UARTCharPut(CONSOLE_UART, g_sRxPack.ui8Data[n]);

		    g_RadioDone = false;
		}
	}

#if DEBUG
	//
	// Program ending. Do any clean up that's needed.
	UARTprintf("Goodbye!\r\n");
#endif

	//
	// Disarm system and disable all motors.
    IntMasterDisable();

#if GNDMTRS_ACTIVATED
    //
    // Set the ground motors to zero throttle.
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
#endif

#if AIRMTRS_ACTIVATED
    //
    // Set the air motors to zero throttle and disable the generators.
    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, ZEROTHROTTLE);
    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, ZEROTHROTTLE);
    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, ZEROTHROTTLE);
    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, ZEROTHROTTLE);

    PWMGenDisable(PWM0_BASE, PWM_GEN_0);
    PWMGenDisable(PWM0_BASE, PWM_GEN_1);
    PWMGenDisable(PWM0_BASE, PWM_GEN_2);
#endif

    //
    // Turn off all LEDs except for the first and fourth
    // to indicate program completion.
	TurnOffLED(5);
	TurnOnLED(1);
	TurnOnLED(4);

	return 0;
}

/*
 * Interrupt handlers go here:
 */
//*****************************************************************************
//
// SysTick Interrupt handler. Blinks LED 4 at a constant rate of 1 Hz in order
// to indicate program operation.
//
//*****************************************************************************
void SysTickIntHandler(void) {
	switch (sStatus.ui8Mode)
	{
		case 'D': { // Driving
#if GNDMTRS_ACTIVATED
		uint8_t txBuffer[4];

		//
		// Check the direction. For the LW, CCW is the forward direction.
		if (sThrottle.i32LWThrottlePerc >= 0) {
			//
			// Rotate the wheel in the CCW direction. (0 - 1023)
			sThrottle.ui16GndMtrLWThrottle = RX24_THROTTLE_INCREMENT
					* sThrottle.i32LWThrottlePerc;
		} else if (sThrottle.i32LWThrottlePerc < 0) {
			//
			// Rotate the wheel in the CW direction. (1024 - 2047).
			sThrottle.ui16GndMtrLWThrottle = RX24_THROTTLE_INCREMENT
					* (-sThrottle.i32LWThrottlePerc) + 1024;
		}

		//
		// Check the RW direction. CW is forward direction.
		if (sThrottle.i32RWThrottlePerc >= 0) {
			//
			// Rotate the wheel in the CW direction. (1024 - 2047)
			sThrottle.ui16GndMtrRWThrottle = RX24_THROTTLE_INCREMENT
					* sThrottle.i32RWThrottlePerc + 1024;
		} else if (sThrottle.i32RWThrottlePerc < 0) {
			//
			// Rotate the wheel in the CCW direction. (0 - 1023).
			sThrottle.ui16GndMtrRWThrottle = RX24_THROTTLE_INCREMENT
					* (-sThrottle.i32RWThrottlePerc);
		}

		//
		// Build the LW instruction packet.
		txBuffer[0] = RX24_WRITE_DATA;
		txBuffer[1] = RX24_REG_MOVING_VEL_LSB;
		txBuffer[2] = (uint8_t) (sThrottle.ui16GndMtrLWThrottle & 0x00ff);
		txBuffer[3] = (uint8_t) (sThrottle.ui16GndMtrLWThrottle >> 8);

		//
		// Send the command to the LW motor.
		Rx24FWrite(GNDMTR1_UART, GNDMTR1_DIRECTION_PORT, GMDMTR1_DIRECTION, 4,
				txBuffer);

		//
		// Build the RW instruction packet.
		txBuffer[2] = (uint8_t) (sThrottle.ui16GndMtrRWThrottle & 0x00ff);
		txBuffer[3] = (uint8_t) (sThrottle.ui16GndMtrRWThrottle >> 8);

		//
		// Send the command to the RW motor.
		Rx24FWrite(GNDMTR2_UART, GNDMTR2_DIRECTION_PORT, GMDMTR2_DIRECTION, 4,
				txBuffer);

#if DEBUG
		if (g_PrintFlag)
		{
			UARTprintf("Driving.\r\n");
			UARTprintf("RW Throttle: %d\r\nLW Throttle: %d\r\n", g_ui32RWThrottle,
					g_ui32LWThrottle);
		}
#endif
#endif
		break;
	}
		case 'F': { // Flying
#if AIRMTRS_ACTIVATED
			float fPitchError, fRollError, fPitchDotDot, fRollDotDot;
			float fPitchDot, fRollDot;
            float fMx, fMy, fMz, fFz;
            float fKd = 30;
            float fKp = 10;
            float fTh1, fTh2, fTh3, fTh4;

            //
            // Pitch and roll errors.
            fPitchError = sStatus.fDesPitch - sStatus.fPitch;
            fRollError = sStatus.fDesRoll - sStatus.fRoll;

            //
            // Figure out the yaw rate.
            if (sStatus.i32DesYaw == 1)
                //
                // Rotate Clockwise.
                fMz = IYY * 10; // (degrees/s/s)
            else if (sStatus.i32DesYaw == -1)
                //
                // Rotate CCW.
                fMz = IYY * -10; // (degrees/s/s)
            else
                //
                // No yaw rotation.
                fMz = 0;

            //
            // Calculate Fz based on the throttle command.
            fFz = (float)sThrottle.ui32AirMtrThrottle;

            //
            // Desired angular velocity.
            fPitchDot = fPitchError * fKd;
            fRollDot = fRollError * fKp;

            //
            // Desired angular accelerations.
            fPitchDotDot = fPitchDot * fKp;
            fRollDotDot = fRollDot * fKp;

            //
            // Calculate Mx, My for desired thrusts.
            fMx = IXX * fPitchDotDot;
            fMy = IYY * fRollDotDot;

            //
            // Desired thrusts based on desired angular acceleration
            // and throttle values.
            fTh1 = B11 * fMx + B12 * fMy + B13 * fMz + B14 * fFz;
            fTh2 = B21 * fMx + B22 * fMy + B23 * fMz + B24 * fFz;
            fTh3 = B31 * fMx + B32 * fMy + B33 * fMz + B34 * fFz;
            fTh4 = B41 * fMx + B42 * fMy + B43 * fMz + B44 * fFz;

            //
            // Convert from a force to a commanded value.
            sThrottle.ui32AirMtr1Throttle = (uint32_t)(fTh1 * THRUST_CMD_RATIO + ZEROTHROTTLE);
            sThrottle.ui32AirMtr2Throttle = (uint32_t)(fTh2 * THRUST_CMD_RATIO + ZEROTHROTTLE);
            sThrottle.ui32AirMtr3Throttle = (uint32_t)(fTh3 * THRUST_CMD_RATIO + ZEROTHROTTLE);
            sThrottle.ui32AirMtr4Throttle = (uint32_t)(fTh4 * THRUST_CMD_RATIO + ZEROTHROTTLE);

            //
            // Send the commands to the motors.
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, sThrottle.ui32AirMtr1Throttle);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, sThrottle.ui32AirMtr2Throttle);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, sThrottle.ui32AirMtr3Throttle);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, sThrottle.ui32AirMtr4Throttle);
#endif
            break;
		}
		case 'A': { // Autonomous - lost radio connection or something.
#if GNDMTRS_ACTIVATED
			uint8_t txBuffer[4];

            // Build the instruction packet.
            txBuffer[0] = RX24_WRITE_DATA;
            txBuffer[1] = RX24_REG_MOVING_VEL_LSB;
            txBuffer[2] = 0x00;
            txBuffer[3] = 0x00;

            //
            // Send the command to the LW motor.
            Rx24FWrite(GNDMTR1_UART, GNDMTR1_DIRECTION_PORT, GMDMTR1_DIRECTION,
                       4, txBuffer);

            //
            // Send the command to the RW motor.
            Rx24FWrite(GNDMTR2_UART, GNDMTR2_DIRECTION_PORT, GMDMTR2_DIRECTION,
                       4, txBuffer);
#endif

#if AIRMTRS_ACTVIATED
            //
			// Zero out the air motors.
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, ZEROTHROTTLE);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, ZEROTHROTTLE);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, ZEROTHROTTLE);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, ZEROTHROTTLE);

            //
            // A smarter system, would do more than just zero out the throttle, in case
            // the vehicle is currently in the air, it could safely descend to the ground.
#endif
            break;
	}
	}
	
    //
    // Blink LED 4 once per second.
    if (g_SysTickCount >= (UPDATE_TRAJECTORY_RATE / 2))
    {
        if (g_LED4On)
        {
			//
			// Turn off LED 4 if it is on.
			TurnOffLED(4);

			g_LED4On = false;
		} else {
			//
			// Otherwise turn it on.
			TurnOnLED(4);

			g_LED4On = true;
		}

#if DEBUG
		//
		// Trigger printing accel and gyro data to PC terminal.
		g_PrintIMUData = true;

		//
		// Trigger printing of the trajectory information.
		g_PrintFlag = true;
#endif

		//
		// Reset SysTick Count.
		g_SysTickCount = 0;
	} else
		g_SysTickCount++;
}

//*****************************************************************************
//
// Interrupt handler for the console communication with the PC.
//
//*****************************************************************************
void ConsoleIntHandler(void) {
	//
	// First get the interrupt status. Then clear the associated interrupt flag.
	uint32_t ui32Status = UARTIntStatus(CONSOLE_UART, true);
	UARTIntClear(CONSOLE_UART, ui32Status);

	//
	// Get the character sent from the PC.
	g_CharConsole = UARTCharGetNonBlocking(CONSOLE_UART);

	//
	// Trigger the flag for char received from console.
	g_ConsoleFlag = true;
}

//*****************************************************************************
//
// Interrupt handler which handles reception of data from the radio.
//
//*****************************************************************************
void RadioIntHandler(void)
{
	int32_t i32RxChar;

	//
	// Clear the interrupt.
	uint32_t ui32Status = UARTIntStatus(RADIO_UART, true);
	UARTIntClear(RADIO_UART, ui32Status);

	while (UARTCharsAvail(RADIO_UART))
	{
		if (ui8Index > MaxIndex)
			MaxIndex = ui8Index;

		//
		// Get the character from the Radio.
		i32RxChar = UARTCharGetNonBlocking(RADIO_UART);

		//
		// Find the magic packet if the data is not valid.
		if (!bValidData) {
			ui8Magic[ui8Index % 4] = (uint8_t) i32RxChar;
			ui8Index = (ui8Index + 1) % 4;

			if ((ui8Magic[(ui8Index) % 4] == 0xFF)
					&& (ui8Magic[(ui8Index + 1) % 4] == 0xFF)
					&& (ui8Magic[(ui8Index + 2) % 4] == 0xFF)
					&& ((ui8Magic[(ui8Index + 3) % 4] == 'C')
							|| (ui8Magic[(ui8Index + 3) % 4] == 'T')
							|| (ui8Magic[(ui8Index + 3) % 4] == '0')
							|| (ui8Magic[(ui8Index + 3) % 4] == 'A')
							|| (ui8Magic[(ui8Index + 3) % 4] == 'D'))) {

				//
				// Found the magic packet.
				bValidData = true;

				g_sRxPack.ui8Data[3] = ui8Magic[(ui8Index + 3) % 4];
				ui8Index = 4;
			}
		}
		else {
			//
			// Found the magic packet. Process the remaining string.
			switch (g_sRxPack.ui8Data[3])
			{
			case 'T':
			case '0':
			case 'A':
			case 'D':
			{
				//
				// Target or Arm/Disarm packet.
				if (ui8Index < sizeof(g_sRxPack.sTargetPacket)) {
					g_sRxPack.ui8Data[ui8Index] = (uint8_t)i32RxChar;
					ui8Index++;
				}
				else {

					//
					// Finished the packet. Do the processing.
					ProcessRadio();

					//
					// Reset the globals.
					ui8Magic[0] = 0;
					ui8Index = 0;
					bValidData = false;

                    g_RadioDone = true;


				    sStatus.bRadioConnected = true;
				    TimerLoadSet(RADIO_TIMER_CHECK, TIMER_A,
				            16000000 / GS_RADIO_RATE);
				}

				break;
			}
			case 'C':
			{
				//
				// Control packet.
				if(ui8Index < sizeof(g_sRxPack.sControlPacket)) {

					g_sRxPack.ui8Data[ui8Index] = (uint8_t)i32RxChar;
					ui8Index++;
				}
				else {

					//
					// Finished the packet. Do the processing.
					ProcessRadio();

					//
					// Reset the globals.
					ui8Magic[0] = 0;
					ui8Index = 0;
					bValidData = false;

				}
				break;
			}
			}
		}
	}
}

//*****************************************************************************
//
// Interrupt handler which notifies main program when GPS data is available.
//
//*****************************************************************************
void GPSIntHandler(void) {
	//
	// Get the interrupt status and clear the associated interrupt.
	uint32_t ui32Status = UARTIntStatus(GPS_UART, true);
	UARTIntClear(GPS_UART, ui32Status);

	//
	// Signal to main() that GPS data is ready to be retrieved.
	g_GPSFlag = true;
}

//*****************************************************************************
//
// Interrupt handler for Solar Panel ADC.
//
//*****************************************************************************
void SPIntHandler(void) {

	uint32_t ADCStatus = ADCIntStatus(SP_ADC, 0, true);

	//
	// First, clear the interrupt flag.
	ADCIntClear(SP_ADC, 0);

	//
	// Trigger the flag for main's evaluation.
	g_ADCFlag = true;
}

//*****************************************************************************
//
// Interrupt handler for timer capture pin T2CC0 on PM0. This interrupt will
// be called twice. Once for each edge of the echo pin on the ultrasonic
// sensor. This handler must be smart enough to handle both cases.
//
//*****************************************************************************
void Timer2AInterrupt(void) {
	uint32_t timerStatus;

	//
	// Get the timer interrupt status and clear the interrupt.
	timerStatus = TimerIntStatus(USONIC_TIMER2, true);
	TimerIntClear(USONIC_TIMER2, timerStatus);

	//
	// Check if this is the rising or falling edge.
	if (g_TimerFirstPass) {
		//
		// This is the first time through, it is a rising edge.
		g_TimerRiseValue = TimerValueGet(USONIC_TIMER2, TIMER_A);

		//
		// Set first pass to false.
		g_TimerFirstPass = false;
	} else {
		//
		// Then it is a falling edge.
		g_TimerFallValue = TimerValueGet(USONIC_TIMER2, TIMER_A);

		//
		// Reset timer pass flag.
		g_TimerFirstPass = true;

		//
		// Trigger evaluation in main().
		g_UltraSonicFlag = true;
	}
}

//*****************************************************************************
//
// Interrupt handler for timer capture pin T2CC0 on PM0. This interrupt will
// be called twice. Once for each edge of the echo pin on the ultrasonic
// sensor. This handler must be smart enough to handle both cases.
//
//*****************************************************************************
void Timer2BInterrupt(void) {
	uint32_t timerStatus;

	//
	// Get the timer interrupt status and clear the interrupt.
	timerStatus = TimerIntStatus(USONIC_TIMER2, true);
	TimerIntClear(USONIC_TIMER2, timerStatus);

	//
	// Check if this is the rising or falling edge.
	if (g_TimerFirstPass) {
		//
		// This is the first time through, it is a rising edge.
		g_TimerRiseValue = TimerValueGet(USONIC_TIMER2, TIMER_B);

		//
		// Set first pass to false.
		g_TimerFirstPass = false;
	} else {
		//
		// Then it is a falling edge.
		g_TimerFallValue = TimerValueGet(USONIC_TIMER2, TIMER_B);

		//
		// Reset timer pass flag.
		g_TimerFirstPass = true;

		//
		// Trigger evaluation in main().
		g_UltraSonicFlag = true;
	}
}
//*****************************************************************************
//
// Interrupt handler for timer capture pin T3CC0 on PM2. This interrupt will
// be called twice. Once for each edge of the echo pin on the ultrasonic
// sensor. This handler must be smart enough to handle both cases.
//
//*****************************************************************************
void Timer3AInterrupt(void) {
	uint32_t timerStatus;

	//
	// Get the timer interrupt status and clear the interrupt.
	timerStatus = TimerIntStatus(USONIC_TIMER3, true);
	TimerIntClear(USONIC_TIMER3, timerStatus);

	//
	// Check if this is the rising or falling edge.
	if (g_TimerFirstPass) {
		//
		// This is the first time through, it is a rising edge.
		g_TimerRiseValue = TimerValueGet(USONIC_TIMER3, TIMER_A);

		//
		// Set first pass to false.
		g_TimerFirstPass = false;
	} else {
		//
		// Then it is a falling edge.
		g_TimerFallValue = TimerValueGet(USONIC_TIMER3, TIMER_A);

		//
		// Reset timer pass flag.
		g_TimerFirstPass = true;

		//
		// Trigger evaluation in main().
		g_UltraSonicFlag = true;
	}
}

//*****************************************************************************
//
// Interrupt handler for timer capture pin T3CC1 on PA7. This interrupt will
// be called twice. Once for each edge of the echo pin on the ultrasonic
// sensor. This handler must be smart enough to handle both cases.
//
//*****************************************************************************
void Timer3BInterrupt(void) {
	uint32_t timerStatus;

	//
	// Get the timer interrupt status and clear the interrupt.
	timerStatus = TimerIntStatus(USONIC_TIMER3, true);
	TimerIntClear(USONIC_TIMER3, timerStatus);

	//
	// Check if this is the rising or falling edge.
	if (g_TimerFirstPass) {
		//
		// This is the first time through, it is a rising edge.
		g_TimerRiseValue = TimerValueGet(USONIC_TIMER3, TIMER_B);

		//
		// Set first pass to false.
		g_TimerFirstPass = false;
	} else {
		//
		// Then it is a falling edge.
		g_TimerFallValue = TimerValueGet(USONIC_TIMER3, TIMER_B);

		//
		// Reset timer pass flag.
		g_TimerFirstPass = true;

		//
		// Trigger evaluation in main().
		g_UltraSonicFlag = true;
	}
}

//*****************************************************************************
//
// Interrupt handler for timer capture pin T1CC0 on PA2. This interrupt will
// be called twice. Once for each edge of the echo pin on the ultrasonic
// sensor. This handler must be smart enough to handle both cases.
//
//*****************************************************************************
void Timer1AInterrupt(void) {
	uint32_t timerStatus;

	//
	// Get the timer interrupt status and clear the interrupt.
	timerStatus = TimerIntStatus(USONIC_TIMER1, true);
	TimerIntClear(USONIC_TIMER1, timerStatus);

	//
	// Check if this is the rising or falling edge.
	if (g_TimerFirstPass) {
		//
		// This is the first time through, it is a rising edge.
		g_TimerRiseValue = TimerValueGet(USONIC_TIMER1, TIMER_A);

		//
		// Set first pass to false.
		g_TimerFirstPass = false;
	} else {
		//
		// Then it is a falling edge.
		g_TimerFallValue = TimerValueGet(USONIC_TIMER1, TIMER_A);

		//
		// Reset timer pass flag.
		g_TimerFirstPass = true;

		//
		// Trigger evaluation in main().
		g_UltraSonicFlag = true;
	}
}

//*****************************************************************************
//
// Interrupt handler for timer capture pin T1CCP1 on PA3. This interrupt will
// be called twice. Once for each edge of the echo pin on the ultrasonic
// sensor. This handler must be smart enough to handle both cases.
//
//*****************************************************************************
void Timer1BInterrupt(void) {
	uint32_t timerStatus;

	//
	// Get the timer interrupt status and clear the interrupt.
	timerStatus = TimerIntStatus(USONIC_TIMER1, true);
	TimerIntClear(USONIC_TIMER1, timerStatus);

	//
	// Check if this is the rising or falling edge.
	if (g_TimerFirstPass) {
		//
		// This is the first time through, it is a rising edge.
		g_TimerRiseValue = TimerValueGet(USONIC_TIMER1, TIMER_B);

		//
		// Set first pass to false.
		g_TimerFirstPass = false;
	} else {
		//
		// Then it is a falling edge.
		g_TimerFallValue = TimerValueGet(USONIC_TIMER1, TIMER_B);

		//
		// Reset timer pass flag.
		g_TimerFirstPass = true;

		//
		// Trigger evaluation in main().
		g_UltraSonicFlag = true;
	}
}

//*****************************************************************************
//
// Interrupt handler for the BMI160 sensor unit.
//
//*****************************************************************************
void BMI160IntHandler(void) {
	uint32_t ui32Status;

	//
	// Get the interrupt status.
	ui32Status = GPIOIntStatus(BOOST_GPIO_PORT_INT, true);

	//
	// Clear the interrupt.
	GPIOIntClear(BOOST_GPIO_PORT_INT, ui32Status);

	//
	// Check which interrupt fired.
	if (ui32Status == BOOST_GPIO_INT) {
		//
		// IMU data is ready.
		g_IMUDataFlag = true;
	}
}

//*****************************************************************************
//
// Interrupt handler for the BME280 sensor unit.
//
//*****************************************************************************
void BME280IntHandler(void) {
	uint32_t ui32Status;

	//
	// Get the interrupt status.
	ui32Status = TimerIntStatus(BME280_TIMER, true);

	//
	// Clear the interrupt.
	TimerIntClear(BME280_TIMER, ui32Status);

	//
	// Trigger the flag in main.
	g_BME280Ready = true;
}

//*****************************************************************************
//
// Radio timeout interrupt. If this is reached, then we have lost
// communication.
//
//*****************************************************************************
void RadioTimeoutIntHandler(void) {
	uint32_t ui32Status;

	//
	// Get the interrupt status.
	ui32Status = TimerIntStatus(RADIO_TIMER_CHECK, true);

	//
	// Clear the interrupt.
	TimerIntClear(RADIO_TIMER_CHECK, ui32Status);

	//
	// Set the new status of the platform.
	sStatus.bRadioConnected = false;
	sStatus.ui8Mode = 'A';
}

//*****************************************************************************
//
// MMA8452Q Interrupt Handler
//
//*****************************************************************************
void MMA8452QIntHandler(void)
{
	uint32_t ui32Status;

	//
	// Get the interrupt status.
	ui32Status = GPIOIntStatus(MMA8452Q_GPIO_INT_PORT, true);

	//
	// Clear the interrupt.
	GPIOIntClear(MMA8452Q_GPIO_INT_PORT, ui32Status);

	//
	// Check which interrupt fired.
	if (ui32Status == MMA8452Q_GPIO_INT) {
		//
		// IMU data is ready.
		g_IMUDataFlag = true;
	}
}

/*
 * Other functions used by main.
 */
//*****************************************************************************
//
// This function will turn on the associated LED.
// Parameter: LEDNum - the desired LED number to turn on.
//
//*****************************************************************************
void TurnOnLED(uint32_t LEDNum) {
	//
	// Turn on the associated LED number.
	switch (LEDNum) {
	case 1: // Turn on User LED 1
	{
		GPIOPinWrite(LED_PORT1, LED1_PIN, LED1_PIN);
		return;
	}
	case 2: // Turn on User LED 2
	{
		GPIOPinWrite(LED_PORT1, LED2_PIN, LED2_PIN);
		return;
	}
	case 3: // Turn on User LED 3
	{
		GPIOPinWrite(LED_PORT2, LED3_PIN, LED3_PIN);
		return;
	}
	case 4: // Turn on User LED 4
	{
		GPIOPinWrite(LED_PORT2, LED4_PIN, LED4_PIN);
		return;
	}
	default: // Turn on all LEDs.
	{
		GPIOPinWrite(LED_PORT1, LED1_PIN | LED2_PIN, LED1_PIN | LED2_PIN);
		GPIOPinWrite(LED_PORT2, LED3_PIN | LED4_PIN, LED3_PIN | LED4_PIN);
		return;
	}
	}
}

//*****************************************************************************
//
// This function will turn off the associated LED.
// Parameter: LEDNum - the desired LED number to turn off.
//
//*****************************************************************************
void TurnOffLED(uint32_t LEDNum) {
	//
	// Turn on the associated LED number.
	switch (LEDNum) {
	case 1: // Turn on User LED 1
	{
		GPIOPinWrite(LED_PORT1, LED1_PIN, 0x00);
		return;
	}
	case 2: // Turn on User LED 2
	{
		GPIOPinWrite(LED_PORT1, LED2_PIN, 0x00);
		return;
	}
	case 3: // Turn on User LED 3
	{
		GPIOPinWrite(LED_PORT2, LED3_PIN, 0x00);
		return;
	}
	case 4: // Turn on User LED 4
	{
		GPIOPinWrite(LED_PORT2, LED4_PIN, 0x00);
		return;
	}
	default: // Turn off all LEDs.
	{
		GPIOPinWrite(LED_PORT1, LED1_PIN | LED2_PIN, 0x00);
		GPIOPinWrite(LED_PORT2, LED3_PIN | LED4_PIN, 0x00);
		return;
	}
	}
}

//*****************************************************************************
//
// This function will stop the current program run in order to wait for a
// button press.
//
// desiredButtonState: One of three values, LEFT_BUTTON, RIGHT_BUTTON or ALL_BUTTONS.
//
//*****************************************************************************
bool WaitForButtonPress(uint8_t desiredButtonState) {
	uint8_t actualButtonState;
	uint8_t rawButtonState;
	uint8_t *pRawButtonState = &rawButtonState;
	uint8_t delta;
	uint8_t *pDelta = &delta;

	//
	// Get the state of the buttons.
	actualButtonState = ButtonsPoll(pDelta, pRawButtonState);

	//
	// Unmask the button state.

	switch (desiredButtonState)
	{
	case LEFT_BUTTON:
	{
		if (actualButtonState & LEFT_BUTTON)
			return true;

		break;
	}
	case RIGHT_BUTTON:
	{
		if (actualButtonState & RIGHT_BUTTON)
			return true;

		break;
	}
	case ALL_BUTTONS:
	{
		if (actualButtonState & ALL_BUTTONS)
			return true;

		break;
	}
	}

	return false;
}

//*****************************************************************************
//
// This function will handle analysis of characters received from the console.
//
//*****************************************************************************
void Menu(char charReceived) {
#if DEBUG
	//
	// Check the character received.
	switch (charReceived) {
	case 'M': // Print Menu.
	{
		UARTprintf("Menu:\r\nM - Print this menu.\r\n");
		UARTprintf("Q - Quit this program.\r\n");
#if GPS_ACTIVATED
		UARTprintf("P - Print raw GPS data.\r\n");
#endif
#if IMU_ACTIVATED
		UARTprintf("B - Print raw accel, gyro and mag data.\r\n");
#endif
#if SECONDARY_ATTITUDE
		UARTprintf("A - Trigger solar panel ADC.\r\n");
#endif
#if ULTRASONIC_ACTIVATED
		UARTprintf("1 - Trigger ultra sonic sensor #1.\r\n");
		UARTprintf("2 - Trigger ultra sonic sensor #2.\r\n");
#endif
#if AIRMTRS_ACTIVATED
		UARTprintf("w - Increase air motor throttle.\r\n");
		UARTprintf("s - Decrease air motor throttle.\r\n");
		UARTprintf("x - Stop the motors.\r\n");
#endif
#if GNDMTRS_ACTIVATED
		UARTprintf("i - Increase ground motor throttle.\r\n");
		UARTprintf("k - Decrease ground motor throttle.\r\n");
		UARTprintf("0 - Cut ground motor throttle.\r\n");
		UARTprintf("l - Turn on/off LED for ground motors.\r\n");
#endif
#if PAYLOAD_DEPLOY
		UARTprintf("D - Deploy payload.\r\n");
#endif
		break;
	}
	case 'Q': // Quit the program
	{
		g_Quit = true;

		break;
	}
#if GPS_ACTIVATED
	case 'P': // Print Raw GPS data.
	{
		if (g_PrintRawGPS)
			g_PrintRawGPS = false;
		else
			g_PrintRawGPS = true;
		break;
	}
#endif
#if IMU_ACTIVATED
	case 'B': // Print raw accel, gyro and mag data.
	{
		if (g_PrintRawBMIData)
			g_PrintRawBMIData = false;
		else
			g_PrintRawBMIData = true;
		break;
	}
#endif
#if SECONDARY_ATTITUDE
	case 'A': // Trigger solar panel ADC.
	{
		if (ADCBusy(SP_ADC)) {
			UARTprintf("ADC Busy!\r\n");
		} else {
			ADCProcessorTrigger(SP_ADC, 0);
		}
		break;
	}
#endif
#if ULTRASONIC_ACTIVATED
	case '1': // Trigger ultrasonic sensor #1
	{
		//
		// Trigger the sensor.
		GPIOPinWrite(USONIC_TRIG_PORT, TRIG_PIN_1, TRIG_PIN_1);

		//
		// Delay for at least 10 us = 160 clock cycles at 16 MHz.
		SysCtlDelay(200);

		//
		// Turn off the trigger pin.
		GPIOPinWrite(USONIC_TRIG_PORT, TRIG_PIN_1, 0x00);

		//
		// Console Feedback.
		UARTprintf("Trigger Sent!\r\n");

		//
		// Indicate which sensor is being operated.
		g_UltraSonicSensor = 1;

		break;
	}
	case '2': // Trigger ultrasonic sensor #2
	{
		//
		// Trigger the sensor.
		GPIOPinWrite(USONIC_TRIG_PORT, TRIG_PIN_2, TRIG_PIN_2);

		//
		// Delay for at least 10 us = 160 clock cycles at 16 MHz.
		SysCtlDelay(160);

		//
		// Turn off the trigger pin.
		GPIOPinWrite(USONIC_TRIG_PORT, TRIG_PIN_2, 0x00);

		//
		// Console Feedback.
		UARTprintf("Trigger Sent!\r\n");

		//
		// Indicate which sensor is being operated.
		g_UltraSonicSensor = 2;

		break;
	}
#endif
#if AIRMTRS_ACTIVATED
	case 'w': // Increase throttle of air motors.
	{
		sThrottle.ui32AirMtrThrottle += g_ui32ThrottleIncrement;
		PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, sThrottle.ui32AirMtrThrottle);
		PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, sThrottle.ui32AirMtrThrottle);
		PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, sThrottle.ui32AirMtrThrottle);
		PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, sThrottle.ui32AirMtrThrottle);

		UARTprintf("Throttle Increase: %d\r\n", sThrottle.ui32AirMtrThrottle);
		break;
	}
	case 's': // Decrease throttle of air motors.
	{
		sThrottle.ui32AirMtrThrottle -= g_ui32ThrottleIncrement;

		if (sThrottle.ui32AirMtrThrottle < ZEROTHROTTLE)
			sThrottle.ui32AirMtrThrottle += g_ui32ThrottleIncrement;
		PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, sThrottle.ui32AirMtrThrottle);
		PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, sThrottle.ui32AirMtrThrottle);
		PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, sThrottle.ui32AirMtrThrottle);
		PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, sThrottle.ui32AirMtrThrottle);

		UARTprintf("Throttle Decrease: %d\r\n", sThrottle.ui32AirMtrThrottle);
		break;
	}
	case 'x': // kill the air throttle.
	{
		PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, ZEROTHROTTLE);
		PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, ZEROTHROTTLE);
		PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, ZEROTHROTTLE);
		PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, ZEROTHROTTLE);

		UARTprintf("Throttle zeroed: %d\r\n", ZEROTHROTTLE);
		break;
	}
#endif
#if GNDMTRS_ACTIVATED
	case 'i': // Increase ground motor throttle.
	{
		uint8_t txBuffer[4];

		//
		// Increase the throttle.
		sThrottle.ui16GndMtrLWThrottle += RX24_THROTTLE_INCREMENT;
		sThrottle.ui16GndMtrRWThrottle += RX24_THROTTLE_INCREMENT;

		//
		// Build the instruction packet.
		txBuffer[0] = RX24_WRITE_DATA;
		txBuffer[1] = RX24_REG_MOVING_VEL_LSB;
		txBuffer[2] = (uint8_t)(sThrottle.ui16GndMtrLWThrottle & 0x00ff);
		txBuffer[3] = (uint8_t)(sThrottle.ui16GndMtrLWThrottle >> 8);

		//
		// Send the command to the LW motor.
		Rx24FWrite(GNDMTR1_UART, GNDMTR1_DIRECTION_PORT, GMDMTR1_DIRECTION,
				4, txBuffer);

		txBuffer[2] = (uint8_t)(sThrottle.ui16GndMtrRWThrottle & 0x00ff);
		txBuffer[3] = (uint8_t)(sThrottle.ui16GndMtrRWThrottle >> 8);

		//
		// Send the command to the RW motor.
		Rx24FWrite(GNDMTR2_UART, GNDMTR2_DIRECTION_PORT, GMDMTR2_DIRECTION,
				4, txBuffer);

		UARTprintf("Speeding up... \r\n");

		break;
	}
	case 'k': // Decrease ground motor throttle.
	{
		uint8_t txBuffer[4];

		//
		// Decrease the throttle.
		sThrottle.ui16GndMtrLWThrottle -= RX24_THROTTLE_INCREMENT;
		sThrottle.ui16GndMtrRWThrottle -= RX24_THROTTLE_INCREMENT;

		//
		// Build the instruction packet.
		txBuffer[0] = RX24_WRITE_DATA;
		txBuffer[1] = RX24_REG_MOVING_VEL_LSB;
		txBuffer[2] = (uint8_t)(sThrottle.ui16GndMtrLWThrottle & 0x00ff);
		txBuffer[3] = (uint8_t)(sThrottle.ui16GndMtrLWThrottle >> 8);

		//
		// Send the command to the LW motor.
		Rx24FWrite(GNDMTR1_UART, GNDMTR1_DIRECTION_PORT, GMDMTR1_DIRECTION,
				4, txBuffer);

		txBuffer[2] = (uint8_t)(sThrottle.ui16GndMtrRWThrottle & 0x00ff);
		txBuffer[3] = (uint8_t)(sThrottle.ui16GndMtrRWThrottle >> 8);

		//
		// Send the command to the RW motor.
		Rx24FWrite(GNDMTR2_UART, GNDMTR2_DIRECTION_PORT, GMDMTR2_DIRECTION,
				4, txBuffer);

		UARTprintf("Slowing down... \r\n");

		break;
	}
	case '0': // Cut ground motor throttle.
	{
		uint8_t txBuffer[4];

		//
		// Incrase the throttle.
		sThrottle.ui16GndMtrLWThrottle = RX24_STOP_CCW;
		sThrottle.ui16GndMtrRWThrottle = RX24_STOP_CCW;

		//
		// Build the instruction packet.
		txBuffer[0] = RX24_WRITE_DATA;
		txBuffer[1] = RX24_REG_MOVING_VEL_LSB;
		txBuffer[2] = (uint8_t)(sThrottle.ui16GndMtrLWThrottle & 0x00ff);
		txBuffer[3] = (uint8_t)(sThrottle.ui16GndMtrLWThrottle >> 8);

		//
		// Send the command to the LW motor.
		Rx24FWrite(GNDMTR1_UART, GNDMTR1_DIRECTION_PORT, GMDMTR1_DIRECTION,
				4, txBuffer);

		txBuffer[2] = (uint8_t)(sThrottle.ui16GndMtrRWThrottle & 0x00ff);
		txBuffer[3] = (uint8_t)(sThrottle.ui16GndMtrRWThrottle >> 8);

		//
		// Send the command to the RW motor.
		Rx24FWrite(GNDMTR2_UART, GNDMTR2_DIRECTION_PORT, GMDMTR2_DIRECTION,
				4, txBuffer);

		UARTprintf("Zero throttle.\r\n");
		break;
	}
	case 'l': // Activate GND motor LED.
	{
		uint8_t txBuffer[3];

		if (g_LEDON) {
			txBuffer[2] = 0x01;
			g_LEDON = false;
		}
		else {
			txBuffer[2] = 0x00;
			g_LEDON = true;
		}

		 //
		// Set the status return register.
		txBuffer[0] = RX24_WRITE_DATA;
		txBuffer[1] = RX24_REG_LED_EN;
		Rx24FWrite(GNDMTR2_UART, GNDMTR2_DIRECTION_PORT, GMDMTR2_DIRECTION,
				3, txBuffer);
	break;
	}
#endif
#if PAYLOAD_DEPLOY
	case 'D': // Deploy the payload.
	{
		DeployPayload();

		break;
	}
#endif
	}
#endif
	//
	// Reset the flag.
	g_ConsoleFlag = false;
}

//*****************************************************************************
//
// This function will parse the GPS data.
//
//*****************************************************************************
void ProcessGPS(void) {
	char currChar;
	char UTC[10];
	char Lat[10];
	char Long[11];
	char alt[6];
	char NorS = '0';
	char EorW = '0';
	int n = 0;
	int i = 0;

	char Buffer[100] = { 0 };

	while (UARTCharsAvail(GPS_UART)) {
		currChar = UARTCharGet(GPS_UART);
		if (g_PrintRawGPS) {
			UARTCharPut(CONSOLE_UART, currChar);
		}

		//
		// Try to find the GPGGA string.
		if (currChar == 'G') {
			currChar = UARTCharGet(GPS_UART);
			if (currChar == 'P') {
				currChar = UARTCharGet(GPS_UART);
				if (currChar == 'G') {
					currChar = UARTCharGet(GPS_UART);
					if (currChar == 'G') {
						currChar = UARTCharGet(GPS_UART);
						if (currChar == 'A') {
							//
							// Dump the comma.
							UARTCharGet(GPS_UART);

							for (n = 0; n < sizeof(Buffer); n++) {
								currChar = UARTCharGet(GPS_UART);

								Buffer[n] = currChar;
							}
						}
					}
				}
			}
		}
	}

	if (Buffer[0] != ',' && Buffer[0] != 0x00) {
		//
		// Set the flag that the GPS is connected.
		g_GPSConnected = true;

		//
		// Grab the UTC
		for (n = 0; n < 9; n++) {
			UTC[n] = Buffer[n];
		}

		//
		// Skip comma.
		n++;

		//
		// Grab the latitude.
		for (i = 0; i < 10; i++) {
			Lat[i] = Buffer[n + i];
		}

		//
		// Skip comma
		n = n + i + 1;

		//
		// Get the N or S.
		NorS = Buffer[n];

		n += 2;

		//
		// Grab the longitude.
		for (i = 0; i < 11; i++) {
			Long[i] = Buffer[n + i];
		}

		//
		// Skip the comma.
		n += i + 1;

		//
		// Grab the E or W.
		EorW = Buffer[n];

		//
		// Step to the altitude.
		n += 12;

		//
		// Grab the altitude.
		for (i = 0; i < 6; i++) {
			alt[i] = Buffer[n + i];
		}

		//
		// Set the UTC to a global variable.
		g_Pack.pack.UTC = atof(UTC);

		//
		// Compute the latitude.
		char temp[8];
		float temp2, temp3;

		temp[0] = Lat[0];
		temp[1] = Lat[1];

		temp2 = atof(temp);

		temp[0] = Lat[2];
		temp[1] = Lat[3];
		temp[2] = Lat[4];
		temp[3] = Lat[5];
		temp[4] = Lat[6];
		temp[5] = Lat[7];
		temp[6] = Lat[8];
		temp[7] = Lat[9];

		temp3 = atof(temp);

		if (NorS == 'N') {
			//
			// Positive latitude.
			g_Pack.pack.lat = temp2 + (temp3 / 60);
		} else if (NorS == 'S') {
			//
			// Negative latitude.
			g_Pack.pack.lat = (-1) * (temp2 + (temp3 / 60));
		}

		//
		// Compute the longitude.
		char templ[3];

		templ[0] = Long[0];
		templ[1] = Long[1];
		templ[2] = Long[2];

		temp2 = atof(templ);

		temp[0] = Long[3];
		temp[1] = Long[4];
		temp[2] = Long[5];
		temp[3] = Long[6];
		temp[4] = Long[7];
		temp[5] = Long[8];
		temp[6] = Long[9];
		temp[7] = Long[10];

		temp3 = atof(temp);

		if (EorW == 'E') {
			//
			// Positive longitude.
			g_Pack.pack.lon = temp2 + temp3 / 60;
		} else if (EorW == 'W') {
			//
			// Negative longitude.
			g_Pack.pack.lon = (-1) * (temp2 + (temp3 / 60));
		}

		//
		// Save the altitude.
		g_Pack.pack.alt = atof(alt);
	} else {
		//
		// GPS is not connected.
		g_GPSConnected = false;
	}

	//
	// Reset flag.
	g_GPSFlag = false;

	//
	// Turn on LED 3 to show that GPS has a lock.
	if (g_GPSConnected) {
		TurnOnLED(3);
	} else {
		TurnOffLED(3);
	}
}

//*****************************************************************************
//
// This function will process the packet received from the ground station.
//
//*****************************************************************************
void ProcessRadio(void) {
	//
	// Good radio connection. Reset the timer and set the status.
	sStatus.bRadioConnected = true;
	TimerLoadSet(RADIO_TIMER_CHECK, TIMER_A,
			16000000 / GS_RADIO_RATE);


	if (sStatus.bArmed) { // Don't mess with any system states until the system is armed.
		switch (g_sRxPack.ui8Data[3]) {
		case 'T': {
			//
			// Change the status to Autonomous.
			sStatus.ui8Mode = 'A';

			//
			// The target location has now been set.
			sStatus.bTargetSet = true;

			break;
		}
		case 'C': {
			//
			// Check if we are flying or driving and update the Status.
			if (g_sRxPack.sControlPacket.flyordrive == g_sRxPack.sControlPacket.fdConfirm)
				if (g_sRxPack.sControlPacket.flyordrive == 'D') {
				    //
				    // Set the system state to driving.
					sStatus.ui8Mode = 'D';

	                //
	                // Get the wheel throttles. They will be sent as percentages from -100 to 100.
	                sThrottle.i32RWThrottlePerc = (g_sRxPack.sControlPacket.throttle);
	                sThrottle.i32LWThrottlePerc = (g_sRxPack.sControlPacket.throttle2);

	                //
	                // Check to make sure neither throttle exceeds 100 or -100.
	                if (sThrottle.i32RWThrottlePerc > 100)
	                    sThrottle.i32RWThrottlePerc = 100;
	                if (sThrottle.i32RWThrottlePerc < -100)
	                    sThrottle.i32RWThrottlePerc = -100;
	                if (sThrottle.i32LWThrottlePerc > 100)
	                    sThrottle.i32LWThrottlePerc = 100;
	                if (sThrottle.i32LWThrottlePerc < -100)
	                    sThrottle.i32LWThrottlePerc = -100;
				}
				else if (g_sRxPack.sControlPacket.flyordrive == 'F') {
				    //
				    // Set the system status to flying.
				    sStatus.ui8Mode = 'F';
					
				    //
				    // Get the throttle for all of the motors.
				    sThrottle.ui32AirMtrThrottle = g_sRxPack.sControlPacket.throttle;

				    //
				    // Get the desired roll and pitch.
				    sStatus.fDesRoll = g_sRxPack.sControlPacket.roll;
				    sStatus.fDesPitch = g_sRxPack.sControlPacket.pitch;

				    //
				    // Check if we need to yaw.
				    if (g_sRxPack.sControlPacket.yaw > 0.0f)
				        sStatus.i32DesYaw = 1;
				    else if (g_sRxPack.sControlPacket.yaw < 0.0f)
				        sStatus.i32DesYaw = -1;
				    else
				        sStatus.i32DesYaw = 0;
				}

			//
			// Check if we should deploy the payload.
			if ((!sStatus.bPayDeployed) &&
			        (g_sRxPack.sControlPacket.payloadRelease == g_sRxPack.sControlPacket.prConfirm))
				if (g_sRxPack.sControlPacket.payloadRelease == 1)
					DeployPayload();

			break;
		}
		case '0': {
			//
			// Change the mode to autonomous.
			sStatus.ui8Mode = 'A';

			//
			// Receiving bad data. Tell main to ignore it.
			sStatus.bTargetSet = false;

			break;
		}
		case 'D': { // Disarm the system.
			//
			// Disable the update trajectory and shut off all motors.
			sStatus.bArmed = false;

			//
			// Turn on all the LEDs to indicate waiting to re-arm.
			TurnOnLED(5);

#if GNDMTRS_ACTVIATED
		    //
		    // Set the ground motors to zero throttle.
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
#endif

#if AIRMTRS_ACTIVATED
			//
		    // Set the air motors to zero throttle and disable the generators.
		    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, ZEROTHROTTLE);
		    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, ZEROTHROTTLE);
		    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, ZEROTHROTTLE);
		    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, ZEROTHROTTLE);

		    PWMGenDisable(PWM0_BASE, PWM_GEN_0);
		    PWMGenDisable(PWM0_BASE, PWM_GEN_1);
		    PWMGenDisable(PWM0_BASE, PWM_GEN_2);
#endif

		    //
		    // Disable interrupts except for the radio and sensors.
		    SysTickDisable();

#if GNDMTRS_ACTIVATED || AIRMTRS_ACTIVATED
            //
            // Enable trajectory timer.
            TimerDisable(UPDATE_TIMER, TIMER_B);
#endif

			break;
		}
#if DEBUG
		default:
		{
		    //
		    // Print out some stuff for debugging.
		    UARTprintf("Radio reached the default case in\r\n");
		    UARTprintf("the ProcessRadio function.\r\n");

		    while(1)
		    {
		        //
		        // Loop doing nothing, til this problem is solved.
		    }

		}
#endif
		}
	} else { // Arm the System.
		switch (g_sRxPack.ui8Data[3]) {
		case 'A': {
			//
			// Arm the system.
			// This is equivalent to pushing the button on the microcontroller.
			sStatus.bArmed = true;

			break;
		}
		}
	}
}

//*****************************************************************************
//
// This function will evaluate the data received in the ADC.
//
//*****************************************************************************
void ProcessADC(void) {
	uint32_t ADCData[8];

	//
	// Get the data.
	ADCSequenceDataGet(SP_ADC, 0, ADCData);

#if DEBUG
	//
	// Print out the data to the console.
	UARTprintf("SP1 = %d\r\n", ADCData[0]);
	UARTprintf("SP2 = %d\r\n", ADCData[1]);
	UARTprintf("SP3 = %d\r\n", ADCData[2]);
	UARTprintf("SP4 = %d\r\n", ADCData[3]);
	UARTprintf("SP5 = %d\r\n", ADCData[4]);
#endif

	//
	// Reset the flag.
	g_ADCFlag = false;
}

//*****************************************************************************
//
// This function will determine how far away an obstacle is.
//
//*****************************************************************************
int ProcessUltraSonic(uint32_t SysClockSpeed) {
	float distance;
	int deltaT;

	//
	// Calculate the timer between edges.
	deltaT = g_TimerRiseValue - g_TimerFallValue;

	if (deltaT < 0) {
		//
		// Got a negative number, add 2^16 * 10 to it.
		deltaT = deltaT + (65535 * 10);
	}

	//
	// Calculate the distance to the obstacle.
	distance = deltaT * 340.0f / 2.0f / SysClockSpeed;

	//
	// Reset the flag.
	g_UltraSonicFlag = false;

#if DEBUG
	UARTprintf("deltaT = %d\r\n", deltaT);
#endif

	return distance;
}

//*****************************************************************************
//
// This function will send a packet to the ground station, if the radio is
// connected, it will be called by Timer 0 - Timer A at the rate specified.
//
//*****************************************************************************
void SendPacket(void) {
	int n;
	uint32_t ui32Status;

	//
	// Get the interrupt status.
	ui32Status = TimerIntStatus(RADIO_TIMER, true);

	//
	// Clear the interrupt.
	TimerIntClear(RADIO_TIMER, ui32Status);

	if (sStatus.bRadioConnected) {
		g_Pack.pack.accelX = sSensStatus.fCurrentAccelX;
		g_Pack.pack.accelY = sSensStatus.fCurrentAccelY;
		g_Pack.pack.accelZ = sSensStatus.fCurrentAccelZ;

		g_Pack.pack.velX = sSensStatus.fCurrentGyroX;
		g_Pack.pack.velY = sSensStatus.fCurrentGyroY;
		g_Pack.pack.velZ = sSensStatus.fCurrentGyroZ;

		g_Pack.pack.posX = g_fMagData[0];
		g_Pack.pack.posY = g_fMagData[1];
		g_Pack.pack.posZ = g_fMagData[2];

		//
		// Current orientation.
		g_Pack.pack.roll = sStatus.fRoll;
		g_Pack.pack.pitch = sStatus.fPitch;
		g_Pack.pack.yaw = sStatus.fYaw;

		//
		// Mode of operation.
		if (sStatus.ui8Mode == 'F') { // flying
			g_Pack.pack.movement = 'F';
			g_Pack.pack.amtr1 = true;
			g_Pack.pack.amtr2 = true;
			g_Pack.pack.amtr3 = true;
			g_Pack.pack.amtr4 = true;
			g_Pack.pack.gndmtr1 = false;
			g_Pack.pack.gndmtr2 = false;
		} else if (sStatus.ui8Mode == 'D') { // driving
			g_Pack.pack.movement = 'D';
			g_Pack.pack.gndmtr1 = true;
			g_Pack.pack.gndmtr2 = true;
			g_Pack.pack.amtr1 = false;
			g_Pack.pack.amtr2 = false;
			g_Pack.pack.amtr3 = false;
			g_Pack.pack.amtr4 = false;
		}
		else { // autonomous
			g_Pack.pack.movement = 'D';
			g_Pack.pack.gndmtr1 = false;
			g_Pack.pack.gndmtr2 = false;
			g_Pack.pack.amtr1 = false;
			g_Pack.pack.amtr2 = false;
			g_Pack.pack.amtr3 = false;
			g_Pack.pack.amtr4 = false;
		}

		//
		// Status bits.
		g_Pack.pack.uS1 = false;
		g_Pack.pack.uS2 = false;
		g_Pack.pack.uS3 = false;
		g_Pack.pack.uS4 = false;
		g_Pack.pack.uS5 = false;
		g_Pack.pack.uS6 = false;
		g_Pack.pack.payBay = sStatus.bPayDeployed;

		//
		// Send the data over the radio.
		for (n = 0; n < sizeof(g_Pack.str); n++)
			UARTCharPut(RADIO_UART, g_Pack.str[n]);
	}
}

//*****************************************************************************
//
// This function will retrieve the accel or gyro data from the BMI160.
//
//*****************************************************************************
void ProcessIMUData(void) {
	uint8_t status;
	uint8_t IMUData[20]; // raw accel, gyro and mag data.
	int16_t i16AccelData[3];
	int16_t i16GyroData[3];
	int16_t i8MagData[3];
	float fAccelDataUnCal[3];
	float fGyroDataUnCal[3];

	//
	// First check the status for which data is ready.
	I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_STATUS, 1, &status);

	//
	// Check if the magnetometer data is ready.
	if ((status & 0x20) == (BMI160_MAG_RDY)) {
		//
		// Then get the data for both the accel, gyro and mag
		I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_MAG_X, 6, IMUData);

		//
		// Get the mag data.
		i8MagData[0] = (((int16_t) IMUData[1] << 8)
				+ (int8_t) (IMUData[0] & 0x1f));
		i8MagData[1] = (((int16_t) IMUData[3] << 8)
				+ (int8_t) (IMUData[2] & 0x1f));
		i8MagData[2] = (((int16_t) IMUData[5] << 8)
				+ (int8_t) (IMUData[4] & 0x7f));

		//
		// Convert to float for DCM and convert to teslas.
		g_fMagData[0] = ((i8MagData[0] / MAGLSB) - g_MagBias[0]) / 1e6;
		g_fMagData[1] = ((i8MagData[1] / MAGLSB) - g_MagBias[1]) / 1e6;
		g_fMagData[2] = ((i8MagData[2] / MAGLSB) - g_MagBias[2]) / 1e6;

		//
		// Blink the LED 1 to indicate sensor is working.
		if (g_LED1On) {
			TurnOffLED(1);
			g_LED1On = false;
		} else {
			TurnOnLED(1);
			g_LED1On = true;
		}
	}

	//
	// Check if the accel and gyro data are ready.
	if ((status & 0xC0) == (BMI160_ACC_RDY | BMI160_GYR_RDY)) {
		//
		// Then get the data for both the accel, gyro and mag
		I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_GYRO_X, 12, IMUData);

		//
		// Set the gyro data to the global variables.
		i16GyroData[0] = (((int16_t) IMUData[1] << 8) + (int8_t) IMUData[0]);
		i16GyroData[1] = (((int16_t) IMUData[3] << 8) + (int8_t) IMUData[2]);
		i16GyroData[2] = (((int16_t) IMUData[5] << 8) + (int8_t) IMUData[4]);

		//
		// Convert data to float.
		fGyroDataUnCal[0] = (((float) (i16GyroData[0])) / GYROLSB) - BGX;
		fGyroDataUnCal[1] = (((float) (i16GyroData[1])) / GYROLSB) - BGY;
		fGyroDataUnCal[2] = (((float) (i16GyroData[2])) / GYROLSB) - BGZ;

		//
		// Calculate the calibrated gyro data.
		sSensStatus.fCurrentGyroX = (fGyroDataUnCal[0] * SGX + fGyroDataUnCal[1] * MGXY + fGyroDataUnCal[2] * MGXZ) - g_GyroStabBias[0];
		sSensStatus.fCurrentGyroY = (fGyroDataUnCal[0] * MGYX + fGyroDataUnCal[1] * SGY + fGyroDataUnCal[2] * MGYZ) - g_GyroStabBias[1];
		sSensStatus.fCurrentGyroZ = (fGyroDataUnCal[0] * MGZX + fGyroDataUnCal[1] * MGZY + fGyroDataUnCal[2] * SGZ) - g_GyroStabBias[2];

		//
		// Set the accelerometer data.
		i16AccelData[0] = (((int16_t) IMUData[7] << 8) + (int8_t) IMUData[6]);
		i16AccelData[1] = (((int16_t) IMUData[9] << 8) + (int8_t) IMUData[8]);
		i16AccelData[2] = (((int16_t) IMUData[11] << 8) + (int8_t) IMUData[10]);

		//
		// Compute the accel data into floating point values.
		fAccelDataUnCal[0] = (((float) i16AccelData[0]) / ACCELLSB) - BAX;
		fAccelDataUnCal[1] = (((float) i16AccelData[1]) / ACCELLSB) - BAY;
		fAccelDataUnCal[2] = (((float) i16AccelData[2]) / ACCELLSB) - BAZ;

		//
		// Calculate the calibrated accelerometer data.
		sSensStatus.fCurrentAccelX = fAccelDataUnCal[0] * SAX + fAccelDataUnCal[1] * MAXY + fAccelDataUnCal[2] * MAXZ;
		sSensStatus.fCurrentAccelY = fAccelDataUnCal[0] * MAYX + fAccelDataUnCal[1] * SAY + fAccelDataUnCal[2] * MAYZ;
		sSensStatus.fCurrentAccelZ = fAccelDataUnCal[0] * MAZX + fAccelDataUnCal[1] * MAZY + fAccelDataUnCal[2] * SAZ;
	}

	//
	// Enable the DCM if it has not been started yet.
	if (g_bDCMStarted == 0)
		TimerEnable(DCM_TIMER, TIMER_A);

#if DEBUG
	//
	// Print the raw data if turned on.
	if (g_PrintRawBMIData && g_PrintIMUData) {
		UARTprintf("Accelx = %d\r\nAccely = %d\r\n", i16AccelData[0],
				i16AccelData[1]);
		UARTprintf("Accelz = %d\r\n", i16AccelData[2]);
		UARTprintf("Gyrox = %d\r\nGyroy = %d\r\n", i16GyroData[0],
				i16GyroData[1]);
		UARTprintf("Gyroz = %d\r\n", i16GyroData[2]);

		UARTprintf("Magx = %d\r\nMagy = %d\r\n", i8MagData[0], i8MagData[1]);
		UARTprintf("Magz = %d\r\n", i8MagData[2]);

		//
		// Reset loop count.
		g_PrintIMUData = false;
	}
#endif

	//
	// Reset the flag
	g_IMUDataFlag = false;
}

//*****************************************************************************
//
// This function will retrieve data from the pressure sensor at a fixed
// frequency.
//
//*****************************************************************************
void ProcessBME280(void) {
	int32_t tempInt;
	uint32_t presInt;
	int32_t rawPress, rawTemp;
	uint8_t rxBuffer[16];
	uint8_t *ptrBuffer = &rxBuffer[0];

	//
	// Get the raw data.
	GetBME280RawData(BOOST_I2C, ptrBuffer);

	rawPress = (rxBuffer[0] << 12) | (rxBuffer[1] << 4) | (rxBuffer[2] >> 4);
	rawTemp = (rxBuffer[3] << 12) | (rxBuffer[4] << 4) | (rxBuffer[5] >> 4);
	//rawHumid = (rxBuffer[6] << 8) | (rxBuffer[7]);

	//tempInt = *g_ptrBME280RawData << 20;

	//
	// Correct the temp data.
	tempInt = BME280_compensate_T_int32(rawTemp, g_p_t_fine,
			g_BME280OffsetValues, g_BME280OffsetUnsigned);

	//
	// Calculate temp in degrees C, float format.
	g_fTemp = tempInt / 100.0f;

	//
	// Correct the pressure data.
	presInt = BME280_compensate_P_int64(rawPress, g_p_t_fine,
			g_BME280OffsetValues, g_BME280OffsetUnsigned + 1);

	//
	// Calculate pressure in float form (Pascals).
	g_fPressure = presInt / 256.0f;
}

//*****************************************************************************
//
// Deploy payload function will update the position of the servo motors in
// order to release the payload.
//
//*****************************************************************************
void DeployPayload(void)
{
    IntMasterDisable();

	//
	// Move the servos to the open position.
    g_ui32ServoAngle = g_ui32ServoEndPosition;

	PWMPulseWidthSet(PWM0_BASE, SERVO_1, g_ui32ServoAngle);
	PWMPulseWidthSet(PWM0_BASE, SERVO_2, g_ui32ServoAngle);

	IntMasterEnable();

	//
	// Modify the flag.
	sStatus.bPayDeployed = true;

#if DEBUG
	UARTprintf("Payload Deployed\r\n");
#endif

}

//*****************************************************************************
//
// This function will wait for the arm trigger from either the left button
// press or the signal from the radio indicating an arm sequence.
//
//*****************************************************************************
void WaitForArming(void)
{
	TurnOnLED(5);

	while (!sStatus.bArmed) {
		if (WaitForButtonPress(LEFT_BUTTON))
				sStatus.bArmed = true;
	}

	TurnOffLED(5);

	//
	// Arm all of the interrupts here.
    SysTickEnable();

#if (AIRMTRS_ACTIVATED)
    //
    // Activate the motors for APOPHIS.
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

#endif

#if GNDMTRS_ACTIVATED || AIRMTRS_ACTIVATED
    //
    // Enable trajectory timer.
    TimerEnable(UPDATE_TIMER, TIMER_B);
#endif

}

void RadioIntHandler3(void) {
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
				if (ui8Index > 40)
					while(1)
					{
						TurnOnLED(5);
						SysCtlDelay(100);
						TurnOffLED(5);
						SysCtlDelay(100);
					}
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
}
