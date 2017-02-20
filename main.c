/*
 * Team Ouranos
 * Project: Aerial Platform for Overland Haul and Import System (APOPHIS)
 *
 *  Created On: Jan 20, 2017
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

#include "buttons.h"

#include "utils/uartstdio.h"

#include "initializations.h"
#include "APOPHIS_pin_map.h"
#include "packet_format.h"
#include "bmi160.h"
#include "i2c_driver.h"
#include "bme280.h"

//*****************************************************************************
//
// Defines
//
//*****************************************************************************

#define DEBUG true

#define CONSOLE_ACTIVATED true
#define RADIO_ACTIVATED true
#define GPS_ACTIVATED false
#define GNDMTRS_ACTIVATED false
#define SOLARS_ACTIVATED false
#define ULTRASONIC_ACTIVATED false
#define SOLENOIDS_ACTIVATED false
#define IMU_ACTIVATED true
#define ALTIMETER_ACTIVATED false
#define AIRMTRS_ACTIVATED true

#define ONEG 16384

//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
void SysTickIntHandler(void);
void ConsoleIntHandler(void);
void RadioIntHandler(void);
void GPSIntHandler(void);
void GndMtr1IntHandler(void);
void GndMtr2IntHandler(void);
void Timer2AInterrupt(void);
void Timer2BInterrupt(void);
void Timer3AInterrupt(void);
void Timer3BInterrupt(void);
void Timer1AInterrupt(void);
void Timer1BInterrupt(void);
void SolenoidInterrupt(void);
void BMI160IntHandler(void);
void BME280IntHandler(void);
void TurnOnLED(uint32_t LEDNum);
void TurnOffLED(uint32_t LEDNum);
void Menu(char CharReceived);
void ProcessGPS(void);
void ProcessRadio(void);
void ProcessADC(void);
int ProcessUltraSonic(uint32_t SysClockSpeed);
void ActivateSolenoids(void);
void DeactivateSolenoids(void);
void SendPacket(void);
void ProcessIMUData(void);
void ProcessMagData(void);
void WaitForButtonPress(uint8_t ButtonState);
void UpdateTrajectory(void);

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************

//
// State of the system structure definition and variable.
typedef struct {
    bool bFlyOrDrive;   // Drive is false, fly is true.
    bool bMode;         // Autonomous is true, manual is false.
    bool bPayDeployed;  // Whether the payload has been deployed.
    bool bGoodRadioData;    // Determines whether the radio data is good or not.
    float fRoll;		// Actual platform roll (degrees).
    float fPitch;		// Actual platform pitch (degrees).
    float fYaw;			// Actual platform yaw (degrees).
} SystemStatus;

typedef struct {
	float fGndMtrLWThrottle;
	float fGndMtrRWThrottle;
	float fAirMtr1Throttle;
	float fAirMtr2Throttle;
	float fAirMtr3Throttle;
	float fAirMtr4Throttle;
} SystemThrottle;

//
// System throttle structure.
SystemThrottle sThrottle;

uint32_t g_mtrThrottle = ZEROTHROTTLE;

//
// Initialize the state of the system.
SystemStatus sStatus;

//
// Global Instance structure to manage the DCM state.
tCompDCM g_sCompDCMInst;
bool g_bDCMStarted;

//
// Radio packet to be sent to the ground station.
uTxPack g_Pack;

//
// Radio packet to be received from ground station.
uRxPack g_sRxPack;

//
// Variable to store when USER LED 4 is on.
bool g_LED4On = false;

//
// Variable to send data to the ground station every second. Triggered by
// SysTickIntHandler().
bool g_SendPacket = false;

//
// Variable to store the system clock speed.
uint32_t g_SysClockSpeed;

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
// Variable to indicate when Radio data is available.
bool g_RadioFlag = false;

/*
 * GPS globals.
 */
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

/*
 * Ultrasonic sensor globals.
 */
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

/*
 * Acceleromter and Gyro global values.
 */
//
// Temporary storage for the accel and gyro data received from the BMI160.
// Each x,y,z value takes up two bytes.
bool g_IMUDataFlag = false;

//
// The number of LSB per g for +/- 2g's.
float g_accelLSBg = 16384;

//
// The number of LSB per degree/s for 2000 degrees/s.
float g_fGyroLSB = 16.4;

//
// Offset compensation data for the accel and gyro.
uint8_t g_offsetData[7] = {0};
uint16_t g_Bias[6] = {0};

//
// Used as global storage for the gyro data.
int16_t g_gyroDataRaw[3];
float g_fGyroData[3];

//
// Used as global storage for the raw mag data.
// The first three elements are the x, y, z values.
// The fourth element is the Rhall vlaue.
int16_t g_magDataRaw[4];

//
// Used to indicate if the IMU is working, by blinking LED 1.
bool g_LED1On = false;

//
// Used to indicate when to print the accel and gyro values to the PC.
// Set to once per second. Triggered every second by the SysTickIntHandler()
bool g_loopCount = false;

//
// Floating point conversion factors for accelerometer. Found by dividing
// 9.81 m/s^2 by the LSB/g / 2. e.g. 9.81 / 8192 = 0.00119750976
float g_Accel2GFactor = 0.00119750976;

/*
 * Globals for BME280 environmental sensor.
 */
//
// BME280 flag for triggered data analysis in main().
bool g_BME280Ready = false;

//
// Variable to determine whether to print the raw accel and gyro data to the terminal.
bool g_PrintRawBMIData = false;

//
// Storage for the BME280 raw data and its pointer.
int8_t g_BME280RawData[7];
int8_t *g_ptrBME280RawData = &g_BME280RawData[0];

int8_t g_BME280OffsetValues[12];
uint8_t g_BME280OffsetUnsigned[2];

//
// Floating format for pressure and temperature data.
float g_Pressure;
float g_Temp;

//
// global temperature fine value for BME280;
int32_t g_t_fine;
int32_t *g_p_t_fine = &g_t_fine;

//
// Global flag for when mag data is ready.
bool g_MagDataFlag = false;

//*****************************************************************************
//
// Start of program.
//
//*****************************************************************************
int main(void) {

    bool bBiasCalcBad = true;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    FPUEnable();
    FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal. (16 MHz)
    g_SysClockSpeed = SysCtlClockFreqSet(SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                                         SYSCTL_XTAL_16MHZ, 16000000);

    //
    // Disable interrupts during initialization period.
    IntMasterDisable();

    //
    // Before doing anything, initialize the LED.
    InitLED(g_SysClockSpeed);

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
    // Initialize the Console if turned on.
    InitConsole();
    UARTprintf("Clock speed: %d\r\n", g_SysClockSpeed);

    //
    // Initialize the radio if turned on.
#if RADIO_ACTIVATED
    InitRadio(g_SysClockSpeed);
#endif

    //
    // Initialize the GPS if turned on.
#if GPS_ACTIVATED
    InitGPS(g_SysClockSpeed);
#endif

    //
    // Initialize the ground motors if turned on.
#if GNDMTRS_ACTIVATED
    InitGndMotors(g_SysClockSpeed);
#endif

    //
    // Initialize the solar panels if turned on.
#if SOLARS_ACTIVATED
    InitSolarPanels();
#endif

    //
    // Initialize the ultrasonic sensors if turned on.
#if ULTRASONIC_ACTIVATED
    InitUltraSonicSensor();
#endif

    //
    // Initialize the solenoid enable pins if turned on.
#if SOLENOIDS_ACTIVATED
    InitSolenoidEnablePins(g_SysClockSpeed);
#endif

    //
    // Initialize the pressure sensor if enabled.
#if ALTIMETER_ACTIVATED
    //
    // Initialize the altimeter.
    InitAltimeter(g_SysClockSpeed, g_BME280OffsetValues);

    //
    // Set the offset values.
    g_BME280OffsetUnsigned[0] = g_BME280OffsetValues[0];
    g_BME280OffsetUnsigned[1] = g_BME280OffsetValues[3];
#endif

    //
    // Initialize the air motors if activated.
#if AIRMTRS_ACTIVATED
    InitAirMtrs(g_SysClockSpeed);
#endif

    //
    // Initialize the BMI160 if enabled.
#if IMU_ACTIVATED

    InitIMU(g_SysClockSpeed, g_offsetData);

    //
    // Get the initial reading of the gyro and accel to calculate a bias.
    while(bBiasCalcBad)
    {
        int numCalcs = 0;
        int index, j;
        uint32_t ui32Sum[6] = {0};
        uint16_t bias[6][50] = {0};
        IntMasterEnable();
        while(numCalcs < 50)
        {
            if (g_IMUDataFlag)
            {
                uint8_t status;
                uint8_t IMUData[12] = {0};

                //
                // First check the status for which data is ready.
                I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_STATUS, 1, &status);

                //
                // Check what status returned.
                if ((status & 0xC0) == (BMI160_ACC_RDY | BMI160_GYR_RDY))
                {
                    //
                    // Then get the data for both the accel and gyro.
                    I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_GYRO_X, 12, IMUData);

                    //
                    // Capture the gyro data.
                    bias[3][numCalcs] = ((IMUData[1] << 8) + IMUData[0]);
                    bias[4][numCalcs] = ((IMUData[3] << 8) + IMUData[2]);
                    bias[5][numCalcs] = ((IMUData[5] << 8) + IMUData[4]);

                    //
                    // Capture the accel data.
                    bias[0][numCalcs] = ((IMUData[7] << 8) + IMUData[6]);
                    bias[1][numCalcs] = ((IMUData[9] << 8) + IMUData[8]);
                    bias[2][numCalcs] = ((IMUData[11] << 8) + IMUData[10]);

                    numCalcs++;
                }
            }
        }

        IntMasterDisable();

        //
        // Calculate the bias.
        for (index = 0; index < numCalcs; index++)
        {
            for (j = 0; j < 6; j++)
            {
                ui32Sum[j] += bias[j][index];
            }
        }

        //
        // Actual bias.
        for (index = 0; index < 6; index++)
        {
            g_Bias[index] = ui32Sum[index] / numCalcs;
        }

        //
        // Check if the accel results are good data.
        if ((g_Bias[0] < ONEG/20) || (g_Bias[0] > (65536 - ONEG/20)))
        {
            //
            // Good data, x-axis is flat. Now check y-axis.
            if ((g_Bias[1] < ONEG/20) || (g_Bias[1] > (65536 - ONEG/20)))
            {
                //
                // Good data, y-axis is flat. Now check z-axis.
                if ((g_Bias[2] > (ONEG - ONEG/20)) || (g_Bias[2] < (ONEG + ONEG/20)))
                {
                    //
                    // Remove the 1G portion of the bias for the z-axis.
                    g_Bias[2] -= ONEG;

                    //
                    // All good data.
                    bBiasCalcBad = false;

                    UARTprintf("Accelerometer calibration successful!\r\n");
                }
                else
                    UARTprintf("BAD CALIBRATION! Z-axis is not pointing up!!\r\n");
            }
            else
                UARTprintf("BAD CALIBRATION X and Y axes are not flat!!\r\n");
        }
        else
            UARTprintf("BAD CALIBRATION X and Y axes are not flat!!\r\n");
    }

    //
    // Initialize the DCM.
    CompDCMInit(&g_sCompDCMInst, 1.0 / 25.0f, 0.3f, 0.7f, 0.0f);
    g_bDCMStarted = false;

#endif

    //
    // Initialize the state of the system.
    sStatus.bFlyOrDrive = false;
    sStatus.bMode = false;
    sStatus.bPayDeployed = false;
    sStatus.bGoodRadioData = false;

    //
    // Initialize the throttle of the system.
    sThrottle.fAirMtr1Throttle = ZEROTHROTTLE;
    sThrottle.fAirMtr2Throttle = ZEROTHROTTLE;
    sThrottle.fAirMtr3Throttle = ZEROTHROTTLE;
    sThrottle.fAirMtr4Throttle = ZEROTHROTTLE;
    sThrottle.fGndMtrRWThrottle = 0.0f;
    sThrottle.fGndMtrLWThrottle = 0.0f;

    //
    // Before starting program, wait for a button press on either switch.
    UARTprintf("Initialization Complete!\r\nPress left button to start.");

    WaitForButtonPress(LEFT_BUTTON);

    //
    // Initialization complete. Enable interrupts.
    IntMasterEnable();

    //
    // Turn off LED1, and enable the systick at 2 Hz to
    // blink LED 4, signifying regular operation.
    TurnOffLED(1);
    SysTickPeriodSet(g_SysClockSpeed / 2);
    SysTickEnable();

#if AIRMTRS_ACTIVATED
    //
    // Activate the motors.
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
#endif

    //
    // Print menu.
    Menu('M');

    //
    // Program start.
    while(!g_Quit)
    {
        //
        // First check for commands from Console.
        if (g_ConsoleFlag)
            Menu(g_CharConsole);

        //
        // Now check if GPS data is ready.
        if (g_GPSFlag)
            ProcessGPS();

        //
        // Check if data from the radio is ready.
        if (g_RadioFlag)
            ProcessRadio();

        //
        // Check if ADC is finished.
        if (g_ADCFlag)
            ProcessADC();

        //
        // Check if Ultrasonic is done.
        if (g_UltraSonicFlag)
            ProcessUltraSonic(g_SysClockSpeed);

        //
        // Check if accel or gyro data is ready.
        if (g_IMUDataFlag)
            ProcessIMUData();

        //
        // Check if mag data is ready.
        if (g_MagDataFlag)
            ProcessMagData();

        //
        // Check if pressure or temperature data is ready.
        if (g_BME280Ready)
        {
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
                                                g_BME280OffsetValues,
                                                g_BME280OffsetUnsigned);

            //
            // Calculate temp in degrees C, float format.
            g_Temp = tempInt / 100.0f;

            //
            // Correct the pressure data.
            presInt = BME280_compensate_P_int64(rawPress, g_p_t_fine,
                                                g_BME280OffsetValues,
                                                g_BME280OffsetUnsigned + 1);

            //
            // Calculate pressure in float form (Pascals).
            g_Pressure = presInt / 256.0f;
        }

        //
        // Check if it is time to send a packet to the ground station.
#if RADIO_ACTIVATED
        if (g_SendPacket)
            SendPacket();
#endif

#if AIRMTRS_ACTIVATED
        //
        // Update the trajectory.
        if (sStatus.bGoodRadioData)
        	UpdateTrajectory();
#endif

    }
    //
    // Program ending. Do any clean up that's needed.

    UARTprintf("Goodbye!\r\n");

    I2CMasterDisable(BOOST_I2C);

    TurnOffLED(5);

    IntMasterDisable();

    return 0;
}

/*
 * Interrupt handlers go here:
 */
//*****************************************************************************
//
// Interrupt handler for the console communication with the PC.
//
//*****************************************************************************
void SysTickIntHandler(void)
{
    if (g_LED4On)
    {
        //
        // Turn off LED 4 if it is on.
        TurnOffLED(4);

        g_LED4On = false;
    }
    else
    {
        //
        // Otherwise turn it on.
        TurnOnLED(4);

        g_LED4On = true;
    }

    //
    // Trigger sending a radio packet to the ground station.
    g_SendPacket = true;

    //
    // Trigger printing accel and gyro data to PC terminal.
    g_loopCount = true;
}

//*****************************************************************************
//
// Interrupt handler for the console communication with the PC.
//
//*****************************************************************************
void ConsoleIntHandler(void)
{
    //
    // First get the interrupt status. Then clear the associated interrupt flag.
    uint32_t ui32Status = UARTIntStatus(CONSOLE_UART, true);
    UARTIntClear(CONSOLE_UART, ui32Status);

    //
    // Get the character sent from the PC.
    g_CharConsole = UARTCharGetNonBlocking(CONSOLE_UART);

    //
    // Echo back to Radio.
    //UARTCharPutNonBlocking(CONSOLE_UART, g_CharConsole);

    //
    // Trigger the flag for char received from console.
    g_ConsoleFlag = true;
}

//*****************************************************************************
//
// Interrupt handler which handles reception of characters from the radio.
//
//*****************************************************************************
void RadioIntHandler(void)
{

    // TODO: wtf magic!?

    static uint8_t ui8Index = 0;
    static uint8_t ui8Magic[4] = {0};
    static uint8_t ui8MagicCount;
    static bool bValidData = false;
    int32_t i32RxChar;

    //
    // Get the interrupt status and clear the associated interrupt.
    uint32_t ui32Status = UARTIntStatus(RADIO_UART, true);
    UARTIntClear(RADIO_UART, ui32Status);

    //
    // Get the character received and send it to the console.
    while(UARTCharsAvail(RADIO_UART))
    {
        i32RxChar = UARTCharGetNonBlocking(RADIO_UART);
        if (ui8Index >= (sizeof(uRxPack) - 1)) ui8Index = 0;
        if (i32RxChar != -1) {
            if (bValidData) {
                //
                // Get the chars over the UART.
                g_sRxPack.ui8Data[ui8Index++] = (uint8_t) i32RxChar;
                if (((g_sRxPack.ui8Data[3] == 'T' || g_sRxPack.ui8Data[3] == '0') && ui8Index >= sizeof(tGSTPacket) - 1) || (g_sRxPack.ui8Data[3] == 'C' && ui8Index >= sizeof(tGSCPacket) - 1)) {
                    ui8Index = 0;
                    g_RadioFlag = true;
                    bValidData = false;
                    break;
                }
            } else {
                ui8Magic[ui8Index] = (uint8_t) i32RxChar;
                ui8Index = (ui8Index + 1) % 4;
                if (ui8MagicCount >= 3) {
                    if (ui8Magic[ui8Index % 4] == 0xFF && ui8Magic[(ui8Index+1) % 4] == 0xFF && ui8Magic[(ui8Index+2) % 4] == 0xFF && (ui8Magic[(ui8Index+3) % 4] == 'T' || ui8Magic[(ui8Index+3) % 4] == 'C' || ui8Magic[(ui8Index+3) % 4] == '0')) {
                        g_sRxPack.ui8Data[3] = ui8Magic[(ui8Index+3) % 4];
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

//*****************************************************************************
//
// Interrupt handler which notifies main program when GPS data is available.
//
//*****************************************************************************
void GPSIntHandler(void)
{
    //
    // Get the interrupt status and clear the associated interrupt.
    uint32_t ui32Status = UARTIntStatus(GPS_UART, true);
    UARTIntClear(GPS_UART, ui32Status);

    //
    // Signal to main() that GPS data is ready to be retreived.
    g_GPSFlag = true;
}

//*****************************************************************************
//
// Interrupt handler for motor 1.
//
//*****************************************************************************
void GndMtr1IntHandler(void)
{
    // TODO: Define Ground Motor 1 interrupt handler.
}

//*****************************************************************************
//
// Interrupt handler for motor 2.
//
//*****************************************************************************
void GndMtr2IntHandler(void)
{
    // TODO: Define Ground Motor 2 interrupt handler.
}

//*****************************************************************************
//
// Interrupt handler for Solar Panel ADC.
//
//*****************************************************************************
void SPIntHandler(void)
{

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
void Timer2AInterrupt(void)
{
    uint32_t timerStatus;

    //
    // Get the timer interrupt status and clear the interrupt.
    timerStatus = TimerIntStatus(USONIC_TIMER2, true);
    TimerIntClear(USONIC_TIMER2, timerStatus);

    //
    // Check if this is the rising or falling edge.
    if (g_TimerFirstPass)
    {
        //
        // This is the first time through, it is a rising edge.
        g_TimerRiseValue = TimerValueGet(USONIC_TIMER2, TIMER_A);

        //
        // Set first pass to false.
        g_TimerFirstPass = false;
    }
    else
    {
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
void Timer2BInterrupt(void)
{
    uint32_t timerStatus;

    //
    // Get the timer interrupt status and clear the interrupt.
    timerStatus = TimerIntStatus(USONIC_TIMER2, true);
    TimerIntClear(USONIC_TIMER2, timerStatus);

    //
    // Check if this is the rising or falling edge.
    if (g_TimerFirstPass)
    {
        //
        // This is the first time through, it is a rising edge.
        g_TimerRiseValue = TimerValueGet(USONIC_TIMER2, TIMER_B);

        //
        // Set first pass to false.
        g_TimerFirstPass = false;
    }
    else
    {
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
void Timer3AInterrupt(void)
{
    uint32_t timerStatus;

    //
    // Get the timer interrupt status and clear the interrupt.
    timerStatus = TimerIntStatus(USONIC_TIMER3, true);
    TimerIntClear(USONIC_TIMER3, timerStatus);

    //
    // Check if this is the rising or falling edge.
    if (g_TimerFirstPass)
    {
        //
        // This is the first time through, it is a rising edge.
        g_TimerRiseValue = TimerValueGet(USONIC_TIMER3, TIMER_A);

        //
        // Set first pass to false.
        g_TimerFirstPass = false;
    }
    else
    {
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
void Timer3BInterrupt(void)
{
    uint32_t timerStatus;

    //
    // Get the timer interrupt status and clear the interrupt.
    timerStatus = TimerIntStatus(USONIC_TIMER3, true);
    TimerIntClear(USONIC_TIMER3, timerStatus);

    //
    // Check if this is the rising or falling edge.
    if (g_TimerFirstPass)
    {
        //
        // This is the first time through, it is a rising edge.
        g_TimerRiseValue = TimerValueGet(USONIC_TIMER3, TIMER_B);

        //
        // Set first pass to false.
        g_TimerFirstPass = false;
    }
    else
    {
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
void Timer1AInterrupt(void)
{
    uint32_t timerStatus;

    //
    // Get the timer interrupt status and clear the interrupt.
    timerStatus = TimerIntStatus(USONIC_TIMER1, true);
    TimerIntClear(USONIC_TIMER1, timerStatus);

    //
    // Check if this is the rising or falling edge.
    if (g_TimerFirstPass)
    {
        //
        // This is the first time through, it is a rising edge.
        g_TimerRiseValue = TimerValueGet(USONIC_TIMER1, TIMER_A);

        //
        // Set first pass to false.
        g_TimerFirstPass = false;
    }
    else
    {
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
void Timer1BInterrupt(void)
{
    uint32_t timerStatus;

    //
    // Get the timer interrupt status and clear the interrupt.
    timerStatus = TimerIntStatus(USONIC_TIMER1, true);
    TimerIntClear(USONIC_TIMER1, timerStatus);

    //
    // Check if this is the rising or falling edge.
    if (g_TimerFirstPass)
    {
        //
        // This is the first time through, it is a rising edge.
        g_TimerRiseValue = TimerValueGet(USONIC_TIMER1, TIMER_B);

        //
        // Set first pass to false.
        g_TimerFirstPass = false;
    }
    else
    {
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
// Interrupt handler for the Solenoid timer in order to turn off the solenoid
// after a short time.
//
//*****************************************************************************
void SolenoidInterrupt(void)
{
    uint32_t timerStatus;

    //
    // Get the timer interrupt status and clear the interrupt.
    timerStatus = TimerIntStatus(SOLENOID_TIMER, true);
    TimerIntClear(SOLENOID_TIMER, timerStatus);

    //
    // Deactivate the solenoid enable pins.
    DeactivateSolenoids();

    //
    // Disable the timer.
    TimerDisable(SOLENOID_TIMER, TIMER_A);
}

//*****************************************************************************
//
// Interrupt handler for the BMI160 sensor unit.
//
//*****************************************************************************
void BMI160IntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrupt status.
    ui32Status = GPIOIntStatus(BOOST_GPIO_PORT_INT, true);

    //
    // Clear the interrupt.
    GPIOIntClear(BOOST_GPIO_PORT_INT, ui32Status);

    //
    // Check which interrupt fired.
    if (ui32Status == BOOST_GPIO_INT)
    {
        //
        // IMU data is ready.
        g_IMUDataFlag = true;
    }
    else if (ui32Status == MAG_GPIO_INT)
    {
        //
        // Magnetometer data is ready.
        g_MagDataFlag = true;
    }


}

//*****************************************************************************
//
// Interrupt handler for the BME280 sensor unit.
//
//*****************************************************************************
void BME280IntHandler(void)
{
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

/*
 * Other functions used by main.
 */
//*****************************************************************************
//
// This function will turn on the associated LED.
// Parameter: LEDNum - the desired LED number to turn on.
//
//*****************************************************************************
void TurnOnLED(uint32_t LEDNum)
{
    //
    // Turn on the associated LED number.
    switch(LEDNum)
    {
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
void TurnOffLED(uint32_t LEDNum)
{
    //
    // Turn on the associated LED number.
    switch(LEDNum)
    {
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
void WaitForButtonPress(uint8_t desiredButtonState)
{
    uint8_t actualButtonState;
    uint8_t rawButtonState;
    uint8_t *pRawButtonState = &rawButtonState;
    uint8_t delta;
    uint8_t *pDelta = &delta;

    //
    // Get the state of the buttons.
    actualButtonState = ButtonsPoll(pDelta, pRawButtonState);

    if (desiredButtonState == LEFT_BUTTON)
    {
        while(actualButtonState != LEFT_BUTTON)
        {
            actualButtonState = ButtonsPoll(pDelta, pRawButtonState) & LEFT_BUTTON;
        }
        return;
    }
    else if (desiredButtonState == RIGHT_BUTTON)
    {
        while(actualButtonState != RIGHT_BUTTON)
        {
            actualButtonState = ButtonsPoll(pDelta, pRawButtonState) & RIGHT_BUTTON;
        }
        return;
    }
    else if (desiredButtonState == ALL_BUTTONS)
    {
        while(actualButtonState != ALL_BUTTONS)
        {
            actualButtonState = ButtonsPoll(pDelta, pRawButtonState) & ALL_BUTTONS;
        }
        return;
    }
}

//*****************************************************************************
//
// This function will handle analysis of characters received from the console.
//
//*****************************************************************************
void Menu(char charReceived)
{
    //
    // Check the character received.
    switch(charReceived)
    {
    case 'Q': // Quit the program
    {
        g_Quit = true;
        break;
    }
    case 'P': // Print Raw GPS data.
    {
        if (g_PrintRawGPS)
            g_PrintRawGPS = false;
        else
            g_PrintRawGPS = true;

        break;
    }
    case 'B': // Print raw accel, gyro and mag data.
    {
        if (g_PrintRawBMIData)
            g_PrintRawBMIData = false;
        else
            g_PrintRawBMIData = true;

        break;
    }
    case 'M': // Print Menu.
    {
        UARTprintf("Menu:\r\nM - Print this menu.\r\n");
        UARTprintf("P - Print raw GPS data.\r\n");
        UARTprintf("B - Print raw accel, gyro and mag data.\r\n");
        UARTprintf("A - Trigger solar panel ADC.\r\n");
        UARTprintf("1 - Trigger ultra sonic sensor #1.\r\n");
        UARTprintf("2 - Trigger ultra sonic sensor #2.\r\n");
        UARTprintf("Y - Activate solenoid enable pins.\r\n");
        UARTprintf("X - Stop the motors.\r\n");
        UARTprintf("Q - Quit this program.\r\n");
        break;
    }
    case 'A': // Trigger solar panel ADC.
    {
        if (ADCBusy(SP_ADC))
        {
            UARTprintf("ADC Busy!\r\n");
        }
        else
        {
            ADCProcessorTrigger(SP_ADC, 0);
        }
        break;
    }
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
#if DEBUG
    case 'W': // Increase throttle of air motors.
        {
            g_mtrThrottle += 100;
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, g_mtrThrottle);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, g_mtrThrottle);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, g_mtrThrottle);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, g_mtrThrottle);
            UARTprintf("Throttle Increase: %d\r\n", g_mtrThrottle);
            break;
        }
        case 'S': // Decrease throttle of air motors.
        {
            g_mtrThrottle -= 100;
            if (g_mtrThrottle < ZEROTHROTTLE)
                g_mtrThrottle += 100;
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, g_mtrThrottle);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, g_mtrThrottle);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, g_mtrThrottle);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, g_mtrThrottle);
            UARTprintf("Throttle Decrease: %d\r\n", g_mtrThrottle);
            break;
        }
        case 'X': // kill the throttle.
        {
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, ZEROTHROTTLE);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, ZEROTHROTTLE);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, ZEROTHROTTLE);
            PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, ZEROTHROTTLE);
            g_mtrThrottle = ZEROTHROTTLE;
            break;
        }
#endif
    case 'Y': // Activate the solenoids
    {
        ActivateSolenoids();
        break;
    }
    }
    //
    // Reset the flag.
    g_ConsoleFlag = false;
}

//*****************************************************************************
//
// This function will forward all data from the GPS to the Console.
//
//*****************************************************************************
void ProcessGPS(void)
{
    char currChar;
    char UTC[10];
    char Lat[10];
    char Long[11];
    char alt[6];
    char NorS = '0';
    char EorW = '0';
    int n = 0;
    int i = 0;

    char Buffer[100] = {0};

    while(UARTCharsAvail(GPS_UART))
    {
        currChar = UARTCharGet(GPS_UART);
        if (g_PrintRawGPS)
        {
            UARTCharPut(CONSOLE_UART, currChar);
        }

        //
        // Try to find the GPGGA string.
        if (currChar == 'G')
        {
            currChar = UARTCharGet(GPS_UART);
            if (currChar == 'P')
            {
                currChar = UARTCharGet(GPS_UART);
                if (currChar == 'G')
                {
                    currChar = UARTCharGet(GPS_UART);
                    if (currChar =='G')
                    {
                        currChar = UARTCharGet(GPS_UART);
                        if (currChar == 'A')
                        {
                            //
                            // Dump the comma.
                            UARTCharGet(GPS_UART);

                            for (n = 0; n < sizeof(Buffer); n++)
                            {
                                currChar = UARTCharGet(GPS_UART);

                                Buffer[n] = currChar;
                            }
                        }
                    }
                }
            }
        }
    }

    if (Buffer[0] != ',' && Buffer[0] != 0x00)
    {
        //
        // Set the flag that the GPS is connected.
        g_GPSConnected = true;

        //
        // Grab the UTC
        for (n = 0; n < 9; n++)
        {
            UTC[n] = Buffer[n];
        }

        //
        // Skip comma.
        n++;

        //
        // Grab the latitude.
        for (i = 0; i < 10; i++)
        {
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
        for (i = 0; i < 11; i++)
        {
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
        for (i = 0; i < 6; i++)
        {
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

        if (NorS == 'N')
        {
            //
            // Positive latitude.
            g_Pack.pack.lat = temp2 + (temp3 / 60);
        }
        else if (NorS == 'S')
        {
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

        if (EorW == 'E')
        {
            //
            // Positive longitude.
            g_Pack.pack.lon = temp2 + temp3 / 60;
        }
        else if (EorW == 'W')
        {
            //
            // Negative longitude.
            g_Pack.pack.lon = (-1) * (temp2 + (temp3 / 60));
        }

        //
        // Save the altitude.
        g_Pack.pack.alt = atof(alt);
    }
    else
    {
        //
        // GPS is not connected.
        g_GPSConnected = false;
    }

    //
    // Reset flag.
    g_GPSFlag = false;

    //
    // Turn on LED 3 to show that GPS has a lock.
    if (g_GPSConnected)
    {
        TurnOnLED(3);
    }
    else
    {
        TurnOffLED(3);
    }
}

//*****************************************************************************
//
// This function will forward all radio data to the console.
//
//*****************************************************************************
void ProcessRadio(void)
{
    switch (g_sRxPack.ui8Data[3]) {
    case 'T':
    {
        //
        // Change the status of the platform.
        sStatus.bMode = true;

        //
        // Make sure main() knows the data is good.
        sStatus.bGoodRadioData = true;

        break;
    }
    case 'C':
    {
        //
        // Change the status of the platform.
        sStatus.bMode = false;

        //
        // Tell main() we are getting good data.
        sStatus.bGoodRadioData = true;

        //
        // Check if we are flying or driving and update the Status.
        if (g_sRxPack.sControlPacket.flyordrive == g_sRxPack.sControlPacket.fdConfirm)
        	if (g_sRxPack.sControlPacket.flyordrive == 'D')
        		sStatus.bFlyOrDrive = false;
        	else if (g_sRxPack.sControlPacket.flyordrive == 'F')
        		sStatus.bFlyOrDrive = true;

        break;
    }
    case '0':
    {
        //
        // Change the status of the platform.
        sStatus.bMode = true;

        //
        // Receiving bad data. Tell main to ignore it.
        sStatus.bGoodRadioData = false;

        break;
    }
    }

    //
    // Reset the connection lost timer.
    // TODO: Set up a connection lost timeout timer.

    //
    // Reset the flag.
    g_RadioFlag = false;
}

//*****************************************************************************
//
// This function will evaluate the data received in the ADC.
//
//*****************************************************************************
void ProcessADC(void)
{
    uint32_t ADCData[8];

    //
    // Get the data.
    ADCSequenceDataGet(SP_ADC, 0, ADCData);

    //
    // Print out the data to the console.
    UARTprintf("SP1 = %d\r\n", ADCData[0]);
    UARTprintf("SP2 = %d\r\n", ADCData[1]);
    UARTprintf("SP3 = %d\r\n", ADCData[2]);
    UARTprintf("SP4 = %d\r\n", ADCData[3]);
    UARTprintf("SP5 = %d\r\n", ADCData[4]);

    //
    // Reset the flag.
    g_ADCFlag = false;
}

//*****************************************************************************
//
// This function will determine how far away an obstacle is.
//
//*****************************************************************************
int ProcessUltraSonic(uint32_t SysClockSpeed)
{
    float distance;
    int deltaT;

    //
    // Calculate the timer between edges.
    deltaT = g_TimerRiseValue - g_TimerFallValue;

    if (deltaT < 0)
    {
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

    UARTprintf("deltaT = %d\r\n", deltaT);

    return distance;
}

//*****************************************************************************
//
// This function will drive the solenoid enable pins to a high state.
//
//*****************************************************************************
void ActivateSolenoids(void)
{
    //
    // Drive the solenoid pins to a high value.
    GPIOPinWrite(SOLENOID_GPIO_PORT1, SOLENOID_PIN_1, SOLENOID_PIN_1);
    GPIOPinWrite(SOLENOID_GPIO_PORT2, SOLENOID_PIN_2, SOLENOID_PIN_2);

    //
    // Turn on USER LED 2 to signal payload deployment.
    TurnOnLED(2);

    //
    // Enable timer 4, so that the pins will be turned off shortly.
    TimerEnable(SOLENOID_TIMER, TIMER_A);

    UARTprintf("Deploying payload...\r\n");
}

//*****************************************************************************
//
// This function will drive the solenoid enable pins to a low state. This
// function should only be called by the timer 4 interrupt.
//
//*****************************************************************************
void DeactivateSolenoids(void)
{
    //
    // Drive the solenoid pins to a high value.
    GPIOPinWrite(SOLENOID_GPIO_PORT1, SOLENOID_PIN_1, 0x00);
    GPIOPinWrite(SOLENOID_GPIO_PORT2, SOLENOID_PIN_2, 0x00);

    //
    // Turn off USER LED 2.
    TurnOffLED(2);

    UARTprintf("Payload Deployed!\r\n");

    //
    // Update system status.
    sStatus.bPayDeployed = true;
}

//*****************************************************************************
//
// This function will drive the solenoid enable pins to a low state. This
// function should only be called by the timer 4 interrupt.
//
//*****************************************************************************
void SendPacket(void)
{
    int n;

    //
    // Dummy test values.
    //g_Pack.pack.UTC = 12345.34;
    //g_Pack.pack.lat = 34.234;
    //g_Pack.pack.lon = 23.234;

    g_Pack.pack.velX = 10;
    g_Pack.pack.velY = 20;
    g_Pack.pack.velZ = 30;
    g_Pack.pack.posX = 40;
    g_Pack.pack.posY = 50;
    g_Pack.pack.posZ = 60;

    //
    // Current orientation.
    g_Pack.pack.roll = sStatus.fRoll;
    g_Pack.pack.pitch = sStatus.fPitch;
    g_Pack.pack.yaw = sStatus.fYaw;

    //
    // Mode of operation.
    if (g_sRxPack.sControlPacket.flyordrive == g_sRxPack.sControlPacket.fdConfirm)
    	g_Pack.pack.movement = g_sRxPack.sControlPacket.flyordrive;

    //
    // Status bits.
    g_Pack.pack.gndmtr1 = false;
    g_Pack.pack.gndmtr2 = false;
    g_Pack.pack.amtr1 = false;
    g_Pack.pack.amtr2 = false;
    g_Pack.pack.amtr3 = false;
    g_Pack.pack.amtr4 = false;
    g_Pack.pack.uS1 = false;
    g_Pack.pack.uS2 = false;
    g_Pack.pack.uS3 = false;
    g_Pack.pack.uS4 = false;
    g_Pack.pack.uS5 = false;
    g_Pack.pack.uS6 = false;
    g_Pack.pack.payBay = false;

    //
    // Send the data over the radio.
    for (n = 0; n < sizeof(g_Pack.str); n++)
    {
        UARTCharPut(RADIO_UART, g_Pack.str[n]);
    }

    //
    // Reset the flag.
    g_SendPacket = false;
}

//*****************************************************************************
//
// This function will retrieve the accel or gyro data from the BMI160.
//
//*****************************************************************************
void ProcessIMUData(void)
{
    uint8_t status;
    uint8_t IMUData[12]; // raw accel and gyro data
    int16_t accelIntData[3];
    int16_t *p_accelX = &accelIntData[0];
    int16_t *p_accelY = &accelIntData[1];
    int16_t *p_accelZ = &accelIntData[2];

    //
    // First check the status for which data is ready.
    I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_STATUS, 1, &status);

    //
    // Check what status returned.
    if ((status & 0xC0) == (BMI160_ACC_RDY | BMI160_GYR_RDY))
    {
        //
        // Then get the data for both the accel and gyro.
        I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_GYRO_X, 12, IMUData);

        //
        // Set the gyro data to the global variables.
        g_gyroDataRaw[0] = ((IMUData[1] << 8) + IMUData[0]) - g_Bias[3];
        g_gyroDataRaw[1] = ((IMUData[3] << 8) + IMUData[2]) - g_Bias[4];
        g_gyroDataRaw[2] = ((IMUData[5] << 8) + IMUData[4]) - g_Bias[5];

        //
        // Convert data to float.
        g_fGyroData[0] = ((float)(g_gyroDataRaw[0])) / g_fGyroLSB;
        g_fGyroData[1] = ((float)(g_gyroDataRaw[1])) / g_fGyroLSB;
        g_fGyroData[2] = ((float)(g_gyroDataRaw[2])) / g_fGyroLSB;

        //
        // Set the accelerometer data.
        *p_accelX = ((IMUData[7] << 8) + IMUData[6]) - g_Bias[0];
        *p_accelY = ((IMUData[9] << 8) + IMUData[8]) - g_Bias[1];
        *p_accelZ = ((IMUData[11] << 8) + IMUData[10]) - g_Bias[2];

        //
        // Compute the accel data into floating point values.
        g_Pack.pack.accelX = ((float)accelIntData[0]) / g_accelLSBg;
        g_Pack.pack.accelY = ((float)accelIntData[1]) / g_accelLSBg;
        g_Pack.pack.accelZ = ((float)accelIntData[2]) / g_accelLSBg;

        //
        // Check if this is the first time.
        if (g_bDCMStarted == 0)
        {
            //
            // Start the DCM.
            CompDCMAccelUpdate(&g_sCompDCMInst, g_Pack.pack.accelX, g_Pack.pack.accelY,
            		g_Pack.pack.accelZ);
            CompDCMGyroUpdate(&g_sCompDCMInst, g_fGyroData[0], g_fGyroData[1], g_fGyroData[2]);

            CompDCMStart(&g_sCompDCMInst);
        }
        else
        {
        	//
        	// DCM is already started, just update it.
        	CompDCMAccelUpdate(&g_sCompDCMInst, g_Pack.pack.accelX, g_Pack.pack.accelY,
        	            		g_Pack.pack.accelZ);
			CompDCMGyroUpdate(&g_sCompDCMInst, g_fGyroData[0], g_fGyroData[1], g_fGyroData[2]);

            CompDCMUpdate(&g_sCompDCMInst);
        }

        //
        // Get the Euler angles.
        CompDCMComputeEulers(&g_sCompDCMInst, &sStatus.fRoll, &sStatus.fPitch,
                                   &sStatus.fYaw);

        //
        // Flip the roll axis. Positive is roll right.
        sStatus.fRoll *= -1;


        //
		// Convert Eulers to degrees. 180/PI = 57.29...
		// Convert Yaw to 0 to 360 to approximate compass headings.
        sStatus.fRoll *= 57.295779513082320876798154814105f;
        sStatus.fPitch *= 57.295779513082320876798154814105f;
        sStatus.fYaw *= 57.295779513082320876798154814105f;
		if(sStatus.fYaw < 0)
		{
			sStatus.fYaw += 360.0f;
		}

        //
        // Loop counter print once per second.
        if (g_loopCount && g_PrintRawBMIData)
        {
            UARTprintf("Accelx = %d\r\nAccely = %d\r\n", accelIntData[0], accelIntData[1]);
            UARTprintf("Accelz = %d\r\n", accelIntData[2]);
            UARTprintf("Gyrox = %d\r\nGyroy = %d\r\n", g_gyroDataRaw[0], g_gyroDataRaw[1]);
            UARTprintf("Gyroz = %d\r\n", g_gyroDataRaw[2]);

            //
            // Reset loop count.
            g_loopCount = false;
        }
    }

    //
    // Reset the flag
    g_IMUDataFlag = false;

    //
    // Blink the LED 1 to indicate sensor is working.
    if (g_LED1On)
    {
        TurnOffLED(1);
        g_LED1On = false;
    }
    else
    {
        TurnOnLED(1);
        g_LED1On = true;
    }
}

//*****************************************************************************
//
// This function will retrieve the mag data from the BMM150.
//
//*****************************************************************************
void ProcessMagData(void)
{
    uint8_t IMUData[8];
    uint8_t status;

    //
    // First check the status for which data is ready.
    I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_STATUS, 1, &status);

    if((status && 0x20) == BMI160_MAG_RDY)
    {
        //
        // Magnetometer data is ready. So get it.
        I2CRead(BOOST_I2C, BMI160_ADDRESS, BMI160_MAG_X, 4, IMUData);

        //
        // Assign it to global variables.
        g_magDataRaw[0] = (IMUData[1] << 8) + IMUData[0];
        g_magDataRaw[1] = (IMUData[3] << 8) + IMUData[2];
        g_magDataRaw[2] = (IMUData[5] << 8) + IMUData[4];
        g_magDataRaw[3] = (IMUData[7] << 8) + IMUData[6];

        //
        // Loop counter, print once per second.
        if (g_loopCount && g_PrintRawBMIData)
        {
            UARTprintf("MagX = %d\r\nMagY = %d\r\n", g_magDataRaw[0], g_magDataRaw[1]);
            UARTprintf("MagZ = %d\r\n", g_magDataRaw[2]);

            //
            // Reset loop count.
            g_loopCount = false;
        }
    }

    //
    // Reset flag.
    g_MagDataFlag = false;
}

//*****************************************************************************
//
// This function will update the trajectory of the platform.
//
//*****************************************************************************
void UpdateTrajectory(void)
{
    //
    // TODO: This is where the control law and stuff will go.

    //
    // Check if we are autonomous or manual.
    if (!sStatus.bMode)
    {
        //
        // Operating in manual mode.
        // Check if we are flying or driving.
        if (!sStatus.bFlyOrDrive)
        {
        	UARTprintf("Driving.\r\n");
            //
            // We are driving. Set the parameters sent from the radio.
        	// Get the wheel throttles.
        	int32_t ui32RWThrottle = (int32_t)(g_sRxPack.sControlPacket.throttle * 100);
        	int32_t ui32LWThrottle = (int32_t)(g_sRxPack.sControlPacket.throttle2 * 100);

        	UARTprintf("RW Throttle: %d\r\nLW Throttle: %d\r\n", ui32RWThrottle, ui32LWThrottle);

            //
            // TODO: Ground travel logic.
        }
        else
        {
        	UARTprintf("Flying.\r\n");
            //
            // We are flying. Set the parameters sent from the radio.
        	// Get the throttle.
        	uint32_t ui32Throttle = (int32_t)(g_sRxPack.sControlPacket.throttle * 100);

        	if(g_sRxPack.sControlPacket.throttle <= 0)
        	{
        		ui32Throttle = 0;
        		sThrottle.fAirMtr1Throttle = (ui32Throttle) * 100 + ZEROTHROTTLE;
        		sThrottle.fAirMtr2Throttle = (ui32Throttle) * 100 + ZEROTHROTTLE;
        	    sThrottle.fAirMtr3Throttle = (ui32Throttle) * 100 + ZEROTHROTTLE;
        	    sThrottle.fAirMtr4Throttle = (ui32Throttle) * 100 + ZEROTHROTTLE;
        	}
        	else
        	{
        		sThrottle.fAirMtr1Throttle = (ui32Throttle) * 100 + ZEROTHROTTLE;
        		sThrottle.fAirMtr2Throttle = (ui32Throttle) * 100 + ZEROTHROTTLE;
        		sThrottle.fAirMtr3Throttle = (ui32Throttle) * 100 + ZEROTHROTTLE;
        		sThrottle.fAirMtr4Throttle = (ui32Throttle) * 100 + ZEROTHROTTLE;
        		UARTprintf("Throttle: %d\r\n", ui32Throttle);

        		//
        		// Get the roll, pitch, yaw.
        		int32_t ui32Roll = (int32_t)(g_sRxPack.sControlPacket.roll * 100);
        		int32_t ui32Pitch = (int32_t)(g_sRxPack.sControlPacket.pitch * 100);
        		int32_t ui32Yaw = (int32_t)(g_sRxPack.sControlPacket.yaw);

        		UARTprintf("Roll: %d\r\nPitch: %d\r\nYaw: %d\r\n", ui32Roll, ui32Pitch, ui32Yaw);

        		//
        		// Check if the pitch error is less than 0.5 or -0.5.
        		if ((sStatus.fPitch - g_sRxPack.sControlPacket.pitch > 0.5f) || (sStatus.fPitch - g_sRxPack.sControlPacket.pitch < -0.5f))
        		{
        			//
        			// Check the pitch.
        			// Pitch is less than desired and negative.
        			if ((sStatus.fPitch < g_sRxPack.sControlPacket.pitch) && (sStatus.fPitch < 0))
        			{
        				//
        				// "Pull Up", increase front motor throttle and decrease back motor throttle.
        				sThrottle.fAirMtr1Throttle += 500;
        				sThrottle.fAirMtr3Throttle -= 500;

        				UARTprintf("Neg Pitch and Pull Up\r\n");
        			}
        			//
        			// Pitch is greater than desired and negative.
        			else if ((sStatus.fPitch > g_sRxPack.sControlPacket.pitch) && (sStatus.fPitch < 0))
        			{
        				//
        				// "Pull Down", decrease front motor throttle and increase back motor throttle.
        				sThrottle.fAirMtr1Throttle -= 500;
        				sThrottle.fAirMtr3Throttle += 500;

        				UARTprintf("Neg Pitch and Pull Down\r\n");
        			}
        			//
        			// Pitch is less than desired and positive.
        			else if ((sStatus.fPitch < g_sRxPack.sControlPacket.pitch) && (sStatus.fPitch > 0))
        			{
        				//
        				// "Pull Up", increase front motor throttle and decrease back motor throttle.
        				sThrottle.fAirMtr1Throttle += 500;
        				sThrottle.fAirMtr3Throttle -= 500;

        				UARTprintf("Pos Pitch and Pull Up\r\n");
        			}
        			//
        			// Pitch is greater than desired and positive.
        			else if ((sStatus.fPitch > g_sRxPack.sControlPacket.pitch) && (sStatus.fPitch > 0))
        			{
        				//
        				// "Pull Down", decrease front motor throttle and increase back motor throttle.
        				sThrottle.fAirMtr1Throttle -= 500;
        				sThrottle.fAirMtr3Throttle += 500;

        				UARTprintf("Pos Pitch and Pull Down\r\n");
        			}
        		}

        		//
        		// Check if roll error is greater than 0.5 degrees.
        		if ((sStatus.fRoll - g_sRxPack.sControlPacket.roll > 0.5f) || (sStatus.fRoll - g_sRxPack.sControlPacket.roll < -0.5f))
        		{
        			//
        			// Check the roll.
        			// Roll is less than desired and negative.
        			if ((sStatus.fRoll < g_sRxPack.sControlPacket.roll) && (sStatus.fRoll < 0))
        			{
        				//
        				// "Roll right", increase left motor throttle and decrease right motor throttle.
        				sThrottle.fAirMtr2Throttle -= 500;
        				sThrottle.fAirMtr4Throttle += 500;

        				UARTprintf("Neg Roll and Roll Right\r\n");
        			}
        			//
        			// Roll is greater than desired and negative.
        			else if ((sStatus.fRoll > g_sRxPack.sControlPacket.roll) && (sStatus.fRoll < 0))
        			{
        				//
        				// "Roll Left", increase right motor throttle and decrease left motor throttle.
        				sThrottle.fAirMtr2Throttle += 500;
        				sThrottle.fAirMtr4Throttle -= 500;

        				UARTprintf("Neg Roll and Roll Left\r\n");
        			}
        			//
        			// Roll is less than desired and positive.
        			else if ((sStatus.fRoll < g_sRxPack.sControlPacket.roll) && (sStatus.fRoll > 0))
        			{
        				//
        				// "Roll Right", increase left motor throttle and decrease right motor throttle.
        				sThrottle.fAirMtr2Throttle -= 500;
        				sThrottle.fAirMtr4Throttle += 500;

        				UARTprintf("Pos Roll and Roll Right\r\n");
        			}
        			//
        			// Roll is greater than desired and positive.
        			else if ((sStatus.fRoll > g_sRxPack.sControlPacket.roll) && (sStatus.fRoll > 0))
        			{
        				//
        				// "Roll Left", decrease left motor throttle and increase right motor throttle.
        				sThrottle.fAirMtr2Throttle += 500;
        				sThrottle.fAirMtr4Throttle -= 500;

        				UARTprintf("Pos Roll and Roll Left\r\n");
        			}
        		}
        	}
        	//
        	// TODO: What about yaw?

        	//
        	// Set the new throttles for the motors.
        	PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, sThrottle.fAirMtr1Throttle);
			PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, sThrottle.fAirMtr2Throttle);
			PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, sThrottle.fAirMtr3Throttle);
			PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, sThrottle.fAirMtr4Throttle);
        }

    }
    else
    {
        //
        // Check if radio is sending good data.
        if (sStatus.bGoodRadioData)
        {
            //
            // Radio data is good, calculate a trajectory.

            //
            // TODO: Calculate a trajectory.
        }
        else
        {
            //
            // Radio data is bad, do nothing.

            //
            // TODO: Add some logic, so that if we lose radio contact, we
            // don't necessarily crash...
        }
    }

}
