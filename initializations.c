/*
 * initializations.c
 *
 *  Created on: Jan 20, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Function definitions for all initializations of all peripherals.
 *      This will aid in simplicity for the main function.
 */

//*****************************************************************************
//
// Includes
//
//*****************************************************************************
#include "APOPHIS_pin_map.h"
#include "initializations.h"

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#include "bmi160.h"
#include "bme280.h"
#include "i2c_driver.h"

//*****************************************************************************
//
// External Function Declarations for IntRegister() functions.
//
//*****************************************************************************
extern void SysTickIntHandler(void);
extern void ConsoleIntHandler(void);
extern void RadioIntHandler(void);
extern void GPSIntHandler(void);
extern void GndMtr1IntHandler(void);
extern void GndMtr2IntHandler(void);
extern void SPIntHandler(void);
extern void Timer2AInterrupt(void);
extern void Timer2BInterrupt(void);
extern void Timer3AInterrupt(void);
extern void Timer3BInterrupt(void);
extern void Timer1AInterrupt(void);
extern void Timer1BInterrupt(void);
extern void SolenoidInterrupt(void);
extern void BMI160IntHandler(void);
extern void BME280IntHandler(void);

/*
 * LED Initialization function.
 */
//*****************************************************************************
//
// This function will initialize the 4 User LED pins. They are pins PN0, PN1
// PF0, and PF4.
//
//*****************************************************************************
void InitLED(uint32_t SysClockSpeed)
{
    //
    // Initialize the GPIO port for the LEDs.
    SysCtlPeripheralEnable(LED_GPIO_PERIPH1);
    SysCtlPeripheralEnable(LED_GPIO_PERIPH2);

    //
    // Configure the pins as output pins.
    GPIOPinTypeGPIOOutput(LED_PORT1, LED1_PIN | LED2_PIN);
    GPIOPinTypeGPIOOutput(LED_PORT2, LED3_PIN | LED4_PIN);

    //
    // Initialize a 1 second SysTick for blinking the LED pin 4 to indicate
    // program running.
    SysTickPeriodSet(SysClockSpeed);

    //
    // Register the interrupt handler for blinking the LED and enable it.
    SysTickIntRegister(SysTickIntHandler);
    SysTickIntEnable();
}

/*
 * UART Initialization Functions
 */
//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the program is running.
//
//*****************************************************************************
void InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(CONSOLE_GPIO_PERIPH);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    GPIOPinConfigure(CONSOLE_CONFIG_PINRX);
    GPIOPinConfigure(CONSOLE_CONFIG_PINTX);

    //
    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(CONSOLE_PERIPH);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(CONSOLE_UART, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(CONSOLE_PORT, CONSOLE_PINRX | CONSOLE_PINTX);

    //
    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);

    //
    // Enable the UART interrupt.
    IntEnable(CONSOLE_INT);
    UARTIntEnable(CONSOLE_UART, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(CONSOLE_UART, ConsoleIntHandler);
}

//*****************************************************************************
//
// This function sets up UART RX/TX communication to enable communication with
// the 3D Robotics Radio V2.
//
//*****************************************************************************
void InitRadio(uint32_t SysClockSpeed)
{
    //
    // Print initializing to the Console.
    UARTprintf("Initializing Radio...\n\r");

    //
    // Enable GPIO port K which is used for UART4 pins.
    SysCtlPeripheralEnable(RADIO_GPIO_PERIPH);

    //
    // Configure the pin muxing for UART4 functions on port K0 and K1.
    GPIOPinConfigure(RADIO_CONFIG_PINRX);
    GPIOPinConfigure(RADIO_CONFIG_PINTX);

    //
    // Enable UART4 so that we can configure the clock.
    SysCtlPeripheralEnable(RADIO_PERIPH);

    //
    // Select the alternate UART function for the Rx pin.
    GPIOPinTypeUART(RADIO_PORT, RADIO_PINRX | RADIO_PINTX);

    //
    // Configure UART6 for 57600, 8-N-1 operation.
    UARTConfigSetExpClk(RADIO_UART, SysClockSpeed, 57600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    IntEnable(RADIO_INT);
    UARTIntEnable(RADIO_UART, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(RADIO_UART, RadioIntHandler);

    //
    // Initialization complete. Print to console.
    UARTprintf("Done!\n\r");
}

//*****************************************************************************
//
// This function sets up UART RX communication to enable communication with
// the GP-20U7 GPS module.
//
//*****************************************************************************
void InitGPS(uint32_t SysClockSpeed)
{
    //
    // Print initializing to the Console.
    UARTprintf("Initializing GPS...\n\r");

    //
    // Enable GPIO port P which is used for UART6 pins.
    SysCtlPeripheralEnable(GPS_GPIO_PERIPH);

    //
    // Configure the pin muxing for UART6 functions on port P0.
    GPIOPinConfigure(GPS_CONFIG_PINRX);

    //
    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(GPS_PERIPH);

    //
    // Select the alternate UART function for the Rx pin.
    GPIOPinTypeUART(GPS_PORT, GPS_PINRX);

    //
    // Configure UART6 for 9600, 8-N-1 operation.
    UARTConfigSetExpClk(GPS_UART, SysClockSpeed, 9600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    IntEnable(GPS_INT);
    UARTIntEnable(GPS_UART, UART_INT_RX);
    UARTIntRegister(GPS_UART, GPSIntHandler);

    //
    // Initialization complete. Print to console.
    UARTprintf("Done!\n\r");
}

//*****************************************************************************
//
// This function sets up UART RX/TX communication to enable communication with
// the two Robotis RX24F motors.
//
//*****************************************************************************
void InitGndMotors(uint32_t SysClockSpeed)
{
    //
    // Print initializing to the Console.
    UARTprintf("Initializing Ground Motors...\n\r");

    //
    // Enable GPIO port C and D which is used for UART2 and UART7 pins.
    SysCtlPeripheralEnable(GNDMTR1_GPIO_PERIPH);
    SysCtlPeripheralEnable(GNDMTR2_GPIO_PERIPH);

    //
    // Configure the pin muxing for UART2 and UART7 functions on port A0 and A1.
    GPIOPinConfigure(GNDMTR1_CONFIG_PINRX);
    GPIOPinConfigure(GNDMTR1_CONFIG_PINTX);
    GPIOPinConfigure(GNDMTR2_CONFIG_PINRX);
    GPIOPinConfigure(GNDMTR2_CONFIG_PINTX);

    //
    // Enable UART2 and UART7 so that we can configure the clock.
    SysCtlPeripheralEnable(GNDMTR1_PERIPH);
    SysCtlPeripheralEnable(GNDMTR2_PERIPH);

    //
    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GNDMTR1_PORT, GNDMTR1_PINRX | GNDMTR1_PINTX);
    GPIOPinTypeUART(GNDMTR2_PORT, GNDMTR2_PINRX | GNDMTR2_PINTX);

    //
    // Configure UART6 for 115,200, 8-N-1 operation.
    // TODO: Check configuration parameters for the Ground Motors.
    UARTConfigSetExpClk(GNDMTR1_UART, SysClockSpeed, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
    UARTConfigSetExpClk(GNDMTR2_UART, SysClockSpeed, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    // TODO: Check interrupt initialization for ground motors.
    IntEnable(GNDMTR1_INT);
    UARTIntEnable(GNDMTR1_UART, UART_INT_RX | UART_INT_RT);

    IntEnable(GNDMTR2_INT);
    UARTIntEnable(GNDMTR2_UART, UART_INT_RX | UART_INT_RT);

    //
    // Initialization complete. Print to console.
    UARTprintf("Done!\n\r");
}





/*
 * ADC Initialization Function for Solar Panels.
 */
void InitSolarPanels(void)
{
    //
    // Print initializing to the Console.
    UARTprintf("Initializing Solar Panels...\n\r");

    //
    // The ADC0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SP_PERIPH);

    //
    // For the solar panels ADC0 is used with AIN0 on port E.
    SysCtlPeripheralEnable(SP_GPIO_PERIPH);

    //
    // Disable the ADC sequence for configuration.
    ADCSequenceDisable(SP_ADC, 0);

    //
    // Configure the clock as required for TM4C129xxx devices.
    ADCClockConfigSet(SP_ADC, ADC_CLOCK_SRC_MOSC | ADC_CLOCK_RATE_FULL, 1);

    //
    // Set the reference for the ADC as the internal 3V reference.
    ADCReferenceSet(SP_ADC, ADC_REF_INT);

    //
    // Select the analog ADC function for these pins.
    GPIOPinTypeADC(SP_PORT, SP1_PIN | SP2_PIN | SP3_PIN | SP4_PIN | SP5_PIN);

    //
    // Enable sample sequence 0 with a processor signal trigger.  Sequence 0
    // will do a single sample when the processor sends a signal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.
    ADCSequenceConfigure(SP_ADC, 0, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0-4 on sequence 0.  Sample channel 0,1,2,3,8 in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 0 (ADC_CTL_END).
    // Sequence 0 has 8 programmable steps.
    ADCSequenceStepConfigure(SP_ADC, 0, 0, SP1_CHANNEL);
    ADCSequenceStepConfigure(SP_ADC, 0, 1, SP2_CHANNEL);
    ADCSequenceStepConfigure(SP_ADC, 0, 2, SP3_CHANNEL);
    ADCSequenceStepConfigure(SP_ADC, 0, 3, SP4_CHANNEL);
    ADCSequenceStepConfigure(SP_ADC, 0, 4, SP5_CHANNEL | ADC_CTL_IE | ADC_CTL_END);

    //
    // Configure interrupts for the ADC.
    ADCIntEnable(SP_ADC, 0);
    ADCIntRegister(SP_ADC, 0, SPIntHandler);
    ADCIntClear(SP_ADC, 0);
    IntEnable(SP_INT);

    //
    // Since sample sequence 0 is now configured, it must be enabled.
    ADCSequenceEnable(SP_ADC, 0);

    // TODO: Check to make sure ADC is configured correctly. Cuz either it's not
    // or it's broken.
    //
    // Initialization complete. Print to console.
    UARTprintf("Done!\n\r");
}





/*
 * Initializations for Ultrasonic Sensors
 */
void InitUltraSonicSensor(void)
{
    UARTprintf("Initializing Ultrasonic Sensors...\n\r");

    //
    // Enable the peripheral used by timers 1, 2 and 3.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    //
    // Enable the peripherals used by the capture pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable the peripherals used by the GPIO Trigger pins.
    SysCtlPeripheralEnable(USONIC_TRIG_PERIPH);

    //
    // Configure the trigger pins as a standard output pin.
    GPIOPinTypeGPIOOutput(USONIC_TRIG_PORT, TRIG_PIN_1);
    GPIOPinTypeGPIOOutput(USONIC_TRIG_PORT, TRIG_PIN_2);
    GPIOPinTypeGPIOOutput(USONIC_TRIG_PORT, TRIG_PIN_3);
    GPIOPinTypeGPIOOutput(USONIC_TRIG_PORT, TRIG_PIN_4);
    GPIOPinTypeGPIOOutput(USONIC_TRIG_PORT, TRIG_PIN_5);
    GPIOPinTypeGPIOOutput(USONIC_TRIG_PORT, TRIG_PIN_6);

    /*
     * Configure Timer 2 for pins PM0, PM1.
     */
    //
    // Configure the GPIO pins for timer 2 as edge capture (PM0, PM1).
    GPIOPinConfigure(GPIO_PM0_T2CCP0);
    GPIOPinConfigure(GPIO_PM1_T2CCP1);
    GPIOPinTypeTimer(USONIC_TIMER2_PORT, CAP_PIN_1);
    GPIOPinTypeTimer(USONIC_TIMER2_PORT, CAP_PIN_2);

    //
    // Configure the timer to be a capture on timer A and B and trigger on both edges.
    TimerConfigure(USONIC_TIMER2, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME
                   | TIMER_CFG_B_CAP_TIME);
    TimerControlEvent(USONIC_TIMER2, TIMER_A, TIMER_EVENT_BOTH_EDGES);
    TimerControlEvent(USONIC_TIMER2, TIMER_B, TIMER_EVENT_BOTH_EDGES);

    //
    // Prescaler, to increase the time to reset of the timer so it can get longer ranges.
    TimerPrescaleSet(USONIC_TIMER2, TIMER_A, 10);
    TimerPrescaleSet(USONIC_TIMER2, TIMER_B, 10);

    //
    // Configure the interrupt for the timer capture.
    TimerIntClear(USONIC_TIMER2, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);
    TimerIntEnable(USONIC_TIMER2, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);
    IntEnable(INT_TIMER2A);
    IntEnable(INT_TIMER2B);

    TimerIntRegister(USONIC_TIMER2, TIMER_A, Timer2AInterrupt);
    TimerIntRegister(USONIC_TIMER2, TIMER_B, Timer2BInterrupt);

    /*
     * Configure Timer 3 for pins PM2, PA7
     */
    //
    // Configure the GPIO pins for timer 3 as edge capture (PM3, PA7).
    GPIOPinConfigure(GPIO_PM2_T3CCP0);
    GPIOPinConfigure(GPIO_PA7_T3CCP1);
    GPIOPinTypeTimer(GPIO_PORTM_BASE, CAP_PIN_3);
    GPIOPinTypeTimer(GPIO_PORTA_BASE, CAP_PIN_4);

    //
    // Configure the timer to be a capture on timer A and B and trigger on both edges.
    TimerConfigure(USONIC_TIMER3, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME
                   | TIMER_CFG_B_CAP_TIME);
    TimerControlEvent(USONIC_TIMER3, TIMER_A, TIMER_EVENT_BOTH_EDGES);
    TimerControlEvent(USONIC_TIMER3, TIMER_B, TIMER_EVENT_BOTH_EDGES);

    //
    // Prescaler, to increase the time to reset of the timer so it can get longer ranges.
    TimerPrescaleSet(USONIC_TIMER3, TIMER_A, 10);
    TimerPrescaleSet(USONIC_TIMER3, TIMER_B, 10);

    //
    // Configure the interrupt for the timer capture.
    TimerIntClear(USONIC_TIMER3, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);
    TimerIntEnable(USONIC_TIMER3, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);
    IntEnable(INT_TIMER3A);
    IntEnable(INT_TIMER3B);

    TimerIntRegister(USONIC_TIMER3, TIMER_A, Timer3AInterrupt);
    TimerIntRegister(USONIC_TIMER3, TIMER_B, Timer3BInterrupt);

    /*
     * Configure Timer 1 for pins PA3, PA2
     */
    //
    // Configure the GPIO pins for timer 3 as edge capture (PM3, PA7).
    GPIOPinConfigure(GPIO_PA2_T1CCP0);
    GPIOPinConfigure(GPIO_PA3_T1CCP1);
    GPIOPinTypeTimer(USOINC_TIMER1_PORT, CAP_PIN_5);
    GPIOPinTypeTimer(USOINC_TIMER1_PORT, CAP_PIN_6);

    //
    // Configure the timer to be a capture on timer A and B and trigger on both edges.
    TimerConfigure(USONIC_TIMER1, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME
                   | TIMER_CFG_B_CAP_TIME);
    TimerControlEvent(USONIC_TIMER1, TIMER_A, TIMER_EVENT_BOTH_EDGES);
    TimerControlEvent(USONIC_TIMER1, TIMER_B, TIMER_EVENT_BOTH_EDGES);

    //
    // Prescaler, to increase the time to reset of the timer so it can get longer ranges.
    TimerPrescaleSet(USONIC_TIMER1, TIMER_A, 10);
    TimerPrescaleSet(USONIC_TIMER1, TIMER_B, 10);

    //
    // Configure the interrupt for the timer capture.
    TimerIntClear(USONIC_TIMER1, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);
    TimerIntEnable(USONIC_TIMER1, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);
    IntEnable(INT_TIMER1A);
    IntEnable(INT_TIMER1B);

    TimerIntRegister(USONIC_TIMER1, TIMER_A, Timer1AInterrupt);
    TimerIntRegister(USONIC_TIMER1, TIMER_B, Timer1BInterrupt);

    //
    // Now enable all of the timers.
    TimerEnable(USONIC_TIMER2, TIMER_A);
    TimerEnable(USONIC_TIMER2, TIMER_B);
    TimerEnable(USONIC_TIMER3, TIMER_A);
    TimerEnable(USONIC_TIMER3, TIMER_B);
    TimerEnable(USONIC_TIMER1, TIMER_A);
    TimerEnable(USONIC_TIMER1, TIMER_B);

    UARTprintf("Done!\n\r");
}


/*
 * Initializations for Solenoid Enable Pins
 */
void InitSolenoidEnablePins(uint32_t SysClockSpeed)
{
    //
    // Enable the peripheral used by the Solenoid Enable pins.
    SysCtlPeripheralEnable(SOLENOID_PERIPH1);
    SysCtlPeripheralEnable(SOLENOID_PERIPH2);

    //
    // Configure the GPIO pins as output pins.
    GPIOPinTypeGPIOOutput(SOLENOID_GPIO_PORT1, SOLENOID_PIN_1);
    GPIOPinTypeGPIOOutput(SOLENOID_GPIO_PORT2, SOLENOID_PIN_2);

    //
    // Enable the peripheral to be used by the timer.
    SysCtlPeripheralEnable(SOLENOID_TIMER_PERIPH);

    //
    // Configure the timer to run for 5 seconds.
    TimerConfigure(SOLENOID_TIMER, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(SOLENOID_TIMER, TIMER_A, SysClockSpeed * 5);

    //
    // Configure the interrupts for the timer.
    TimerIntClear(SOLENOID_TIMER, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(SOLENOID_TIMER, TIMER_TIMA_TIMEOUT);
    IntEnable(SOLENOID_INT);
    TimerIntRegister(SOLENOID_TIMER, TIMER_A, SolenoidInterrupt);
}

/*
 * Initializations for the BMI 160.
 * All of the devices will be communicating over the I2C bus
 * on pins PB2 and PB3 - I2C0 and PC7 for the interrupt.
 * PB2 - I2C0 SDL
 * PB3 - I2C0 SDA
 * PC7 - INT1
 */

void InitIMU(uint32_t SysClockSpeed, uint8_t *offsetCompensation)
{
    UARTprintf("Initializing IMU...\n\r");

    //
    // Initialize the GPIO Peripherals used by this device.
    SysCtlPeripheralEnable(BOOST_GPIO_PERIPH1);
    SysCtlPeripheralEnable(BOOST_GPIO_PERIPH2);

    //
    // Initialize the I2C0 bus for configuration.
    SysCtlPeripheralEnable(BOOST_PERIPH);

    //
    // Configure the pins for GPIO I2C0 use.
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinTypeI2C(BOOST_GPIO_PORT_I2C, BOOST_GPIO_SDA);
    GPIOPinTypeI2CSCL(BOOST_GPIO_PORT_I2C, BOOST_GPIO_SCL);

    //
    // Configure the IMU interrupt on pin PC6.
    GPIOPinTypeGPIOInput(BOOST_GPIO_PORT_INT, BOOST_GPIO_INT);
    GPIOIntTypeSet(BOOST_GPIO_PORT_INT, BOOST_GPIO_INT, GPIO_RISING_EDGE);

    //
    // Configure the magnetometer interrupt on pin PC7.
    GPIOPinTypeGPIOInput(BOOST_GPIO_PORT_INT, MAG_GPIO_INT);
    GPIOIntTypeSet(BOOST_GPIO_PORT_INT, MAG_GPIO_INT, GPIO_RISING_EDGE);

    //
    // Enable and register the function for the interrupt.
    GPIOIntRegister(BOOST_GPIO_PORT_INT, BMI160IntHandler);
    GPIOIntEnable(BOOST_GPIO_PORT_INT, BOOST_GPIO_INT);

    //
    // Disable the I2C module for configuration.
    I2CMasterDisable(BOOST_I2C);

    //
    // Configure and Initialize the I2C bus at 400 kpbs.
    I2CMasterInitExpClk(BOOST_I2C, SysClockSpeed, true);

    //
    // Enable the I2C module.
    I2CMasterEnable(BOOST_I2C);

    //
    // Before calling the BMI160 initialize function, make sure the I2C
    // bus is not busy.
    while(I2CMasterBusBusy(BOOST_I2C))
    {
    }

    //
    // Initialize the BMI160 to have a 25Hz update rate for the
    // accelerometer and gyro and 16 Hz for the magnetometer,
    // +/-2g setting on the accelerometer and 2000 deg/s for the gyro.
    InitBMI160(BOOST_I2C, BMI160_ACC_25_HZ, BMI160_ACC_RANGE_2G, BMI160_GYR_25_HZ,
               BMI160_GYR_RATE_2000, BMI160_MAG_31_HZ, offsetCompensation);

    //
    // Initialize the BMM150 to have a 25 Hz update rate.
    InitBMM150(BOOST_I2C);

    //
    // Turn off interrupts, since I2CWrite turns them on.
    IntMasterDisable();

    UARTprintf("Done!\n\r");
}

/*
 * Initialization for the BME280 altimeter.
 *
 * BME280 does not feature interrupts. It must
 * be polled at a regular interval. The configuration
 * parameters are set for recommended for indoor
 * navigation. This results in an approximate
 * operational rate of 25Hz.
 */
void InitAltimeter(uint32_t SysClockSpeed, int8_t *offsetValues)
{
    UARTprintf("Initializing BME280...\n\r");

    //
    // Since BME280 does not have interrupts,
    // configure timer to poll data.
    SysCtlPeripheralEnable(BME280_TIMER_PERIPH);

    //
    // Configure and enable the timer.
    //
    // Configure the timer to run at 25 Hz.
    TimerConfigure(BME280_TIMER, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(BME280_TIMER, TIMER_A, SysClockSpeed / 25);

    //
    // Configure the interrupts for the timer.
    TimerIntClear(BME280_TIMER, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(BME280_TIMER, TIMER_TIMA_TIMEOUT);
    IntEnable(BME280_INT);
    TimerIntRegister(BME280_TIMER, TIMER_A, BME280IntHandler);

    //
    // Initialize the BME280 sensor.
    InitBME280(BOOST_I2C, offsetValues);

    //
    // Turn off interrupts, since I2CWrite turns them on.
    IntMasterDisable();

    //
    // Enable the timer.
    TimerEnable(BME280_TIMER, TIMER_A);

    UARTprintf("Done!\n\r");
}



/*
 * Initialization for the PWM module 0 for the
 * air motors. PWM M0 pins 0-3 wll be used.
 */
void InitAirMtrs(uint32_t sysClockSpeed)
{
    uint32_t speed;

    UARTprintf("Initializing air motors...\n\r");

    //
    // Turn on the peripherals for the PWM.
    SysCtlPeripheralEnable(PWM_PERIPHERAL);
    SysCtlPeripheralEnable(PWM_GPIO_PERIPH);

    //
    // Configure the GPIO pins.
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinTypePWM(PWM_GPIO_PORT1, PWM_MTR_1 | PWM_MTR_2 | PWM_MTR_3);
    GPIOPinTypePWM(PWM_GPIO_PORT2, PWM_MTR_4);

    //
    // Frequency of PWM.
    speed = (sysClockSpeed / PWM_FREQUENCY) - 1;

    //
    // Configure the PWM.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, speed);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, speed);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, speed);

    //
    // Initialize pulse to 50%
    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, ZEROTHROTTLE);
    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, ZEROTHROTTLE);
    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, ZEROTHROTTLE);
    PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, ZEROTHROTTLE);

    //
    // Set the output PWM modules.
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT |
                   PWM_OUT_4_BIT, true);

    UARTprintf("Done!\n\r");
}
