/*
 * APOPHIS_pin_map.h
 *
 *  Created on: Jan 20, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Define a standard naming convention for all used peripherals and their
 *      associated ports and pins.
 */

#ifndef APOPHIS_PIN_MAP_H_
#define APOPHIS_PIN_MAP_H_

/*
 * LED Peripheral Defines
 *
 * Pins: PN0, PN1, PF0, PF4
 */
#define LED_GPIO_PERIPH1 SYSCTL_PERIPH_GPION
#define LED_GPIO_PERIPH2 SYSCTL_PERIPH_GPIOF
#define LED_PORT1 GPIO_PORTN_BASE
#define LED_PORT2 GPIO_PORTF_BASE
#define LED1_PIN GPIO_PIN_1
#define LED2_PIN GPIO_PIN_0
#define LED3_PIN GPIO_PIN_4
#define LED4_PIN GPIO_PIN_0



/*
 * UART Peripheral Defines
 *
 *  UART0 Rx/Tx will be used for terminal communication with the PC.
 *  Pins: UART0RX - PA0, UART0TX - PA1
 *  Speed: 115,200 baud
 *  Size: 8 bit
 *  Stop: 1 stop
 *  Parity: no parity.
 *
 *  UART4 Rx/Tx will be used for radio communication.
 *  Pins UART4RX - PK0, UART4TX PK1
 *  Speed: 57600 baud
 *  Size: 8 bits
 *  Stop: 1 stop bit
 *  Parity: no parity
 *
 *  UART6 Rx will be used to communicate with the GPS - GP-20U7 peripheral.
 *  Pin: UART6RX - PP0
 *  Speed: 9600 baud
 *  Size: 8 bits
 *  Stop: 1 stop bit
 *  Parity: no parity
 *
 *  UART2 Rx/Tx will be used to communicate with Ground Motor 1 - Robitis RX24F.
 *  Pin: UART2RX - PD4, UART2TX - PD5
 *  Speed:
 *  Size:
 *  Stop:
 *  Parity:
 *
 *  UART7 Rx/Tx will be used to communicate with Ground Motor 2 - Robitis RX24F.
 *  Pins: UART7RX - PC4, UART7TX - PC5
 *  Speed:
 *  Size:
 *  Stop:
 *  Parity:
 */

//
// UART0 RX/TX - Console
#define CONSOLE_GPIO_PERIPH SYSCTL_PERIPH_GPIOA
#define CONSOLE_CONFIG_PINRX GPIO_PA0_U0RX
#define CONSOLE_CONFIG_PINTX GPIO_PA1_U0TX
#define CONSOLE_PORT GPIO_PORTA_BASE
#define CONSOLE_PINRX GPIO_PIN_0
#define CONSOLE_PINTX GPIO_PIN_1
#define CONSOLE_PERIPH SYSCTL_PERIPH_UART0
#define CONSOLE_UART UART0_BASE
#define CONSOLE_INT INT_UART0

//
// UART4 RX/TX - Radio
#define RADIO_GPIO_PERIPH SYSCTL_PERIPH_GPIOK
#define RADIO_CONFIG_PINRX GPIO_PK0_U4RX
#define RADIO_CONFIG_PINTX GPIO_PK1_U4TX
#define RADIO_PORT GPIO_PORTK_BASE
#define RADIO_PINRX GPIO_PIN_0
#define RADIO_PINTX GPIO_PIN_1
#define RADIO_PERIPH SYSCTL_PERIPH_UART4
#define RADIO_UART UART4_BASE
#define RADIO_INT INT_UART4

//
// UART6 RX - GPS
#define GPS_GPIO_PERIPH SYSCTL_PERIPH_GPIOP
#define GPS_CONFIG_PINRX GPIO_PP0_U6RX
#define GPS_PORT GPIO_PORTP_BASE
#define GPS_PINRX GPIO_PIN_0
#define GPS_PERIPH SYSCTL_PERIPH_UART6
#define GPS_UART UART6_BASE
#define GPS_INT INT_UART6

//
// UART2 RX/TX - Ground Motor 1
#define GNDMTR1_GPIO_PERIPH SYSCTL_PERIPH_GPIOD
#define GNDMTR1_GPIO_DIRECTION_PERIPH SYSCTL_PERIPH_GPIOP
#define GNDMTR1_CONFIG_PINRX GPIO_PD4_U2RX
#define GNDMTR1_CONFIG_PINTX GPIO_PD5_U2TX

#define GNDMTR1_PORT GPIO_PORTD_BASE
#define GNDMTR1_PINRX GPIO_PIN_4
#define GNDMTR1_PINTX GPIO_PIN_5
#define GNDMTR1_DIRECTION_PORT GPIO_PORTP_BASE
#define GMDMTR1_DIRECTION GPIO_PIN_1

#define GNDMTR1_PERIPH SYSCTL_PERIPH_UART2
#define GNDMTR1_UART UART2_BASE
#define GNDMTR1_INT INT_UART2

//
// UART7 RX/TX - Ground Motor 2
#define GNDMTR2_GPIO_PERIPH SYSCTL_PERIPH_GPIOC
#define GNDMTR2_GPIO_DIRECTION_PERIPH SYSCTL_PERIPH_GPIOE
#define GNDMTR2_CONFIG_PINRX GPIO_PC4_U7RX
#define GNDMTR2_CONFIG_PINTX GPIO_PC5_U7TX

#define GNDMTR2_PORT GPIO_PORTC_BASE
#define GNDMTR2_PINRX GPIO_PIN_4
#define GNDMTR2_PINTX GPIO_PIN_5
#define GNDMTR2_DIRECTION_PORT GPIO_PORTE_BASE
#define GMDMTR2_DIRECTION GPIO_PIN_4

#define GNDMTR2_PERIPH SYSCTL_PERIPH_UART7
#define GNDMTR2_UART UART7_BASE
#define GNDMTR2_INT INT_UART7

/*
 * Radio Timer Defines
 * Timer 5 - Timer A will be used
 * to detect radio communication loss.
 * It will be run at 3 Hz, if it times out,
 * this will indicate that there is no radio
 * signal being received.
 * Timer 0 - Timer A will be used to trigger
 * a radio packet send at the specified
 * frequency.
 */
#define RADIO_TIMER_CHECK TIMER5_BASE
#define RADIO_TIMER_CHECK_INT INT_TIMER5A

#define RADIO_TIMER TIMER0_BASE
#define RADIO_TIMER_INT INT_TIMER0A
#define RADIO_TIMER_RATE 10
#define GS_RADIO_RATE 3

/*
 *  ADC Peripheral Defines for Solar Panels
 *
 *  Using ADC0
 *
 *  Configuration:
 *  Solar Panel 1: Pin PE0
 *  Solar Panel 2: Pin PE1
 *  Solar Panel 3: Pin PE2
 *  Solar Panel 4: Pin PE3
 *  Solar Panel 5: Pin PE5
 */
#define SP_PERIPH SYSCTL_PERIPH_ADC1
#define SP_GPIO_PERIPH SYSCTL_PERIPH_GPIOE
#define SP_PORT GPIO_PORTE_BASE
#define SP1_PIN GPIO_PIN_0
#define SP2_PIN GPIO_PIN_1
#define SP3_PIN GPIO_PIN_2
#define SP4_PIN GPIO_PIN_3
#define SP5_PIN GPIO_PIN_5
#define SP_ADC ADC1_BASE
#define SP1_CHANNEL ADC_CTL_CH3
#define SP2_CHANNEL ADC_CTL_CH2
#define SP3_CHANNEL ADC_CTL_CH4
#define SP4_CHANNEL ADC_CTL_CH0
#define SP5_CHANNEL ADC_CTL_CH8
#define SP_INT INT_ADC1SS0



/*
 * Ultrasonic Sensor Peripheral Defines
 *
 * 6 Total Sensors - each has two communication pins
 * Trigger Pin - used to initiate a pulse on the sensor.
 *      This requires a logical high pulse for a minimum
 *      of 10 micro seconds. This will be done using a timer.
 *
 * Capture Pin - used to count the length of time of the
 *      response. Requires counting the amount of time
 *      between the initial high signal and the falling
 *      edge to a low signal.
 *      Formula: Distance = HighTime * 340 m/s / 2.
 *
 *      Trigger Pins:
 *      PL0 - PL5
 *
 *      Capture Pins:
 *      Use Timers 1, 2, and 3
 *      Capture 1 - Timer2 TimerA
 *      Capture 2 - Timer2 TimerB
 *      Capture 3 - Timer3 TimerA
 *      Capture 4 - Timer3 TimerB
 *      Capture 5 - Timer1 TimerB
 *      Capture 6 - Timer1 TimerA
 */

#define USONIC_TRIG_PERIPH SYSCTL_PERIPH_GPIOL
#define USONIC_TRIG_PORT GPIO_PORTL_BASE

#define TRIG_PIN_1 GPIO_PIN_4
#define TRIG_PIN_2 GPIO_PIN_5
#define TRIG_PIN_3 GPIO_PIN_0
#define TRIG_PIN_4 GPIO_PIN_1
#define TRIG_PIN_5 GPIO_PIN_2
#define TRIG_PIN_6 GPIO_PIN_3

#define USONIC_GPIO_PERIPH SYSCTL_PERIPH_GPIOM
#define USONIC_TIMER2_PORT GPIO_PORTM_BASE
#define USONIC_TIMER2 TIMER2_BASE

#define USONIC_TIMER3 TIMER3_BASE

#define USOINC_TIMER1_PORT GPIO_PORTA_BASE
#define USONIC_TIMER1 TIMER1_BASE

#define CAP_PIN_1 GPIO_PIN_0
#define CAP_PIN_2 GPIO_PIN_1
#define CAP_PIN_3 GPIO_PIN_2
#define CAP_PIN_4 GPIO_PIN_7
#define CAP_PIN_5 GPIO_PIN_3
#define CAP_PIN_6 GPIO_PIN_2

/*
 * Solenoid Enable Pins Peripheral Defines
 *
 * 2 Enable Pins
 * PM7, PP5
 * Must both be driven high at the same time
 * in order to activate the solenoids.
 *
 * To prevent excessive current loss, a timer
 * must be used to turn off the solenoid enable
 * pins after a set amount of time. We will be
 * using timer 4 for this purpose.
 */

#define SOLENOID_PERIPH1 SYSCTL_PERIPH_GPIOM
#define SOLENOID_PERIPH2 SYSCTL_PERIPH_GPIOP

#define SOLENOID_GPIO_PORT1 GPIO_PORTM_BASE
#define SOLENOID_GPIO_PORT2 GPIO_PORTP_BASE

#define SOLENOID_PIN_1 GPIO_PIN_7
#define SOLENOID_PIN_2 GPIO_PIN_5

#define SOLENOID_TIMER_PERIPH SYSCTL_PERIPH_TIMER4
#define SOLENOID_TIMER TIMER4_BASE
#define SOLENOID_INT INT_TIMER4A

/*
 * BOOSTXL-SENSORS Pin Peripheral Defines
 *
 * 2 communication pins, 1 Interrupt pin
 * PB2 - I2C0 SCL
 * PB3 - I2C0 SDA
 * PC7 - INT 1
 */

#define BOOST_GPIO_PERIPH1 SYSCTL_PERIPH_GPIOB
#define BOOST_GPIO_PERIPH2 SYSCTL_PERIPH_GPIOC
#define BOOST_PERIPH SYSCTL_PERIPH_I2C0

#define BOOST_GPIO_PORT_I2C GPIO_PORTB_BASE
#define BOOST_GPIO_PORT_INT GPIO_PORTC_BASE
#define BOOST_GPIO_SDA GPIO_PIN_3
#define BOOST_GPIO_SCL GPIO_PIN_2
#define BOOST_GPIO_INT GPIO_PIN_6
#define MAG_GPIO_INT GPIO_PIN_7

#define BOOST_I2C I2C0_BASE

#define DCM_TIMER_PERIPH SYSCTL_PERIPH_TIMER7
#define DCM_TIMER TIMER7_BASE
#define DCM_TIMER_INT INT_TIMER7A
#define DCM_UPDATE_RATE 100.0f // Hz

/*
 * BOOSTXL_SENSORS BME280 Peripheral Defines
 *
 * Uses the same pins as IMU.
 */
#define BME280_TIMER_PERIPH SYSCTL_PERIPH_TIMER6
#define BME280_TIMER TIMER6_BASE
#define BME280_INT INT_TIMER6A

/*
 * Air Motors PWM Module Peripheral Defines
 *
 * Pins:
 * PF1, PF2, PF3, PG0
 */
#define PWM_PERIPHERAL SYSCTL_PERIPH_PWM0
#define PWM_GPIO_PERIPH SYSCTL_PERIPH_GPIOG
#define PWM_GPIO_PORT1 GPIO_PORTF_BASE
#define PWM_GPIO_PORT2 GPIO_PORTG_BASE

#define PWM_MTR_1 GPIO_PIN_1
#define PWM_MTR_2 GPIO_PIN_2
#define PWM_MTR_3 GPIO_PIN_3
#define PWM_MTR_4 GPIO_PIN_0
#define PWM_MTR_5 GPIO_PIN_1
#define PWM_MTR_6 GPIO_PIN_4

#define MOTOR_OUT_1 PWM_OUT_1
#define MOTOR_OUT_2 PWM_OUT_2
#define MOTOR_OUT_3 PWM_OUT_3
#define MOTOR_OUT_4 PWM_OUT_4
#define MOTOR_OUT_5 PWM_OUT_5
#define MOTOR_OUT_6 PWM_OUT_6

#define PWM_FREQUENCY 200

#define UPDATE_TIMER TIMER7_BASE
#define UPDATE_TIMER_INT INT_TIMER7B
#define UPDATE_TRAJECTORY_RATE 25

/*
 * MMA8452Q Accelerometer Defines
 *
 * Pins:
 * PP4, PN4, PN5
 * I2C SCL: PN5
 * I2C SDA: PN4
 * INT1: PP4
 */
#define MMA8452Q_GPIO_PERIPH SYSCTL_PERIPH_GPION
#define MMA8452Q_I2C_PERIPH SYSCTL_PERIPH_I2C2
#define MMA8452Q_INT_PERIPH SYSCTL_PERIPH_GPIOP

#define MMA8452Q_GPIO_I2C GPIO_PORTN_BASE
#define MMA8452Q_GPIO_INT_PORT GPIO_PORTP_BASE
#define MMA8452Q_GPIO_INT GPIO_PIN_4
#define MMA8452Q_GPIO_SDA GPIO_PIN_4
#define MMA8452Q_GPIO_SCL GPIO_PIN_5

#define MMA8452Q_I2C I2C2_BASE


#endif /* APOPHIS_PIN_MAP_H_ */
