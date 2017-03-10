/*
 * gnd_mtrs.h
 *
 *  Created on: Mar 6, 2017
 *      Author: b
 */

#ifndef GND_MTRS_H_
#define GND_MTRS_H_

//
// Read or Write Direction
// High indicates CPU writing to motor.
// Low indicates CPU reading from motor.

//
// Default Device ID
#define RX24_ID 0xFE

//
// Types of Instruction commands
#define RX24_PING 0x01
#define RX24_READ_DATA 0x02
#define RX24_WRITE_DATA 0x03
#define RX24_REG_WRITE 0x04
#define RX24_ACTION 0x05
#define RX24_RESET 0x06
#define RX24_SYNC_WRITE 0x83

//
// Register Descriptions
// EEPROM - will remain set if power is lost.
#define RX24_REG_MODEL_LSB 0x00
#define RX24_REG_MODEL_MSB 0x01
#define RX24_REG_FIRMWARE 0x02
#define RX24_REG_ID 0x03
#define RX24_REG_BAUD_RATE 0x04
#define RX24_REG_RETURN_DELAY_TIME 0x05
#define RX24_REG_CW_ANGLE_LIMIT_LSB 0x06
#define RX24_REG_CW_ANGLE_LIMIT_MSB 0x07
#define RX24_REG_CCW_ANGLE_LIMIT_LSB 0x08
#define RX24_REG_CCW_ANGLE_LIMIT_MSB 0x09
#define RX24_REG_INT_TEMP_LIMIT 0x0B
#define RX24_REG_LOW_VOLTAGE_LIMIT 0x0C
#define RX24_REG_HIGH_VOLTAGE_LIMIT 0x0D
#define RX24_REG_MAX_TORQUE_LSB 0x0E
#define RX24_REG_MAX_TORQUE_MSB 0x0F
#define RX24_REG_STATUS_RETURN_LEVEL 0x10
#define RX24_REG_ALARM_LED 0x11
#define RX24_REG_ALARM_SHUTDOWN 0x12

//
// RAM - will default to original value on power loss.
#define RX24_REG_TORQ_EN 0x18
#define RX24_REG_LED_EN 0x19
#define RX24_REG_CW_COMP_MARGIN 0x1A
#define RX24_REG_CCW_COMP_MARGIN 0x1B
#define RX24_REG_CW_COMP_SLOPE 0x1C
#define RX24_REG_CCW_COMP_SLOPE 0x1D
#define RX24_REG_GOAL_POS_LSB 0x1E
#define RX24_REG_GOAL_POS_MSB 0x1F
#define RX24_REG_MOVING_VEL_LSB 0x20
#define RX24_REG_MOVING_VEL_MSB 0x21
#define RX24_REG_TORQ_LIM_LSB 0x22
#define RX24_REG_TORQ_LIM_MSB 0x23

//
// Read only values
#define RX24_REG_PRES_POS_LSB 0x24
#define RX24_REG_PRES_POS_MSB 0x25
#define RX24_REG_PRES_VEL_LSB 0x26
#define RX24_REG_PRES_VEL_MSB 0x27
#define RX24_REG_PRES_LOAD_LSB 0x28
#define RX24_REG_PRES_LOAD_MSB 0x29
#define RX24_REG_PRES_VOLTAGE 0x2A
#define RX24_REG_PRES_TEMP 0x2B
#define RX24_REG_REGISTERED 0x2C
#define RX24_REG_MOVING 0x2E

//
// RW values
#define RX24_REG_LOCK 0x2F
#define RX24_REG_PUNCH_LSB 0x30
#define RX24_REG_PUNCH_MSB 0x31

//
// REGISTER VALUES
//

//
// Baud Rate
#define RX24_BAUD_9600 0xCF
#define RX24_BAUD_57600 0x22
#define RX24_BAUD_115200 0x10

//
// Return Delay Time
// Defualt is 0.5 ms
#define RX24_RTN_DELAY_20U 0x0A

//
// CW/CCW Angle Limit
// Should both be 0 for wheel mode.
#define RX24_ANGLE_LIMIT_NONE 0x00

//
// Voltage Limits
#define RX24_LOW_VOLT_9V 0x5A
#define RX24_HIGH_VOLT_13V 0x82

//
// Max Torque
#define RX24_MAX_TORQ_100 0x03FF
#define RX24_MAX_TORQ_50 0x0200

//
// Status Return
#define RX24_STS_RTN_NONE 0x00
#define RX24_STS_RTN_READ 0x01
#define RX24_STS_RTN_ALL 0x02

//
// Alarm LED and Shutdown
#define RX24_ERROR_INSTRUCTION 0x40
#define RX24_ERROR_OVERLOAD 0x20
#define RX24_ERROR_CHECKSUM 0x10
#define RX24_ERROR_RANGE 0x08
#define RX24_ERROR_OVERHEAT 0x04
#define RX24_ERROR_ANGLE_LIMIT 0x02
#define RX24_ERROR_INPUT_VOLTAGE 0x01

//
// Torque Enable
#define RX24_TORQ_EN 0x01
#define RX24_TORQ_DIS 0x00

//
// LED
#define RX24_LED_ON 0x01
#define RX24_LED_OFF 0x00

//
// Compliance
// Only used for joint control.

//
// Goal Position
// Only used for joint control.

//
// Moving Speed
// Value 0 - 2047 (0x7FF)
// Last bit controls direction. 0 - 1023 is CCW
// 1024 - 2047 is CW
// It is % of max output not direct speed.
// Throttle increment is 2% of total output.
#define RX24_STOP_CCW 0x0000
#define RX24_STOP_CW 0x0400
#define RX24_THROTTLE_INCREMENT 0x0A

//
// Torque limit
// See Max Torque register values.

//
// Present Position
// Value between 0 - 1023, each unit is ~0.29 degrees.

//
// Present speed
// Value between 0 - 2047, same as Moving speed.

//
// Present load
// Value between 0 - 2047, same as Moving speed,
// but for torque instead of output.

//
// Present Voltage
// Same as High/Low voltage limit.

//
// Present temperature
// In degrees Celsius. Value is exact temp in decimal.

//
// Registered instruction

//
// Moving
#define RX24_MOVING 0x01
#define RX24_NOT_MOVING 0x00

//
// Lock
// To unlock eeprom, power must be removed.
#define RX24_EEPROM_LOCK 0x01

//
// Punch
// Current to drive motor is at minimum...
// ???

//
// Function Prototypes.

void InitRx24FMotor(uint32_t Uart_base, uint32_t Gpio_base, uint32_t Gpio_pin);
void Rx24FWrite(uint32_t UART_BASE, uint32_t GPIO_BASE, uint32_t GPIO_PIN,
                uint32_t numBytes, uint8_t *buffer);
void Rx24FRead(uint32_t UART_BASE, uint32_t GPIO_BASE, uint32_t GPIO_PIN,
              uint8_t *rxBuffer, uint8_t *txBuffer);


#endif /* GND_MTRS_H_ */
