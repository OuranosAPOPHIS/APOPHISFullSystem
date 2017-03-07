/*
 * gnd_mtrs.c
 *
 *  Created on: Mar 6, 2017
 *      Author: Brandon Klefman
 *      Purpose: Initialization function and interaction
 *      functions for the RX24F motors.
 */

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#include "gnd_mtrs.h"

#define DEBUG true

/*
 * Initialization for Rx24 motor 2.
 * Uses UART2 for RX and TX.
 *
 */
void InitRx24FMotor(uint32_t UART_BASE, uint32_t GPIO_BASE, uint32_t GPIO_PIN)
{
    uint8_t txBuffer[4];
    uint8_t rxBuffer[100] = { 0 };
    uint32_t index = 0;

    //
    // Reset the device.
    txBuffer[0] = RX24_RESET;
    Rx24FWrite(UART_BASE, GPIO_BASE, GPIO_PIN, 1, txBuffer);

    //
    // Set the status return register.
    txBuffer[0] = RX24_WRITE_DATA;
    txBuffer[1] = RX24_REG_STATUS_RETURN_LEVEL;
    txBuffer[2] = RX24_STS_RTN_READ;
    Rx24FWrite(UART_BASE, GPIO_BASE, GPIO_PIN, 3, txBuffer);

#if DEBUG
    //
    // Read the baud rate and output the value to the console.
    txBuffer[0] = RX24_READ_DATA;
    txBuffer[1] = RX24_REG_STATUS_RETURN_LEVEL;
    txBuffer[2] = 0x01;
    //Rx24FRead(UART_BASE, GPIO_BASE, GPIO_PIN, rxBuffer, txBuffer);

    UARTprintf("Status Return Level: 0x%x\r\n", rxBuffer[0]);
#endif

    //
    // Set the baud rate of the device. Default is 57600.
    txBuffer[0] = RX24_WRITE_DATA;
    txBuffer[1] = RX24_REG_BAUD_RATE;
    txBuffer[2] = RX24_BAUD_57600;
    Rx24FWrite(UART_BASE, GPIO_BASE, GPIO_PIN, 3, txBuffer);

#if DEBUG
    //
    // Read the baud rate and output the value to the console.
    txBuffer[0] = RX24_READ_DATA;
    txBuffer[1] = RX24_REG_BAUD_RATE;
    txBuffer[2] = 0x01;
    Rx24FRead(UART_BASE, GPIO_BASE, GPIO_PIN, rxBuffer, txBuffer);

    UARTprintf("BAUD RATE: 0x%x\r\n", rxBuffer[0]);
#endif

    //
    // Set the CW angle limit value
    txBuffer[0] = RX24_WRITE_DATA;
    txBuffer[1] = RX24_REG_CW_ANGLE_LIMIT_LSB;
    txBuffer[2] = RX24_ANGLE_LIMIT_NONE;
    txBuffer[3] = RX24_ANGLE_LIMIT_NONE;
    Rx24FWrite(UART_BASE, GPIO_BASE, GPIO_PIN, 4, txBuffer);

#if DEBUG
    //
    // Read the CW angle limit value.
    txBuffer[0] = RX24_READ_DATA;
    txBuffer[1] = RX24_REG_CW_ANGLE_LIMIT_LSB;
    txBuffer[2] = 0x02;
    Rx24FRead(UART_BASE, GPIO_BASE, GPIO_PIN, rxBuffer, txBuffer);

    UARTprintf("CW Angle Limit: 0x%x%x\r\n", rxBuffer[0], rxBuffer[1]);
#endif

    //
    // Set the CCW angle limit value
    txBuffer[0] = RX24_WRITE_DATA;
    txBuffer[1] = RX24_REG_CCW_ANGLE_LIMIT_LSB;
    txBuffer[2] = RX24_ANGLE_LIMIT_NONE;
    txBuffer[3] = RX24_ANGLE_LIMIT_NONE;
    Rx24FWrite(UART_BASE, GPIO_BASE, GPIO_PIN, 4, txBuffer);

#if DEBUG
    //
    // Read the CW angle limit value.
    txBuffer[0] = RX24_READ_DATA;
    txBuffer[1] = RX24_REG_CCW_ANGLE_LIMIT_LSB;
    txBuffer[2] = 0x02;
    Rx24FRead(UART_BASE, GPIO_BASE, GPIO_PIN, rxBuffer, txBuffer);

    UARTprintf("CCW Angle Limit: 0x%x%x\r\n", rxBuffer[0], rxBuffer[1]);
#endif

    //
    // Set the low voltage limit.
    txBuffer[0] = RX24_WRITE_DATA;
    txBuffer[1] = RX24_REG_LOW_VOLTAGE_LIMIT;
    txBuffer[2] = RX24_LOW_VOLT_9V;
    Rx24FWrite(UART_BASE, GPIO_BASE, GPIO_PIN, 3, txBuffer);

#if DEBUG
    //
    // Read the low voltage limit and output the value to the console.
    txBuffer[0] = RX24_READ_DATA;
    txBuffer[1] = RX24_REG_LOW_VOLTAGE_LIMIT;
    txBuffer[2] = 0x01;
    Rx24FRead(UART_BASE, GPIO_BASE, GPIO_PIN, rxBuffer, txBuffer);

    UARTprintf("Low Voltage Limit: 0x%x\r\n", rxBuffer[0]);
#endif

    //
    // Set the high voltage limit.
    txBuffer[0] = RX24_WRITE_DATA;
    txBuffer[1] = RX24_REG_HIGH_VOLTAGE_LIMIT;
    txBuffer[2] = RX24_HIGH_VOLT_13V;
    Rx24FWrite(UART_BASE, GPIO_BASE, GPIO_PIN, 3, txBuffer);

#if DEBUG
    //
    // Read the high voltage limit and output the value to the console.
    txBuffer[0] = RX24_READ_DATA;
    txBuffer[1] = RX24_REG_HIGH_VOLTAGE_LIMIT;
    txBuffer[2] = 0x01;
    Rx24FRead(UART_BASE, GPIO_BASE, GPIO_PIN, rxBuffer, txBuffer);

    UARTprintf("High Voltage Limit: 0x%x\r\n", rxBuffer[0]);
#endif

    //
    // Set the max torque limit.
    txBuffer[0] = RX24_WRITE_DATA;
    txBuffer[1] = RX24_REG_MAX_TORQUE_LSB;
    txBuffer[2] = (uint8_t)(RX24_MAX_TORQ_100 >> 8);
    txBuffer[3] = (uint8_t)RX24_MAX_TORQ_100;
    Rx24FWrite(UART_BASE, GPIO_BASE, GPIO_PIN, 4, txBuffer);

#if DEBUG
    //
    // Read the max torque limit
    txBuffer[0] = RX24_READ_DATA;
    txBuffer[1] = RX24_REG_MAX_TORQUE_LSB;
    txBuffer[2] = 0x02;
    Rx24FRead(UART_BASE, GPIO_BASE, GPIO_PIN, rxBuffer, txBuffer);

    UARTprintf("Max Torque: 0x%x%x\r\n", rxBuffer[0], rxBuffer[1]);
#endif

    GPIOPinWrite(GPIO_BASE, GPIO_PIN, 0x00);

}

/*
 * This function will write a specified number of bytes to the RX24 motor.
 * UART_BASE - base address of the UART module used with the motor.
 * GPIO_BASE - gpio address for the direction pin.
 * GPIO_PIN - gpio pin for the direction indication.
 * numBytes - number of bytes to send to the device.
 * buffer - bytes to send to device. Must be the instruction and all parameters only.
 *
 * Note: This function does not check for FIFO overflow, so the numBytes
 * must be less than 16.
 */
void Rx24FWrite(uint32_t UART_BASE, uint32_t GPIO_BASE, uint32_t GPIO_PIN,
                uint32_t numBytes, uint8_t *buffer)
{
    uint32_t index;
    uint8_t length;
    uint8_t checkSum;
    uint8_t sum = 0;
    uint16_t temp = 0;

    //
    // Drive the direction pin High to indicate a write.
    GPIOPinWrite(GPIO_BASE, GPIO_PIN, GPIO_PIN);

    //
    // Write two 0xFF to indicate start of packet transfer.
    for (index = 0; index < 2; index++)
    {
        //
        // Make sure there is space available in the FIFO.
        while (!UARTSpaceAvail(UART_BASE));

        UARTCharPutNonBlocking(UART_BASE, 0xFF);
    }

    while (!UARTSpaceAvail(UART_BASE));

    //
    // Send the device ID.
    UARTCharPutNonBlocking(UART_BASE, RX24_ID);
    while (!UARTSpaceAvail(UART_BASE));

    //
    // Send the packet size.
    length = numBytes - 1 + 2;
    UARTCharPutNonBlocking(UART_BASE, length);

    //
    // Write to the device.
    for (index = 0; index < numBytes; index++)
    {
        while (!UARTSpaceAvail(UART_BASE));

        //
        // Write to the UART.
        UARTCharPutNonBlocking(UART_BASE, buffer[index]);

        //
        // Calculate part of the check sum.
        sum += buffer[index];
    }

    while (!UARTSpaceAvail(UART_BASE));

    //
    // Calculate and write the checksum.
    temp = (RX24_ID + length + sum);

    //
    // Check to make sure temp does not exceed 255.
    if (temp > 255)
        checkSum = ~(temp & 0x00FF);
    else
        checkSum = ~(temp);

    UARTCharPutNonBlocking(UART_BASE, checkSum);

    //
    // Return the direction pin to low.
    GPIOPinWrite(GPIO_BASE, GPIO_PIN, 0x00);
}

/*
 * Read a value from the RX24 Motor.
 * UART_BASE - base address of the UART module used with the motor.
 * GPIO_BASE - gpio address for the direction pin.
 * GPIO_PIN - gpio pin for the direction indication.
 * numBytes - number of bytes to read from the device.
 * rxBuffer - bytes to read from device.
 * txBuffer - bytes to send to device. This must be the read address and
 * number of bytes to read.
 */
void Rx24FRead(uint32_t UART_BASE, uint32_t GPIO_BASE, uint32_t GPIO_PIN,
              uint8_t *rxBuffer, uint8_t *txBuffer)
{
    uint32_t index;
    uint8_t length;
    uint8_t checkSum;
    uint8_t sum = 0;
    uint16_t temp = 0;
    uint8_t tempBuffer[16] = { 0 };
    bool firstPass = true;
    bool validData = false;

    //
    // Drive the direction pin High to indicate a write.
    GPIOPinWrite(GPIO_BASE, GPIO_PIN, GPIO_PIN);

    //
    // Write two 0xFF to indicate start of packet transfer.
    for (index = 0; index < 2; index++)
    {
        //
        // Make sure there is space available in the FIFO.
        while (!UARTSpaceAvail(UART_BASE))
            ;

        UARTCharPutNonBlocking(UART_BASE, 0xFF);
    }

    while (!UARTSpaceAvail(UART_BASE))
        ;

    //
    // Send the device ID.
    UARTCharPutNonBlocking(UART_BASE, RX24_ID);
    while (!UARTSpaceAvail(UART_BASE))
        ;

    //
    // Send the packet size. Read commands will always be 4 bytes.
    length = 4;
    UARTCharPutNonBlocking(UART_BASE, length);

    //
    // Send the Read instruction.
    while (!UARTSpaceAvail(UART_BASE))
        ;
    UARTCharPutNonBlocking(UART_BASE, RX24_READ_DATA);

    //
    // Send the rest of the data.
    for (index = 0; index < 2; index++)
    {
        while (!UARTSpaceAvail(UART_BASE))
            ;

        UARTCharPutNonBlocking(UART_BASE, txBuffer[index]);
    }

    while (!UARTSpaceAvail(UART_BASE))
                ;

    //
    // Write the checksum.
    //
    // Calculate and write the checksum.
    temp = (RX24_ID + length + sum);

    //
    // Check to make sure temp does not exceed 255.
    if (temp > 255)
        checkSum = ~(temp & 0x00FF);
    else
        checkSum = ~(temp);

    UARTCharPutNonBlocking(UART_BASE, checkSum);

    //
    // Wait for everything to send.
    while (UARTBusy(UART_BASE))
        ;

    //
    // Drive the direction pin to low to indicate a read.
    GPIOPinWrite(GPIO_BASE, GPIO_PIN, 0x00);
    SysCtlDelay(10000);
    index = 0;

	while (UARTCharsAvail(UART_BASE))
	{
		tempBuffer[index++] = UARTCharGetNonBlocking(UART_BASE);
	}

    //
    // Get the data sent back by the motor.
    while (UARTCharsAvail(UART_BASE) && (index < sizeof(tempBuffer)))
    {
        if (!validData) {
            tempBuffer[index++] = UARTCharGetNonBlocking(UART_BASE);
            if (!firstPass && (tempBuffer[index] == 0xFF) && (tempBuffer[index - 1] == 0xFF))
            {
                //
                // Found the start of the packet.
                validData = true;
                index = 0;
            }
            else
                firstPass = false;
        }
        else
        {
            //
            // Get the next bytes.
            tempBuffer[index++] = UARTCharGetNonBlocking(UART_BASE);
        }
    }

    //
    // Return the register data.
    for (index = 0; index < txBuffer[2]; index++)
        rxBuffer[index] = tempBuffer[index + 5];
}
