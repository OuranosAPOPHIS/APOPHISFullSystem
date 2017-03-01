/*
 * i2c.c
 *
 *  Created on: Jan 31, 2017
 *      Author: Brandon Klefman
 *      Purpose: Driver for the I2C module.
 */

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#include "i2c_driver.h"

#include "utils/uartstdio.h"

//*****************************************************************************
//
// This function does a burst write to the I2C device specified.
//
// I2C_base - the base address of the I2C module.
// deviceAddress - address of the slave device on the I2C bus.
// numBytes - number of bytes to send over the bus.
// txBuffer - pointer to the data to be sent.
//
//*****************************************************************************
void I2CBurstWrite(uint32_t I2C_base, uint8_t deviceAddress, int numBytes, uint8_t *txBuffer)
{
    int index = 0;

    //
    // Disable interrupts.
    IntMasterDisable();

    //
    // Set the slave address. False indicates that it is a
    // read command. (Master is writing to the slave).
    I2CMasterSlaveAddrSet(I2C_base, deviceAddress, false);

    //
    // Send data to the device.
    I2CMasterDataPut(I2C_base, txBuffer[index++]);

    //
    // Initiate a slave read in burst form.
    I2CMasterControl(I2C_base, I2C_MASTER_CMD_BURST_SEND_START);

    //
    // Wait for the transaction to finish.
    I2CWait(I2C_base);

    while(index < numBytes - 1)
    {
        //
        // Send the data.
        I2CMasterDataPut(I2C_base, txBuffer[index++]);

        //
        // Initiate a slave read in burst form.
        I2CMasterControl(I2C_base, I2C_MASTER_CMD_BURST_SEND_CONT);

        //
        // Wait for the transaction to finish.
        I2CWait(I2C_base);
    }

    //
    // Send the last piece of data.
    I2CMasterDataPut(I2C_base, txBuffer[index++]);

    //
    // Finish the read.
    I2CMasterControl(I2C_base, I2C_MASTER_CMD_BURST_SEND_FINISH);

    //
    // Wait for the transaction to finish.
    I2CWait(I2C_base);

    //
    // All done, re-enable interrupts.
    IntMasterEnable();
}

//*****************************************************************************
//
// This function does a single write to the I2C device specified.
//
// I2C_base - the base address of the I2C module.
// deviceAddress - address of the slave device on the I2C bus.
// txBuffer - pointer to the data to be sent.
//
//*****************************************************************************
void I2CSingleWrite(uint32_t I2C_base, uint8_t deviceAddress, uint8_t *txBuffer)
{
    //
    // Set the address to send from.
    I2CMasterSlaveAddrSet(I2C_base, deviceAddress, false);

    //
    // Send the data.
    I2CMasterDataPut(I2C_base, txBuffer[0]);

    //
    // Initiate a slave read in burst form.
    I2CMasterControl(I2C_base, I2C_MASTER_CMD_SINGLE_SEND);

    //
    // Wait for the transaction to finish.
    I2CWait(I2C_base);
}
//*****************************************************************************
//
// This function reads from the specified I2C slave device.
//
// I2C_base - the base address of the I2C module.
// deviceAddress - address of the slave device on the I2C bus.
// readRegister - register to read in the device.
// numBytes - number of bytes to send over the bus.
// txBuffer - pointer to the data to be sent.
//
//*****************************************************************************
void I2CRead(uint32_t I2C_base, uint8_t deviceAddress, uint8_t readRegister,
             int numBytes, uint8_t *rxBuffer)
{
    int index = 0;

    IntMasterDisable();

    //
    // Make sure the bus is clear.
    I2CWait(I2C_base);

    //
    // Set the slave address. False indicates that this is a
    // read command. (Master is writing to the slave).
    I2CMasterSlaveAddrSet(I2C_base, deviceAddress, false);

    //
    // First send the register which we want to read from.
    I2CMasterDataPut(I2C_base, readRegister);

    //
    // Tell the device to read the data sent.
    I2CMasterControl(I2C_base, I2C_MASTER_CMD_SINGLE_SEND);

    //
    // Wait for that to finish.
    I2CWait(I2C_base);

    if (numBytes == 1)
    {
        //
        // Set the slave address. True indicates that this is a
        // write command. (Master is reading from the slave).
        I2CMasterSlaveAddrSet(I2C_base, deviceAddress, true);

        //
        // Send the command to the device.
        I2CMasterControl(I2C_base, I2C_MASTER_CMD_SINGLE_RECEIVE);

        //
        // Wait for transaction to finish.
        I2CWait(I2C_base);

        //
        // Read one byte.
        rxBuffer[index++] = I2CMasterDataGet(I2C_base);
    }
    else
    {
        //
        // Set the slave address. True indicates that this is a
        // write command. (Master is reading from the slave).
        I2CMasterSlaveAddrSet(I2C_base, deviceAddress, true);

        //
        // Send the command to the device.
        I2CMasterControl(I2C_base, I2C_MASTER_CMD_BURST_RECEIVE_START);

        //
        // Wait for transaction to finish.
        I2CWait(I2C_base);

        //
        // Read one byte.
        rxBuffer[index++] = I2CMasterDataGet(I2C_base);

        while(index < numBytes - 1)
        {
            //
            // Send the command to the device.
            I2CMasterControl(I2C_base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

            //
            // Wait for transaction to finish.
            I2CWait(I2C_base);

            //
            // Get another byte.
            rxBuffer[index++] = I2CMasterDataGet(I2C_base);
        }

        //
        // Send the command to the device.
        I2CMasterControl(I2C_base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

        //
        // Wait for transaction to finish.
        I2CWait(I2C_base);

        //
        // Read one last byte.
        rxBuffer[index++] = I2CMasterDataGet(I2C_base);
    }

    IntMasterEnable();
}

//*****************************************************************************
//
// This function waits for the transaction to complete on the desired I2C bus.
//
// I2C_base - the base address of the I2C module.
//
//*****************************************************************************
uint32_t I2CWait(uint32_t I2C_base)
{
    uint32_t status;

    //
    // Wait for the transaction to complete.
    SysCtlDelay(1000);
    while((I2CMasterBusy(I2C_base)));

    //
    // Get the error status.
    status = I2CMasterErr(I2C_base);

    //
    // Check the error.
    if (status == I2C_MASTER_ERR_NONE)
    {
        //
        // No error.
        return status;
    }
    else if (status == I2C_MASTER_ERR_ADDR_ACK)
    {
        //
        // Address acknowledge error.
        UARTprintf("ADDRESS ACK ERROR\n\r");
    }
    else if (status == I2C_MASTER_ERR_DATA_ACK)
    {
        //
        // Data acknowledge error.
        UARTprintf("DATA ACK ERROR\n\r");
    }
    else if (status == I2C_MASTER_ERR_ARB_LOST)
    {
        //
        // ARB lost error.
        UARTprintf("ARB LOST ERROR\n\r");
    }

    return status;
}
