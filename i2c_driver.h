/*
 * i2c.h
 *
 *  Created on: Jan 31, 2017
 *      Author: b
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

void I2CBurstWrite(uint32_t I2C_base, uint8_t deviceAddress, int numBytes, uint8_t *txBuffer);
void I2CSingleWrite(uint32_t I2C_base, uint8_t deviceAddress, uint8_t *txBuffer);
void I2CRead(uint32_t I2C_base, uint8_t deviceAddress, uint8_t readRegister,
             int numBytes, uint8_t *rxBuffer);
uint32_t I2CWait(uint32_t I2C_base);

#endif /* I2C_DRIVER_H_ */
