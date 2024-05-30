/**
 ******************************************************************************
 * File Name          : I2CPeripheral_Interface.h
 * Description        : Peripheral Interface for Drivers
 *   This file contains declarations for I2C peripheral functions that must be defined.
 * Authors            : Chris (cjchanx)
 ******************************************************************************
*/
#ifndef INCLUDE_PERIPHERAL_INTERFACE_H_
#define INCLUDE_PERIPHERAL_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* User Implemented Functions -----------------------------------------------------------------
 * These functions must be implemented by the user to interface with the communication peripherals.
*/

/**
 * @brief I2C Write Function
 * @param device_addr Address of the I2C Device (8-bit address)
 * @param data Pointer to the data to be written
 * @param len Length of write data in bytes
 * @return 0 on success, 1 on failure
 */
uint8_t I2C_Write(uint8_t device_addr, uint8_t* data, uint8_t len);

/**
 * @brief I2C Read Function
 * @param device_addr Address of the I2C Device (8-bit address)
 * @param dest Pointer to the buffer to store the data (must be >= len in size)
 * @param len Length of the read data in bytes
 * @return 0 on success, 1 on failure
 */
uint8_t I2C_Read(uint8_t device_addr, uint8_t* dest, uint8_t len);

/* Internal Function Abstractions -----------------------------------------------------------
* These functions are used by the drivers to interface with the user-defined functions.
*/
#define I2C_7B_ADDR_TO_8B_W(addr) ((addr << 1) & 0xFE)
#define I2C_WRITE_TO_READ_ADDR(addr) (addr | 0x01)

/**
 * @brief Write to one register address
 * 
 * @param dev Device address (8-bit, write address)
 * @param reg Register address to write to
 * @param data Byte to write to the register
 * @return uint8_t 
 */
static inline uint8_t I2C_WriteRegister(uint8_t dev, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return I2C_Write(dev, buf, 2);
}

/**
 * @brief Read registers starting from a register address
 * 
 * @param dev Device address (8-bit, write address)
 * @param reg Register address to start reading from
 * @param dest Pointer to the buffer to store the data
 * @param len Length of the data to be read, must be <= size of buffer
 * @return uint8_t 
 */
static inline uint8_t I2C_ReadRegisters(uint8_t dev, uint8_t reg, uint8_t* dest, uint8_t len) {
    if (!I2C_Write(dev, &reg, 1)) return 1;
    return I2C_Read(I2C_WRITE_TO_READ_ADDR(dev), dest, len);
}

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_PERIPHERAL_INTERFACE_H_ */
