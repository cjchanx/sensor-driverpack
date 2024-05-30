/**
 ******************************************************************************
 * File Name          : I2CPeripheral_STM32HAL.c
 * Description        : Example polling I2C Peripheral Driver using STM32 HAL
 * Authors            : Chris (cjchanx)
 ******************************************************************************
*/

/* Includes ------------------------------------------------------- */
#include "I2CPeripheral_Interface.h"
#include "stm32l1xx_hal.h" // Change this to the target processor family

/* I2C Line ------------------------------------------------------- */
#define I2C_LINE hi2c1 // Change this to the target I2C Handle
extern I2C_HandleTypeDef I2C_LINE; 

/* Functions ------------------------------------------------------ */
/**
 * @brief I2C Write Function
 * @param device_addr Address of the I2C Device (8-bit address)
 * @param data Pointer to the data to be written
 * @param len Length of write data in bytes
 * @return 0 on success, 1 on failure
 */
uint8_t I2C_Write(uint8_t device_addr, uint8_t* data, uint8_t len) {
    if (HAL_I2C_Master_Transmit(&I2C_LINE, device_addr, data, len, 1000) == HAL_OK)
        return 0;
    return 1;
}

/**
 * @brief I2C Read Function
 * @param device_addr Address of the I2C Device (8-bit address)
 * @param dest Pointer to the buffer to store the data (must be >= len in size)
 * @param len Length of the read data in bytes
 * @return 0 on success, 1 on failure
 */
uint8_t I2C_Read(uint8_t device_addr, uint8_t* dest, uint8_t len) {
    if (HAL_I2C_Master_Receive(&I2C_LINE, device_addr, dest, len, 1000) == HAL_OK)
        return 0;
    return 1;

}
