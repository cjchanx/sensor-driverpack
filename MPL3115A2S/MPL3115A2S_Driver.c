/**
 ******************************************************************************
 * File Name          : MPL3115A2S_Driver.c
 * Description        : MPL3115A2S Driver
 * Authors            : Chris (cjchanx)
 ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "./MPL3115A2S_Driver.h"

/* Macros and Constants ------------------------------------------------------*/
#define MPL3115A2S_ADDR (0xC0) // 8-bit write address for MPL3115A2S

// Registers
#define MPL3115A2S_STATUS (0x00) // Status Register

#define MPL3115A2S_OUT_P_MSB (0x01) // Pressure Data Out MSB
#define MPL3115A2S_OUT_P_CSB (0x02) // Pressure Data Out CSB
#define MPL3115A2S_OUT_P_LSB (0x03) // Pressure Data Out LSB

#define MPL3115A2S_OUT_T_MSB (0x04) // Temperature Data Out MSB
#define MPL3115A2S_OUT_T_LSB (0x05) // Temperature Data Out LSB
#define MPL3115A2S_DR_STATUS (0x06) // Data Ready Status

// FIFO Registers
#define MPL3115A2S_F_SETUP (0x0F) // FIFO Setup
    #define MPL3115A2S_F_MODE_CIRCULAR (0b01 << 6) // Circular FIFO Mode
    #define MPL3115A2S_F_MODE_FIFO (0b10 << 6) // FIFO Mode Stop on Full

// Control Registers
#define MPL3115A2S_CTRL_REG1 (0x26) // Control Register 1
    #define MPL3115A2S_CTRL_ALT_SHIFT (7) // Altimeter Mode Shift
    #define MPL3115A2S_CTRL_OSR_SHIFT (3) // Oversampling Ratio Shift
    #define MPL3115A2S_CTRL_RST (0b1 << 2) // Software Reset
    #define MPL3115A2S_CTRL_OST (0b1 << 1) // One-Shot
    #define MPL3115A2S_CTRL_ACTIVE (0b1) // Active Mode Enable (from Standby)

#define MPL3115A2S_PT_DATA_CFG (0x13) // PT Data Configuration
    #define MPL3115A2S_PT_DATA_CFG_DREM_SHIFT (2) // Data Ready Event Mode Shift
    #define MPL3115A2S_PT_DATA_CFG_PDEFE_SHIFT (1) // Pressure Data Event Flag Enable Shift
    #define MPL3115A2S_PT_DATA_CFG_TDEFE_SHIFT (0) // Temperature Data Event Flag Enable Shift
    #define MPL3115A2S_PT_DATA_CFG_EN_ALL (0b111) // Enable All Flags

#define MPL3115A2S_CTRL_REG3 (0x28) // Control Register 3
    #define MPL3115A2S_CTRL_REG3_IPOL1_SHIFT (5) // Interrupt 1 Polarity Shift
    #define MPL3115A2S_CTRL_REG3_PP_OD1_SHIFT (4) // Interrupt 1 Open Drain Shift
    #define MPL3115A2S_CTRL_REG3_IPOL2_SHIFT (1) // Interrupt 2 Polarity Shift
    #define MPL3115A2S_CTRL_REG3_PP_OD2_SHIFT (0) // Interrupt 2 Open Drain Shift

#define MPL3115A2S_CTRL_REG4 (0x29) // Control Register 4
    #define MPL3115A2S_CTRL_REG4_INT_EN_DRDY_SHIFT (7) // Data Ready Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_FIFO_SHIFT (6) // FIFO Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_PW_SHIFT (5) // Pressure Window Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_TW_SHIFT (4) // Temperature Window Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_PTH_SHIFT (3) // Pressure Threshold Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_TTH_SHIFT (2) // Temperature Threshold Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_PCHG_SHIFT (1) // Pressure Change Interrupt Enable Shift
    #define MPL3115A2S_CTRL_REG4_INT_EN_TCHG_SHIFT (0) // Temperature Change Interrupt Enable Shift

#define MPL3115A2S_WHO_AM_I (0x0C) // Device ID Register
    #define MPL3115A2S_WHO_AM_I_ID (0xC4) // Device ID

/* Helpers -------------------------------------------------------------------*/
static uint8_t DecodeConfiguration(MPL3115A2S_Config* config) {
    return (config->altimeter_mode << MPL3115A2S_CTRL_ALT_SHIFT) | (config->os_ratio << MPL3115A2S_CTRL_OSR_SHIFT);
}
static uint8_t DecodeDataConfiguration(MPL3115A2S_Data_Config* config) {
    if(config->enable_all_flags) 
        return MPL3115A2S_PT_DATA_CFG_EN_ALL;
    return (config->data_ready_event_mode << MPL3115A2S_PT_DATA_CFG_DREM_SHIFT) | 
           (config->pressure_data_event_flag << MPL3115A2S_PT_DATA_CFG_PDEFE_SHIFT) | 
           (config->temperature_data_event_flag << MPL3115A2S_PT_DATA_CFG_TDEFE_SHIFT);
}
static uint8_t DecodeIntConfiguration_CTRL3(MPL3115A2S_Int_Config* config) {
    return (config->int1_active_high << MPL3115A2S_CTRL_REG3_IPOL1_SHIFT) | 
           (config->int1_open_drain << MPL3115A2S_CTRL_REG3_PP_OD1_SHIFT) | 
           (config->int2_active_high << MPL3115A2S_CTRL_REG3_IPOL2_SHIFT) | 
           (config->int2_open_drain << MPL3115A2S_CTRL_REG3_PP_OD2_SHIFT);
}
static uint8_t DecodeIntConfiguration_CTRL4(MPL3115A2S_Int_Config* config) {
    return (config->data_ready_it_enable << MPL3115A2S_CTRL_REG4_INT_EN_DRDY_SHIFT) | 
           (config->fifo_it_enable << MPL3115A2S_CTRL_REG4_INT_EN_FIFO_SHIFT) | 
           (config->pressure_window_it_enable << MPL3115A2S_CTRL_REG4_INT_EN_PW_SHIFT) | 
           (config->temperature_window_it_enable << MPL3115A2S_CTRL_REG4_INT_EN_TW_SHIFT) | 
           (config->pressure_threshold_it_enable << MPL3115A2S_CTRL_REG4_INT_EN_PTH_SHIFT) | 
           (config->temperature_threshold_it_enable << MPL3115A2S_CTRL_REG4_INT_EN_TTH_SHIFT) | 
           (config->pressure_change_it_enable << MPL3115A2S_CTRL_REG4_INT_EN_PCHG_SHIFT) | 
           (config->temperature_change_it_enable << MPL3115A2S_CTRL_REG4_INT_EN_TCHG_SHIFT);
}

static uint32_t DecodePressureAsInt(uint8_t* buf, uint16_t multiplier) {
    // MSB, CSB, LSB in Q18.2 format
    uint32_t pressure = (buf[0] << 16) | (buf[1] << 8) | buf[2];
    pressure >>= 4; // Discard unused bits
    pressure *= multiplier;
    pressure /= 4;
    return pressure;
}
static float DecodePressureAsFloat(uint8_t* buf) {
    // MSB, CSB, LSB in Q18.2 format
    uint32_t pressure = (buf[0] << 16) | (buf[1] << 8) | buf[2];
    pressure >>= 4; // Discard unused bits
    return (float)pressure / 4.0f;
}

static int32_t DecodeAltitudeAsInt(uint8_t* buf, uint16_t multiplier) {
    // MSB, CSB, LSB in Q16.4 format
    int32_t altitude = ((short)((buf[0] << 8) | buf[1]));
    altitude *= multiplier;
    altitude += ((buf[2] >> 4) * multiplier) / 16;
    return altitude;
}
static float DecodeAltitudeAsFloat(uint8_t* buf) {
    // MSB, CSB, LSB in Q16.4 format
    float altitude = ((short)((buf[0] << 8) | buf[1])) + ((float)(buf[2] >> 4)/16.0f);
    return altitude;
}

static int32_t DecodeTemperatureAsInt(uint8_t* buf, uint16_t multiplier) {
    // MSB, LSB in Q8.4 format
    int32_t temperature = (short)((buf[0] << 8) | buf[1]);
    temperature >>= 4; // Discard unused bits
    temperature *= multiplier;
    temperature /= 16;
    return temperature;
}
static float DecodeTemperatureAsFloat(uint8_t* buf) {
    // MSB, LSB in Q8.4 format
    int16_t temperature = (short)((buf[0] << 8) | buf[1]);
    temperature >>= 4; // Discard unused bits
    return (float)temperature / 16.0f;
}

/* Internal Enums -------------------------------------------------------------*/
typedef enum {
    MPL_STATE_NOT_INIT = 0,
    MPL_STATE_READY_ALT,
    MPL_STATE_READY_PRES,
} MPL_STATE;

/* Static Data ----------------------------------------------------------------*/
static MPL_STATE state = MPL_STATE_NOT_INIT;

/* Initialization Functions -----------------------------------------------------------------*/

MPL_OP_STATUS MPL3115A2S_Init(MPL3115A2S_Config* const cfg, MPL3115A2S_Data_Config* const data_cfg, MPL3115A2S_Int_Config* const int_cfg) {
    // Check the device ID
    if (MPL3115A2S_CheckDeviceID() != MPL_OK) return MPL_ERR;

    // Decode the configuration
    uint8_t ctrl_reg1 = DecodeConfiguration(cfg);
    uint8_t pt_data_cfg = DecodeDataConfiguration(data_cfg);
    uint8_t ctrl_reg3 = DecodeIntConfiguration_CTRL3(int_cfg);
    uint8_t ctrl_reg4 = DecodeIntConfiguration_CTRL4(int_cfg);

    // Write the configuration
    if (I2C_WriteRegister(MPL3115A2S_ADDR, MPL3115A2S_CTRL_REG1, ctrl_reg1)) return MPL_ERR;
    if (I2C_WriteRegister(MPL3115A2S_ADDR, MPL3115A2S_PT_DATA_CFG, pt_data_cfg)) return MPL_ERR;
    if (I2C_WriteRegister(MPL3115A2S_ADDR, MPL3115A2S_CTRL_REG3, ctrl_reg3)) return MPL_ERR;
    if (I2C_WriteRegister(MPL3115A2S_ADDR, MPL3115A2S_CTRL_REG4, ctrl_reg4)) return MPL_ERR;

    // Set active mode
    ctrl_reg1 |= MPL3115A2S_CTRL_ACTIVE;
    if (I2C_WriteRegister(MPL3115A2S_ADDR, MPL3115A2S_CTRL_REG1, ctrl_reg1)) return MPL_ERR;

    if(cfg->altimeter_mode) state = MPL_STATE_READY_ALT;
    else state = MPL_STATE_READY_PRES;

    return MPL_OK;
}

/* Read Data Functions (combination, float) -----------------------------------------*/

MPL_OP_STATUS MPL3115A2S_ReadDataPolling(float* pres_alt, float* temp) {
    if(state == MPL_STATE_NOT_INIT) return MPL_NOT_INIT;

    // Check if data is ready
    if (MPL3115A2S_DataReady() != MPL_OK) return MPL_DATA_NOT_READY;

    uint8_t buf[5];
    if (I2C_ReadRegisters(MPL3115A2S_ADDR, MPL3115A2S_OUT_P_MSB, buf, 5)) return MPL_ERR;

    if(state == MPL_STATE_READY_PRES)
        *pres_alt = DecodePressureAsFloat(buf);
    else
        *pres_alt = DecodeAltitudeAsFloat(buf);

    *temp = DecodeTemperatureAsFloat(buf + 3);
    return MPL_OK;
}

MPL_OP_STATUS MPL3115A2S_ReadDataExtInterrupt(float* pres_alt, float* temp)  {
    if(state == MPL_STATE_NOT_INIT) return MPL_NOT_INIT;

    // Check INT_SOURCE register for data ready
    if(MPL3115A2S_DataReadyIT() != MPL_OK) return MPL_DATA_NOT_READY;

    uint8_t buf[5];
    if (I2C_ReadRegisters(MPL3115A2S_ADDR, MPL3115A2S_OUT_P_MSB, buf, 5)) return MPL_ERR;

    if(state == MPL_STATE_READY_PRES)
        *pres_alt = DecodePressureAsFloat(buf);
    else
        *pres_alt = DecodeAltitudeAsFloat(buf);
    
    *temp = DecodeTemperatureAsFloat(buf + 3);
    return MPL_OK;
}

/* Read Data Functions (float) ---------------------------------------------------------------*/

MPL_OP_STATUS MPL3115A2S_ReadPressure(float* pressure) {
    if(state == MPL_STATE_NOT_INIT) return MPL_NOT_INIT;
    else if(state == MPL_STATE_READY_ALT) return MPL_INVALID_SETTING; // Cannot read pressure in altimeter mode

    uint8_t buf[3];
    if (I2C_ReadRegisters(MPL3115A2S_ADDR, MPL3115A2S_OUT_P_MSB, buf, 3)) return MPL_ERR;
    *pressure = DecodePressureAsFloat(buf);
    return MPL_OK;
}

MPL_OP_STATUS MPL3115A2S_ReadAltitude(float* altitude) {
    if(state == MPL_STATE_NOT_INIT) return MPL_NOT_INIT;
    else if(state == MPL_STATE_READY_PRES) return MPL_INVALID_SETTING; // Cannot read altitude in barometer mode

    uint8_t buf[3];
    if (I2C_ReadRegisters(MPL3115A2S_ADDR, MPL3115A2S_OUT_P_MSB, buf, 3)) return MPL_ERR;
    *altitude = DecodeAltitudeAsFloat(buf);
    return MPL_OK;
}

MPL_OP_STATUS MPL3115A2S_ReadTemperature(float* temperature) {
    if(state == MPL_STATE_NOT_INIT) return MPL_NOT_INIT;

    uint8_t buf[2];
    if (I2C_ReadRegisters(MPL3115A2S_ADDR, MPL3115A2S_OUT_T_MSB, buf, 2)) return MPL_ERR;
    *temperature = DecodeTemperatureAsFloat(buf);
    return MPL_OK;
}

/* Read Data Functions (integer) ------------------------------------------------------------*/

MPL_OP_STATUS MPL3115A2S_ReadPressureAsInt(int32_t* pressure, uint16_t multiplier) {
    if(state == MPL_STATE_NOT_INIT) return MPL_NOT_INIT;
    else if(state == MPL_STATE_READY_ALT) return MPL_INVALID_SETTING; // Cannot read pressure in altimeter mode

    uint8_t buf[3];
    if (I2C_ReadRegisters(MPL3115A2S_ADDR, MPL3115A2S_OUT_P_MSB, buf, 3)) return MPL_ERR;
    *pressure = DecodePressureAsInt(buf, multiplier);
    return MPL_OK;
}

MPL_OP_STATUS MPL3115A2S_ReadAltitudeAsInt(int32_t* altitude, uint16_t multiplier) {
    if(state == MPL_STATE_NOT_INIT) return MPL_NOT_INIT;
    else if(state == MPL_STATE_READY_PRES) return MPL_INVALID_SETTING; // Cannot read altitude in barometer mode

    uint8_t buf[3];
    if (I2C_ReadRegisters(MPL3115A2S_ADDR, MPL3115A2S_OUT_P_MSB, buf, 3)) return MPL_ERR;
    *altitude = DecodeAltitudeAsInt(buf, multiplier);
    return MPL_OK;
}

MPL_OP_STATUS MPL3115A2S_ReadTemperatureAsInt(int32_t* temperature, uint16_t multiplier) {
    if(state == MPL_STATE_NOT_INIT) return MPL_NOT_INIT;

    uint8_t buf[2];
    if (I2C_ReadRegisters(MPL3115A2S_ADDR, MPL3115A2S_OUT_T_MSB, buf, 2)) return MPL_ERR;
    *temperature = DecodeTemperatureAsInt(buf, multiplier);
    return MPL_OK;
}

/* Control Functions -----------------------------------------------------------------------*/

MPL_OP_STATUS MPL3115A2S_Reset() {
    if (I2C_WriteRegister(MPL3115A2S_ADDR, MPL3115A2S_CTRL_REG1, MPL3115A2S_CTRL_RST)) return MPL_ERR;
    return MPL_OK;
}

MPL_OP_STATUS MPL3115A2S_CheckDeviceID() {
    uint8_t id = 0;
    if (I2C_ReadRegisters(MPL3115A2S_ADDR, MPL3115A2S_WHO_AM_I, &id, 1)) return MPL_ERR;
    if (id == MPL3115A2S_WHO_AM_I_ID) return MPL_OK;
    return MPL_INVALID_ID;
}

MPL_OP_STATUS MPL3115A2S_DataReady() {
    uint8_t status = 0;
    if (I2C_ReadRegisters(MPL3115A2S_ADDR, MPL3115A2S_STATUS, &status, 1)) return MPL_ERR;
    if (status & 0x08) return MPL_OK;
    return MPL_DATA_NOT_READY;
}

MPL_OP_STATUS MPL3115A2S_ReadIntSource(uint8_t* int_source) {
    if (I2C_ReadRegisters(MPL3115A2S_ADDR, MPL3115A2S_STATUS, int_source, 1)) return MPL_ERR;
    return MPL_OK;
}

MPL_OP_STATUS MPL3115A2S_DataReadyIT() {
    uint8_t int_source = 0;
    if (MPL3115A2S_ReadIntSource(&int_source) != MPL_OK) return MPL_ERR;
    if (int_source & 0x80) return MPL_OK;
    return MPL_DATA_NOT_READY;
}

MPL_OP_STATUS MPL3115A2S_IsAltimeterMode() {
    uint8_t ctrl_reg1 = 0;
    if (I2C_ReadRegisters(MPL3115A2S_ADDR, MPL3115A2S_CTRL_REG1, &ctrl_reg1, 1)) return MPL_ERR;

    // Make sure this matches the current state
    if((ctrl_reg1 >> MPL3115A2S_CTRL_ALT_SHIFT) & 0x01) {
        if(state == MPL_STATE_READY_ALT) return MPL_OK;
    } else {
        if(state == MPL_STATE_READY_PRES) {
            state = MPL_STATE_READY_ALT;
            return MPL_ERR;
        }
    }

    return MPL_INVALID_SETTING;
}
