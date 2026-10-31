#pragma once

#include <stdint.h>

/**
 * @file BMI160_Registers.h
 * @brief Streamlined register addresses, command codes, and constants for Bosch BMI160 IMU.
 */

namespace BMI160_ESPx_Defs {

    // I2C Addresses & Chip Identification
    static constexpr uint8_t I2C_ADDR_LOWER         = 0x68; // SA0 / SDO pin connected to GND
    static constexpr uint8_t I2C_ADDR_UPPER         = 0x69; // SA0 / SDO pin connected to VDDIO (3.3V)
    static constexpr uint8_t CHIP_ID_VALUE          = 0xD1; // Expected value in REG_CHIP_ID

    // Primary Sensor Registers
    static constexpr uint8_t REG_CHIP_ID            = 0x00; // Chip ID register (0xD1)
    static constexpr uint8_t REG_ERR_REG            = 0x02; // Error register
    static constexpr uint8_t REG_PMU_STATUS         = 0x03; // Power mode status
    static constexpr uint8_t REG_DATA_GYRO          = 0x0C; // 6 bytes gyroscope data (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    static constexpr uint8_t REG_DATA_ACCEL         = 0x12; // 6 bytes accelerometer data (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    static constexpr uint8_t REG_STATUS             = 0x1B; // Status register
    static constexpr uint8_t REG_TEMP_L             = 0x20; // Temperature output LSB
    static constexpr uint8_t REG_TEMP_H             = 0x21; // Temperature output MSB

    // Configuration Registers
    static constexpr uint8_t REG_ACC_CONF           = 0x40; // Accel ODR & Bandwidth
    static constexpr uint8_t REG_ACC_RANGE          = 0x41; // Accel g-range
    static constexpr uint8_t REG_GYR_CONF           = 0x42; // Gyro ODR & Bandwidth
    static constexpr uint8_t REG_GYR_RANGE          = 0x43; // Gyro angular rate range

    // Command Register & Opcodes
    static constexpr uint8_t REG_CMD                = 0x7E; // Command register
    static constexpr uint8_t CMD_ACC_PM_NORMAL      = 0x11; // Wake up Accel to normal mode
    static constexpr uint8_t CMD_GYR_PM_NORMAL      = 0x15; // Wake up Gyro to normal mode
    static constexpr uint8_t CMD_SOFT_RESET         = 0xB6; // Trigger software reset
}
