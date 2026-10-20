#pragma once

#include <stdint.h>

/**
 * @file BMI160_Registers.h
 * @brief Register addresses, command codes, and bitmask constants for Bosch BMI160 IMU.
 */

namespace BMI160_ESPx_Defs {

    // I2C Addresses
    static constexpr uint8_t I2C_ADDR_LOWER         = 0x68; // SA0 / SDO pin connected to GND
    static constexpr uint8_t I2C_ADDR_UPPER         = 0x69; // SA0 / SDO pin connected to VDDIO (3.3V)
    static constexpr uint8_t CHIP_ID_VALUE          = 0xD1; // Expected value in REG_CHIP_ID

    // Primary Registers
    static constexpr uint8_t REG_CHIP_ID            = 0x00; // Chip ID register (0xD1)
    static constexpr uint8_t REG_ERR_REG            = 0x02; // Error register
    static constexpr uint8_t REG_PMU_STATUS         = 0x03; // Power mode status
    static constexpr uint8_t REG_DATA_MAG           = 0x04; // 8 bytes aux magnetometer data
    static constexpr uint8_t REG_DATA_GYRO          = 0x0C; // 6 bytes gyroscope data (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    static constexpr uint8_t REG_DATA_ACCEL         = 0x12; // 6 bytes accelerometer data (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    static constexpr uint8_t REG_SENSORTIME_0       = 0x18; // 24-bit sensor time (byte 0)
    static constexpr uint8_t REG_SENSORTIME_1       = 0x19; // 24-bit sensor time (byte 1)
    static constexpr uint8_t REG_SENSORTIME_2       = 0x1A; // 24-bit sensor time (byte 2)
    static constexpr uint8_t REG_STATUS             = 0x1B; // Status register
    static constexpr uint8_t REG_INT_STATUS_0       = 0x1C; // Interrupt status 0
    static constexpr uint8_t REG_INT_STATUS_1       = 0x1D; // Interrupt status 1
    static constexpr uint8_t REG_INT_STATUS_2       = 0x1E; // Interrupt status 2
    static constexpr uint8_t REG_INT_STATUS_3       = 0x1F; // Interrupt status 3
    static constexpr uint8_t REG_TEMP_L             = 0x20; // Temperature output LSB
    static constexpr uint8_t REG_TEMP_H             = 0x21; // Temperature output MSB
    static constexpr uint8_t REG_FIFO_LENGTH_0      = 0x22; // FIFO byte counter LSB
    static constexpr uint8_t REG_FIFO_LENGTH_1      = 0x23; // FIFO byte counter MSB
    static constexpr uint8_t REG_FIFO_DATA          = 0x24; // FIFO read data port

    // Configuration Registers
    static constexpr uint8_t REG_ACC_CONF           = 0x40; // Accel ODR & Bandwidth
    static constexpr uint8_t REG_ACC_RANGE          = 0x41; // Accel g-range
    static constexpr uint8_t REG_GYR_CONF           = 0x42; // Gyro ODR & Bandwidth
    static constexpr uint8_t REG_GYR_RANGE          = 0x43; // Gyro angular rate range
    static constexpr uint8_t REG_MAG_CONF           = 0x44; // Aux magnetometer rate
    static constexpr uint8_t REG_FIFO_DOWNS         = 0x45; // FIFO downsampling
    static constexpr uint8_t REG_FIFO_CONFIG_0      = 0x46; // FIFO watermark
    static constexpr uint8_t REG_FIFO_CONFIG_1      = 0x47; // FIFO data selection

    // Interrupt Configuration & Mapping
    static constexpr uint8_t REG_INT_EN_0           = 0x50; // Interrupt enable 0 (any-motion, double-tap, etc.)
    static constexpr uint8_t REG_INT_EN_1           = 0x51; // Interrupt enable 1 (high-g, data-ready)
    static constexpr uint8_t REG_INT_EN_2           = 0x52; // Interrupt enable 2 (step detector, nomotion)
    static constexpr uint8_t REG_INT_OUT_CTRL       = 0x53; // INT1 / INT2 pin electrical properties
    static constexpr uint8_t REG_INT_LATCH          = 0x54; // Latch duration configuration
    static constexpr uint8_t REG_INT_MAP_0          = 0x55; // Map interrupts to INT1 pin
    static constexpr uint8_t REG_INT_MAP_1          = 0x56; // Map interrupts to INT2 pin
    static constexpr uint8_t REG_INT_MAP_2          = 0x57; // Map data-ready & FIFO to INT1/INT2
    static constexpr uint8_t REG_INT_DATA_0         = 0x58; // Low/High-g thresholds
    static constexpr uint8_t REG_INT_MOTION_0       = 0x5F; // Any-motion threshold
    static constexpr uint8_t REG_INT_MOTION_1       = 0x60; // Any-motion duration & high-g
    static constexpr uint8_t REG_INT_TAP_0          = 0x63; // Tap duration / shock
    static constexpr uint8_t REG_INT_TAP_1          = 0x64; // Tap threshold

    // Hardware FOC & Offsets
    static constexpr uint8_t REG_FOC_CONF           = 0x69; // Fast Offset Compensation configuration
    static constexpr uint8_t REG_CONF               = 0x6A; // General configuration (pull-ups, etc.)
    static constexpr uint8_t REG_OFFSET_ACC_X       = 0x71; // Accelerometer X offset (7:0)
    static constexpr uint8_t REG_OFFSET_ACC_Y       = 0x72; // Accelerometer Y offset (7:0)
    static constexpr uint8_t REG_OFFSET_ACC_Z       = 0x73; // Accelerometer Z offset (7:0)
    static constexpr uint8_t REG_OFFSET_GYR_X       = 0x74; // Gyroscope X offset (7:0)
    static constexpr uint8_t REG_OFFSET_GYR_Y       = 0x75; // Gyroscope Y offset (7:0)
    static constexpr uint8_t REG_OFFSET_GYR_Z       = 0x76; // Gyroscope Z offset (7:0)
    static constexpr uint8_t REG_OFFSET_EN          = 0x77; // Offset enable bits

    // Step Counter & Detector
    static constexpr uint8_t REG_STEP_CNT_0         = 0x78; // Step counter LSB
    static constexpr uint8_t REG_STEP_CNT_1         = 0x79; // Step counter MSB
    static constexpr uint8_t REG_STEP_CONF_0        = 0x7A; // Step config 0 (mode tuning)
    static constexpr uint8_t REG_STEP_CONF_1        = 0x7B; // Step config 1 (enable bit)

    // Command Register & Opcodes
    static constexpr uint8_t REG_CMD                = 0x7E; // Command register

    static constexpr uint8_t CMD_START_FOC          = 0x03; // Start Fast Offset Compensation
    static constexpr uint8_t CMD_ACC_PM_SUSPEND     = 0x10; // Put Accel into suspend mode
    static constexpr uint8_t CMD_ACC_PM_NORMAL      = 0x11; // Put Accel into normal mode
    static constexpr uint8_t CMD_ACC_PM_LOWPOWER    = 0x12; // Put Accel into low-power mode
    static constexpr uint8_t CMD_GYR_PM_SUSPEND     = 0x14; // Put Gyro into suspend mode
    static constexpr uint8_t CMD_GYR_PM_NORMAL      = 0x15; // Put Gyro into normal mode
    static constexpr uint8_t CMD_GYR_PM_FASTSTART   = 0x17; // Put Gyro into fast-startup mode
    static constexpr uint8_t CMD_STEP_CNT_CLR       = 0xB2; // Clear step counter
    static constexpr uint8_t CMD_FIFO_FLUSH         = 0xB0; // Flush FIFO buffer
    static constexpr uint8_t CMD_INT_RESET          = 0xB1; // Reset interrupt latch
    static constexpr uint8_t CMD_SOFT_RESET         = 0xB6; // Trigger software reset
}
