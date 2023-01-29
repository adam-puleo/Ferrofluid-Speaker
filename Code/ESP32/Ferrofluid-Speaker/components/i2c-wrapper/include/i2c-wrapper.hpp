/**
 * @file i2c-wrapper.hpp
 * @author Adam Puleo (adam.puleo@icloud.com)
 * @brief
 * @version 1.0.0
 * @date 2022-11-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "driver/i2c.h"

/** 
 * @brief Class to simplify code when using the ESP's I2C library.
 * @author Adam Puleo
 * @copyright Apache License Version 2
*/
class i2c_wrapper {
    public:
        // Constructor
        i2c_wrapper(const i2c_port_t port,
                    const uint32_t frequency,
                    const gpio_num_t scl_pin,
                    const gpio_num_t sda_pin);

        /**
         * @brief Send I2C commands to the remote device. data_to_write needs the register and any parameters.
         *
         * @param device_address 8-bit address of the device to write to. This function will calculate the read address.
         * @param operation Read or write data to the remote device
         * @param data_to_write Register and possibly data to send to the remote device.
         * @param size_write_data Number of bytes in data_to_write
         * @param buffer If this is a read operation, buffer to store that data, else NULL
         * @param buffer_size Size of buffer in bytes if this is a read operation, else 0
         *
         * @return ESP_OK on success. If there is an error, passes the error code back.
         */
        esp_err_t send_i2c_commands(const uint8_t device_address,
                                    const i2c_rw_t operation,
                                    const uint8_t *data_to_write,
                                    const size_t size_write_data,
                                    uint8_t *buffer,
                                    const size_t buffer_size);

        /**
         * @brief Read multiple 8bit registers and return the bytes in buffer.
         * 
         * @param device_address 8-bit address of the devices write address. This function will calculate the read address.
         * @param start_reg Register to start reading at.
         * @param num_regs Number of registers to read.
         * @param buffer Buffer to store the bytes in.
         * @return esp_err_t ESP_OK on success. If there is an error, passes the error code back.
         */
        esp_err_t read_multi_regs(const uint8_t device_address,
                                  const uint8_t start_reg,
                                  const uint8_t num_regs,
                                  uint8_t *buffer);

        /**
         * @brief Writes multiple 8bit registers.
         * 
         * @param device_address 8-bit address of the devices write address. This function will calculate the read address.
         * @param start_reg Register to start reading at.
         * @param num_regs Number of registers to read.
         * @param input_bytes Bytes to write.
         * @return esp_err_t ESP_OK on success. If there is an error, passes the error code back.
         */
        esp_err_t write_multi_regs(const uint8_t device_address,
                                   const uint8_t start_reg,
                                   const uint8_t num_regs,
                                   const uint8_t input_bytes[]);

        // Destructor
        ~i2c_wrapper();

    private:
        // Configuration parameters.
        i2c_port_t PORT;
        uint32_t FREQUENCY;
        gpio_num_t SCL_PIN;
        gpio_num_t SDA_PIN;

        // Setup config constants for ESP's I2C communications.
        const size_t I2C_MASTER_TX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
        const size_t I2C_MASTER_RX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
        const TickType_t I2C_WAIT = 1000 / portTICK_RATE_MS;  // A second should be more than enough time?
};
