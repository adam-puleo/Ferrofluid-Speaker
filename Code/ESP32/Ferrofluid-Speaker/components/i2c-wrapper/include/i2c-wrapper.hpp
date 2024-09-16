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

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

/** 
 * @brief Class to simplify code when using the ESP's I2C library.
 * @author Adam Puleo
 * @copyright Apache License Version 2
*/
class i2c_wrapper {
    public:
        /**
         * @brief Construct a new i2c wrapper object for communicating with an I2C slave. The caller is responsible for calling i2c_new_master_bus.
         * 
         * @param bus_handle Pointer to the mater bus handle as returned by i2c_new_master_bus.
         * @param frequency SCL line frequency.
         * @param device_address Raw 7-bit address without the write/read bit for the slave device to communicate with.
         * @param xfer_timeout_ms Wait timeout, in ms. Note: -1 means wait forever.
         */
        i2c_wrapper(i2c_master_bus_handle_t *bus_handle,
                    const uint32_t frequency,
                    const uint16_t device_address,
                    const int xfer_timeout_ms);

        /**
         * @brief Send I2C commands to the remote device. data_to_write needs the register and any parameters.
         *
         * @param operation Read or write data to the remote device.
         * @param data_to_write Register and possibly data to send to the remote device.
         * @param size_write_data Number of bytes in data_to_write.
         * @param buffer If this is a read operation, buffer to store that data, else NULL.
         * @param buffer_size Size of buffer in bytes if this is a read operation, else 0.
         *
         * @return ESP_OK on success. If there is an error, passes the error code back.
         */
        esp_err_t send_i2c_commands(const i2c_rw_t operation,
                                    const uint8_t *data_to_write,
                                    const size_t size_write_data,
                                    uint8_t *buffer,
                                    const size_t buffer_size);

        /**
         * @brief Read multiple 8bit registers and return the bytes in buffer.
         * 
         * @param start_reg Register to start reading at.
         * @param num_regs Number of registers to read.
         * @param buffer Buffer to store the bytes in.
         * @return esp_err_t ESP_OK on success. If there is an error, passes the error code back.
         */
        esp_err_t read_multi_regs(const uint8_t start_reg,
                                  const uint8_t num_regs,
                                  uint8_t *buffer);

        /**
         * @brief Writes multiple 8bit registers.
         * 
         * @param start_reg Register to start reading at.
         * @param num_regs Number of registers to read.
         * @param input_bytes Bytes to write.
         * @return esp_err_t ESP_OK on success. If there is an error, passes the error code back.
         */
        esp_err_t write_multi_regs(const uint8_t start_reg,
                                   const uint8_t num_regs,
                                   const uint8_t input_bytes[]);

        // Destructor
        ~i2c_wrapper();

    private:
        i2c_master_bus_handle_t *bus_handle;
        i2c_master_dev_handle_t dev_handle;
        int xfer_timeout_ms;
};
