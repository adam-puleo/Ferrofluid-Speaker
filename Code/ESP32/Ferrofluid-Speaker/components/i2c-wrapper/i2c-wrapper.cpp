#include "i2c-wrapper.hpp"

#define LOG_LOCAL_LEVEL ESP_LOG_WARN
const char* i2c_wrapper_TAG = "i2c_wrapper";
#include "esp_log.h"

// Constructor
i2c_wrapper::i2c_wrapper(i2c_master_bus_handle_t *bus_handle,
                         const uint32_t frequency,
                         const uint16_t device_address,
                         const int xfer_timeout_ms) {
    this->bus_handle = bus_handle;
    this->xfer_timeout_ms = xfer_timeout_ms;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = frequency,
        .scl_wait_us = 0,
        .flags{},
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_cfg, &this->dev_handle));
}

// Destructor
i2c_wrapper::~i2c_wrapper() {
    i2c_master_bus_rm_device(this->dev_handle);
}

esp_err_t i2c_wrapper::send_i2c_commands(const i2c_rw_t operation,
                                         const uint8_t data_to_write[],
                                         const size_t size_write_data,
                                         uint8_t *buffer,
                                         const size_t buffer_size) {
    esp_err_t result;

    if (data_to_write == nullptr) {
        ESP_LOGE(i2c_wrapper_TAG, "ERROR: Trying to send I2C command but data_to_write is null.");
        return ESP_ERR_INVALID_ARG;
    }
    if (size_write_data == 0) {
        ESP_LOGE(i2c_wrapper_TAG, "ERROR: Trying to send I2C command but size_write_data is zero.");
        return ESP_ERR_INVALID_ARG;
    }

    if (operation == I2C_MASTER_READ) {
        if (buffer == nullptr) {
            ESP_LOGE(i2c_wrapper_TAG, "ERROR: I2C read operation with no buffer.");
            return ESP_ERR_INVALID_ARG;
        }
        if (buffer_size == 0) {
            ESP_LOGE(i2c_wrapper_TAG, "ERROR: I2C read operation with buffer size of zero.");
            return ESP_ERR_INVALID_ARG;
        }
    }

    /*
        Dummy loop so that common cleanup code can be after the loop.
    */
    result = i2c_master_transmit(this->dev_handle, data_to_write, size_write_data, this->xfer_timeout_ms);
    if (result != ESP_OK) {
        ESP_LOGE(i2c_wrapper_TAG, "ERROR: Could not write I2C data: %d", result);
        return result;
    }

    if (operation == I2C_MASTER_READ) {
        result = i2c_master_receive(this->dev_handle, buffer, buffer_size, this->xfer_timeout_ms);
        if (result != ESP_OK) {
            ESP_LOGE(i2c_wrapper_TAG, "ERROR: Could not read I2C data: %d", result);
            return result;
        }
    }

    return result;
}

esp_err_t i2c_wrapper::read_multi_regs(const uint8_t start_reg,
                                       const uint8_t num_regs,
                                       uint8_t *buffer) {
    esp_err_t result = ESP_OK;

    for (uint8_t reg = start_reg; reg < start_reg + num_regs; reg++) {
        result = send_i2c_commands(I2C_MASTER_READ,
                                   &reg, sizeof(reg),
                                   &buffer[reg - start_reg], sizeof(uint8_t));
        if (result != ESP_OK) {
            ESP_LOGE(i2c_wrapper_TAG, "Could not read reg 0x%x.", reg);
            return result;
        }
    }

    return result;
}

esp_err_t i2c_wrapper::write_multi_regs(const uint8_t start_reg,
                                        const uint8_t num_regs,
                                        const uint8_t input_bytes[]) {
    esp_err_t result = ESP_OK;
    uint8_t buffer[2];

    for (uint8_t reg = start_reg; reg < start_reg + num_regs; reg++) {
        buffer[0] = reg;
        buffer[1] = input_bytes[reg - start_reg];
        result = send_i2c_commands(I2C_MASTER_WRITE,
                                   buffer, sizeof(buffer),
                                   NULL, 0);
        if (result != ESP_OK) {
            ESP_LOGE(i2c_wrapper_TAG, "Could not write reg 0x%x.", reg);
            return result;
        }
    }

    return result;
}
