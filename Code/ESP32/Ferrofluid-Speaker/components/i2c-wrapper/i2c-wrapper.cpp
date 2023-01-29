#include "i2c-wrapper.hpp"

#define LOG_LOCAL_LEVEL ESP_LOG_WARN
const char* i2c_wrapper_TAG = "i2c_wrapper";
#include "esp_log.h"

// Constructor
i2c_wrapper::i2c_wrapper(const i2c_port_t port,
                         const uint32_t frequency,
                         const gpio_num_t scl_pin,
                         const gpio_num_t sda_pin) {
    this->PORT = port;
    this->FREQUENCY = frequency;
    this->SCL_PIN = scl_pin;
    this->SDA_PIN = sda_pin;

    // Enable I2C bus.
    const i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = false,
        .scl_pullup_en = false,
        .master={.clk_speed = FREQUENCY},
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(this->PORT, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(this->PORT, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
}

// Destructor
i2c_wrapper::~i2c_wrapper() {
}

esp_err_t i2c_wrapper::send_i2c_commands(const uint8_t device_address,
                                         const i2c_rw_t operation,
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

    /* Setup batch of commands to be sent */
    i2c_cmd_handle_t i2c_cmd_handle = i2c_cmd_link_create();
    result = i2c_master_start(i2c_cmd_handle);
    if (result != ESP_OK) {
        ESP_LOGE(i2c_wrapper_TAG, "ERROR: Could not start I2C: %d", result);
        return result;
    }

    /*
        Dummy loop so that common cleanup code can be after the loop.
    */
    do {
        result = i2c_master_write_byte(i2c_cmd_handle, device_address, true);
        if (result != ESP_OK) {
            ESP_LOGE(i2c_wrapper_TAG, "ERROR: Could not write I2C write addr: %d", result);
            break;
        }

        result = i2c_master_write(i2c_cmd_handle, data_to_write, size_write_data, true);
        if (result != ESP_OK) {
            ESP_LOGE(i2c_wrapper_TAG, "ERROR: Could not write I2C data: %d", result);
            break;
        }

        if (operation == I2C_MASTER_READ) {
            result = i2c_master_start(i2c_cmd_handle);
            if (result != ESP_OK) {
                ESP_LOGE(i2c_wrapper_TAG, "ERROR: Could not second start I2C: %d", result);
                return result;
            }
            
            result = i2c_master_write_byte(i2c_cmd_handle, device_address | 1, true);
            if (result != ESP_OK) {
                ESP_LOGE(i2c_wrapper_TAG, "ERROR: Could not write I2C addr in prep for read: %d", result);
                break;
            }

            result = i2c_master_read(i2c_cmd_handle, buffer, buffer_size, I2C_MASTER_NACK);
            if (result != ESP_OK) {
                ESP_LOGE(i2c_wrapper_TAG, "ERROR: Reading I2C data.");
                break;
            }
        }

        result = i2c_master_stop(i2c_cmd_handle);
        if (result != ESP_OK) {
            ESP_LOGE(i2c_wrapper_TAG, "ERROR: Could not stop I2C: %d", result);
            break;
        }

        result = i2c_master_cmd_begin(this->PORT, i2c_cmd_handle, this->I2C_WAIT);
        if (result != ESP_OK) {
            ESP_LOGE(i2c_wrapper_TAG, "ERROR: Could not issue I2C stream commands: %s", esp_err_to_name(result));
            break;
        }
    } while (false);

    i2c_cmd_link_delete(i2c_cmd_handle);

    return result;
}

esp_err_t i2c_wrapper::read_multi_regs(const uint8_t device_address,
                                       const uint8_t start_reg,
                                       const uint8_t num_regs,
                                       uint8_t *buffer) {
    esp_err_t result = ESP_OK;

    for (uint8_t reg = start_reg; reg < start_reg + num_regs; reg++) {
        result = send_i2c_commands(device_address,
                                   I2C_MASTER_READ,
                                   &reg, sizeof(reg),
                                   &buffer[reg - start_reg], sizeof(uint8_t));
        if (result != ESP_OK) {
            ESP_LOGE(i2c_wrapper_TAG, "Could not read reg 0x%x.", reg);
            return result;
        }
    }

    return result;
}

esp_err_t i2c_wrapper::write_multi_regs(const uint8_t device_address,
                                        const uint8_t start_reg,
                                        const uint8_t num_regs,
                                        const uint8_t input_bytes[]) {
    esp_err_t result = ESP_OK;
    uint8_t buffer[2];

    for (uint8_t reg = start_reg; reg < start_reg + num_regs; reg++) {
        buffer[0] = reg;
        buffer[1] = input_bytes[reg - start_reg];
        result = send_i2c_commands(device_address,
                                   I2C_MASTER_WRITE,
                                   buffer, sizeof(buffer),
                                   NULL, 0);
        if (result != ESP_OK) {
            ESP_LOGE(i2c_wrapper_TAG, "Could not write reg 0x%x.", reg);
            return result;
        }
    }

    return result;
}
