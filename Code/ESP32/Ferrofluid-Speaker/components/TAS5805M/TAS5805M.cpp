#include <string.h>

#include "TAS5805M.hpp"
#include "FreeRTOSConfig.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
const char* TAS5805M_TAG = "TAS5805M";
#include "esp_log.h"

// Constructor
TAS5805M::TAS5805M(const uint8_t write_address,
                   const i2c_port_t port,
                   const uint32_t frequency,
                   const gpio_num_t pdn_pin,
                   const gpio_num_t fault_pin,
                   const gpio_num_t scl_pin,
                   const gpio_num_t sda_pin) {
    this->AMP_WRITE_ADDR = write_address;
    this->AMP_READ_ADDR = write_address | 1;
    this->PORT = port;
    this->FREQUENCY = frequency;
    this->PDN = pdn_pin;
    this->FAULT = fault_pin;
    this->SCL_PIN = scl_pin;
    this->SDA_PIN = sda_pin;

    /* 2.0 Stereo BTL system, Figure 8-2, section 8.2.1.1 */
    /* See 7.5.2.5 DSP Memory Book, Page about how to address memory */
    /* See 7.5.3.1 Startup Procedures */

    gpio_config_t io_conf = {};
    // Set the bit mask for the PDN output pin. 
    io_conf.pin_bit_mask = (1ULL << PDN);
    // Set pin as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // Disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // Disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // Configure GPIO pin with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Reset the variable
    io_conf = {};
    // Set the bit mask for the input pins.
    io_conf.pin_bit_mask = (1ULL << FAULT);
    // Set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // Configure GPIO pins with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Enable I2C so we can control the TAS5805M.
    const i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = false,
        .scl_pullup_en = false,
        .master={.clk_speed = FREQUENCY},
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(PORT, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(PORT, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
}

esp_err_t TAS5805M::start_pre_i2s() {
    esp_err_t result;

    // We do not know what state the chip is in, make sure it's powered down before starting it.
    result = stop();
    if (result != ESP_OK) {
        return result;
    }

    // Enable the amp (TAS5805M).
    result = gpio_set_level(PDN, 1);
    if (result == ESP_OK) {
        state = DEEP_SLEEP;
        // Per specification, wait at least 5ms before enabling the I2S clocks.
        vTaskDelay(pdMS_TO_TICKS(10));
    } else {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Could not bring PDN high: %d", result);
    }
    return result;
}

esp_err_t TAS5805M::start_post_i2s() {
    esp_err_t result;

    // Wait 100ms for I2S to settle, just for giggles.
    //vTaskDelay(pdMS_TO_TICKS(100));

    /* Send commands to initialize the TAS5805M. */

    /* Section 7.5.2.5 of the TAS5805M Data Sheet explains pages and books. */
    /* Go to page zero. */
    const uint8_t PAGE_ZERO[] = {PAGE_REG, 0x00};
    result = send_i2c_commands(I2C_MASTER_WRITE,
                               PAGE_ZERO, sizeof(PAGE_ZERO),
                               NULL, 0);
    if (result != ESP_OK) {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Could not move to page zero: %s", esp_err_to_name(result));
        return result;
    }

    /* Go to book zero. */
    const uint8_t BOOK_ZERO[] = {BOOK_REG, 0x00};
    result = send_i2c_commands(I2C_MASTER_WRITE,
                               BOOK_ZERO, sizeof(BOOK_ZERO),
                               NULL, 0);
    if (result != ESP_OK) {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Could not move to book zero: %s", esp_err_to_name(result));
        return result;
    }

    /* Go to page zero. */
    result = send_i2c_commands(I2C_MASTER_WRITE,
                               PAGE_ZERO, sizeof(PAGE_ZERO),
                               NULL, 0);
    if (result != ESP_OK) {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Could not move to page zero of book zero: %s", esp_err_to_name(result));
        return result;
    }

    /* Move amp to HiZ state */
    const uint8_t HiZ_CMDS[] = {DEVICE_CTRL_2, 0b10};
    result = send_i2c_commands(I2C_MASTER_WRITE,
                               HiZ_CMDS, sizeof(HiZ_CMDS),
                               NULL, 0);
    if (result != ESP_OK) {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Could not setup amp (HiZ): %s", esp_err_to_name(result));
        return result;
    }

    state = HiZ;

    /* Per specification (7.5.3.1) wait at least 5ms for device to settle. */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Set the volume to the middle of the road so we don't blast out someone's ears when we turn on. */
    const uint8_t VOL_CMDS[] = {DIG_VOL_CTL, 0b00110000};
    result = send_i2c_commands(I2C_MASTER_WRITE,
                               VOL_CMDS, sizeof(VOL_CMDS),
                               NULL, 0);

    /* Set the analog gain so that we do not go over 12v. */
    /* Set 0x54, AGAIN[4:0] to 12v, see section 7.3.8.1 & section 7.6.1.21
    12v system so gain (AGAIN) should be set to 0x15, to produce a peak output voltage of 12v. 
    Assuming linear: y=mx+b
    x=0; b=29.5
    x=31; y=4.95; m=-0.79
    x=(y-b)/m=(12-29.5)/-0.79=22
    Round up a little and AGAIN_REG should be 23. */
    const uint8_t AGAIN_CMDS[] = {AGAIN_REG, 0x17};
    result = send_i2c_commands(I2C_MASTER_WRITE, 
                               AGAIN_CMDS, sizeof(AGAIN_CMDS),
                               NULL, 0);
    if (result != ESP_OK) {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Could not setup amp (AGAIN): %s", esp_err_to_name(result));
        return result;
    }

    /* Setup the ADR/FAULT pin to signal when there is a fault. */
    /* Set ADR/FAULT pin to output fault, set reg 0x61 = 0x0b then 0x60 = 0x01, section 5 */
    const uint8_t FAULT1_CMDS[] = {ADR_PIN_CONFIG, 0b01011};
    result = send_i2c_commands(I2C_MASTER_WRITE, 
                               FAULT1_CMDS, sizeof(FAULT1_CMDS),
                               NULL, 0);
    if (result != ESP_OK) {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Could not setup amp (ADR/FAULT 1): %s", esp_err_to_name(result));
        return result;
    }

    /* Finish setting up the ADR/FAULT pin. */
    /* Set ADR/FAULT pin to output fault, set reg 0x61 = 0x0b then 0x60 = 0x01, section 5 */
    const uint8_t FAULT2_CMDS[] = {ADR_PIN_CTRL, 0b1};
    result = send_i2c_commands(I2C_MASTER_WRITE,
                               FAULT2_CMDS, sizeof(FAULT2_CMDS),
                               NULL, 0);
    if (result != ESP_OK) {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Could not setup amp (ADR/FAULT 2): %s", esp_err_to_name(result));
        return result;
    }

    /* Are these next two required? */
    /* 7.6.1.5 SIG_CH_CTRL Register (Offset = 28h) needs to be set to match I2S audio, defaults to auto detect. */

    /* 7.6.1.9 SAP_CTRL1 Register, does does word length need to be changed from default of 24 bits? */
    /* ESP defaults to 16. When the ESP is set to 24 it freaks out. */
    const uint8_t SAP_CTRL1_CMDS[] = {SAP_CTRL1, 0b00};
    result = send_i2c_commands(I2C_MASTER_WRITE,
                               SAP_CTRL1_CMDS, sizeof(SAP_CTRL1_CMDS),
                               NULL, 0);
    if (result != ESP_OK) {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Could not setup amp (I2S bits): %s", esp_err_to_name(result));
        return result;
    }

    /* Tell amp to start playing. */
    /* Play Mode. Register 0x03h -D[1:0]=11, device stays in Play Mode. See section 7.6.1.3 DEVICE_CTRL_2 Register */
    const uint8_t PLAY_CMDS[] = {DEVICE_CTRL_2, 0b11};
    result = send_i2c_commands(I2C_MASTER_WRITE,
                               PLAY_CMDS, sizeof(PLAY_CMDS),
                               NULL, 0);
    if (result != ESP_OK) {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Could not set the amp to play mode: %s", esp_err_to_name(result));
        return result;
    }

    state = ON;
    return result;
}

esp_err_t TAS5805M::stop() {
    // Stop the TAS5805M - Section 7.5.3.2.
    esp_err_t result = gpio_set_level(PDN, 0);
    if (result == ESP_OK) {
        state = HiZ;
        // Per specification, wait 6ms before powering down the chip.
        vTaskDelay(pdMS_TO_TICKS(12));
    } else {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Could not bring PDN low: %s", esp_err_to_name(result));
    }
    return result;
}

bool TAS5805M::error() {
    bool error = false;

    if (state == ON) {
        esp_err_t result;
        uint8_t buffer;
        uint8_t data;

        if (gpio_get_level(FAULT) == 0) {  // Fault is active low.
            error = true;
            /* 7.6.1.36 CHAN_FAULT Register, read to determine fault */
            buffer = CHAN_FAULT;
            result = send_i2c_commands(I2C_MASTER_READ,
                                       &buffer, 1,
                                       &data, 1);
            if (result == ESP_OK) {
                ESP_LOGI(TAS5805M_TAG, "CHAN_FAULT register: %x", data);
            } else {
                ESP_LOGE(TAS5805M_TAG, "ERROR: Could not read CHAN_FAULT Register: %s", esp_err_to_name(result));
                return result;
            }

            /* 7.6.1.37 GLOBAL_FAULT1 Register, read to determine fault */
            buffer = GLOBAL_FAULT1;
            result = send_i2c_commands(I2C_MASTER_READ,
                                       &buffer, 1,
                                       &data, 1);
            if (result == ESP_OK) {
                ESP_LOGI(TAS5805M_TAG, "GLOBAL_FAULT1 register: %x", data);
            } else {
                ESP_LOGE(TAS5805M_TAG, "ERROR: Could not read GLOBAL_FAULT1 Register: %s", esp_err_to_name(result));
                return result;
            }

            /* 7.6.1.38 GLOBAL_FAULT2 Register */
            buffer = GLOBAL_FAULT2;
            result = send_i2c_commands(I2C_MASTER_READ,
                                       &buffer, 1,
                                       &data, 1);
            if (result == ESP_OK) {
                ESP_LOGI(TAS5805M_TAG, "GLOBAL_FAULT2 register: %x", data);
            } else {
                ESP_LOGE(TAS5805M_TAG, "ERROR: Could not read GLOBAL_FAULT2 Register: %s", esp_err_to_name(result));
                return result;
            }

            buffer = CLKDET_STATUS;
            result = send_i2c_commands(I2C_MASTER_READ,
                                       &buffer, 1,
                                       &data, 1);
            if (result == ESP_OK) {
                ESP_LOGI(TAS5805M_TAG, "CLKDET_STATUS register: %x", data);
            } else {
                ESP_LOGE(TAS5805M_TAG, "ERROR: Could not read CLKDET_STATUS Register: %s", esp_err_to_name(result));
                return result;
            }

            buffer = FS_MON;
            result = send_i2c_commands(I2C_MASTER_READ,
                                       &buffer, 1,
                                       &data, 1);
            if (result == ESP_OK) {
                ESP_LOGI(TAS5805M_TAG, "FS_MON register: %x", data);
            } else {
                ESP_LOGE(TAS5805M_TAG, "ERROR: Could not read FS_MON Register: %s", esp_err_to_name(result));
                return result;
            }

            buffer = SIG_CH_CTRL;
            result = send_i2c_commands(I2C_MASTER_READ,
                                       &buffer, 1,
                                       &data, 1);
            if (result == ESP_OK) {
                ESP_LOGI(TAS5805M_TAG, "SIG_CH_CTRL register: %x", data);
            } else {
                ESP_LOGE(TAS5805M_TAG, "ERROR: Could not read SIG_CH_CTRL Register: %s", esp_err_to_name(result));
                return result;
            }

            buffer = DIE_ID;
            result = send_i2c_commands(I2C_MASTER_READ,
                                       &buffer, 1,
                                       &data, 1);
            if (result == ESP_OK) {
                ESP_LOGI(TAS5805M_TAG, "DIE_ID register: %x", data);
            } else {
                ESP_LOGE(TAS5805M_TAG, "ERROR: Could not read DIE_ID Register: %s", esp_err_to_name(result));
                return result;
            }

            buffer = POWER_STATE;
            result = send_i2c_commands(I2C_MASTER_READ,
                                       &buffer, 1,
                                       &data, 1);
            if (result == ESP_OK) {
                ESP_LOGI(TAS5805M_TAG, "POWER_STATE register: %x", data);
            } else {
                ESP_LOGE(TAS5805M_TAG, "ERROR: Could not read POWER_STATE Register: %s", esp_err_to_name(result));
                return result;
            }
        } else {
            /* Check for over temperature warning. */
            /* 7.6.1.39 OT WARNING Register (Offset = 73h) */
            buffer = OT_WARNING;
            result = send_i2c_commands(I2C_MASTER_READ,
                                       &buffer, 1,
                                       &data, 1);
            if (result == ESP_OK) {
                if ((data & 0b10) == 1) {
                    ESP_LOGI(TAS5805M_TAG, "OT register: %x", data);
                    error = true;
                }
            } else {
                ESP_LOGE(TAS5805M_TAG, "ERROR: Could not read the OT register: %s", esp_err_to_name(result));
            }
        }
    }

    return error;
}

// Destructor
TAS5805M::~TAS5805M() {
    // Need to issue commands to shutdown the amp.
    // See section 7.5.3.2
}

/**
 * @brief Send I2C commands to the remote device. data_to_write just needs the register and any parameters. Device address, book, and page bytes are added on automatically by this function.
 *
 * @param cmd_handle I2C commands list
 * @param operation Read or write data to the remote device
 * @param data_to_write Just register and possibly data to send to the remote device. Device address, book, and page bytes are added on automatically by this function.
 * @param size_write_data Number of bytes in data_to_write
 * @param buffer If this is a read operation, buffer to store that data, else NULL
 * @param buffer_size Size of buffer in bytes if this is a read operation, else 0
 *
 * @return
 *     ESP_OK on success. If there is an error, passes the error code back.
 */
esp_err_t TAS5805M::send_i2c_commands(const i2c_rw_t operation,
                                      const uint8_t data_to_write[],
                                      const size_t size_write_data,
                                      uint8_t *buffer,
                                      const size_t buffer_size) {
    esp_err_t result;

    if (data_to_write == nullptr) {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Trying to send I2C command but data_to_write is null.");
        return ESP_ERR_INVALID_ARG;
    }
    if (size_write_data == 0) {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Trying to send I2C command but size_write_data is zero.");
        return ESP_ERR_INVALID_ARG;
    }

    if (operation == I2C_MASTER_READ) {
        if (buffer == nullptr) {
            ESP_LOGE(TAS5805M_TAG, "ERROR: I2C read operation with no buffer.");
            return ESP_ERR_INVALID_ARG;
        }
        if (buffer_size == 0) {
            ESP_LOGE(TAS5805M_TAG, "ERROR: I2C read operation with buffer size of zero.");
            return ESP_ERR_INVALID_ARG;
        }
    }

    /* Setup batch of commands to be sent */
    i2c_cmd_handle_t i2c_cmd_handle = i2c_cmd_link_create();
    result = i2c_master_start(i2c_cmd_handle);
    if (result != ESP_OK) {
        ESP_LOGE(TAS5805M_TAG, "ERROR: Could not start I2C: %d", result);
        return result;
    }

    /*
        Dummy loop so that common cleanup code can be after the loop.
    */
    do {
        result = i2c_master_write_byte(i2c_cmd_handle, AMP_WRITE_ADDR, true);
        if (result != ESP_OK) {
            ESP_LOGE(TAS5805M_TAG, "ERROR: Could not write I2C write addr: %d", result);
            break;
        }

        result = i2c_master_write(i2c_cmd_handle, data_to_write, size_write_data, true);
        if (result != ESP_OK) {
            ESP_LOGE(TAS5805M_TAG, "ERROR: Could not write I2C data: %d", result);
            break;
        }

        if (operation == I2C_MASTER_READ) {
            result = i2c_master_start(i2c_cmd_handle);
            if (result != ESP_OK) {
                ESP_LOGE(TAS5805M_TAG, "ERROR: Could not second start I2C: %d", result);
                return result;
            }
            
            result = i2c_master_write_byte(i2c_cmd_handle, AMP_READ_ADDR, true);
            if (result != ESP_OK) {
                ESP_LOGE(TAS5805M_TAG, "ERROR: Could not write I2C addr in prep for read: %d", result);
                break;
            }

            result = i2c_master_read(i2c_cmd_handle, buffer, buffer_size, I2C_MASTER_NACK);
            if (result != ESP_OK) {
                ESP_LOGE(TAS5805M_TAG, "ERROR: Reading I2C data.");
                break;
            }
        }

        result = i2c_master_stop(i2c_cmd_handle);
        if (result != ESP_OK) {
            ESP_LOGE(TAS5805M_TAG, "ERROR: Could not stop I2C: %d", result);
            break;
        }

        result = i2c_master_cmd_begin(PORT, i2c_cmd_handle, I2C_WAIT);
        if (result != ESP_OK) {
            ESP_LOGE(TAS5805M_TAG, "ERROR: Could not issue I2C stream commands: %d", result);
            break;
        }
    } while (false);

    i2c_cmd_link_delete(i2c_cmd_handle);

    return result;
}
