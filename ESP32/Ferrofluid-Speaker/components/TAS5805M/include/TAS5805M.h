#pragma once

#include "driver/i2c.h"

/** 
 * @brief Class for managing the TAS5805M amplifier.
 * @author Adam Puleo
 * @copyright Apache License Version 2
*/
class TAS5805M {
    public:
        // Constructor
        TAS5805M(const uint8_t write_address,
                 const i2c_port_t port,
                 const uint32_t frequency,
                 const gpio_num_t pdn_pin,
                 const gpio_num_t fault_pin,
                 const gpio_num_t scl_pin,
                 const gpio_num_t sda_pin);

        esp_err_t start_pre_i2s();
        esp_err_t start_post_i2s();
        esp_err_t stop();
        bool error();

        // Destructor
        ~TAS5805M();

    private:
        // Configuration parameters for the amp.
        i2c_port_t PORT;
        uint32_t FREQUENCY;
        gpio_num_t PDN;
        gpio_num_t FAULT;
        gpio_num_t SCL_PIN;
        gpio_num_t SDA_PIN;
        uint8_t AMP_WRITE_ADDR;  // I2C write address
        uint8_t AMP_READ_ADDR;  // I2C read address

        enum state_t {
            POWER_OFF,
            DEEP_SLEEP,
            HiZ,
            ON
        };
        state_t state = POWER_OFF;

        esp_err_t send_i2c_commands(const i2c_rw_t operation,
                                    const uint8_t *data_to_write,
                                    const size_t size_write_data,
                                    uint8_t *buffer,
                                    const size_t buffer_size);

        // Constants (registers) for I2C data streams.
        const uint8_t PAGE_REG = 0x00;  // Page register, see section 7.5.2.5
        const uint8_t DEVICE_CTRL_2 = 0x03;  // Section 7.6.1.3
        const uint8_t SIG_CH_CTRL = 0x28;  // Section 7.6.1.5
        const uint8_t SAP_CTRL1 = 0x33;  // Section 7.6.1.9
        const uint8_t FS_MON = 0x37;  // Section 7.6.1.12
        const uint8_t CLKDET_STATUS = 0x39;  // Section 7.6.1.14
        const uint8_t DIG_VOL_CTL = 0x4C;  // Digital volume, section 7.6.1.15
        const uint8_t AGAIN_REG = 0x54;  // Analog gain register, see section 7.3.8.1
        const uint8_t ADR_PIN_CTRL = 0x60;  // Section 7.6.1.24 
        const uint8_t ADR_PIN_CONFIG = 0x61;  // Section 7.6.1.25
        const uint8_t DIE_ID = 0x67;  // Section 7.6.1.27
        const uint8_t POWER_STATE = 0x68;  // Section 7.6.1.28
        const uint8_t CHAN_FAULT = 0x70; // Section 7.6.1.36
        const uint8_t GLOBAL_FAULT1 = 0x71; // Section 7.6.1.37
        const uint8_t GLOBAL_FAULT2 = 0x72; // Section 7.6.1.38
        const uint8_t OT_WARNING = 0x73; // Section 7.6.1.39
        const uint8_t BOOK_REG = 0x7F;  // Book register, see section 7.5.2.5

        // Setup config constants for ESP's I2C communication to the amp.
        const size_t I2C_MASTER_TX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
        const size_t I2C_MASTER_RX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
        const TickType_t I2C_WAIT = 1000 / portTICK_RATE_MS;  // A second should be more than enough time?
};
