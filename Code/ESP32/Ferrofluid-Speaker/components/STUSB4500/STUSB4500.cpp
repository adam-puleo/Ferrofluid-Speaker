/**
 * @file STUSB4500.cpp
 * @author Adam Puleo (adam.puleo@icloud.com)
 * @brief This code is based off of information found in UM2650 User manual - "The STUSB4500 software programming guide".
 * @version 0.1
 * @date 2022-11-25
 * 
 * @copyright Copyright (c) 2022 
 * 
 */

#include "STUSB4500.hpp"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
const char* STUSB4500_TAG = "STUSB4500";
#include "esp_log.h"

#define CHECK_ERROR(error_code, ...) if (error_code != ESP_OK) {ESP_LOGE(STUSB4500_TAG, ##__VA_ARGS__); return error_code;}

// Constructor
STUSB4500::STUSB4500(i2c_wrapper *i2c_interface,
                     const uint8_t write_address) {
    this->i2c_interface = i2c_interface;
    this->WRITE_ADDR = write_address;
}

/**
 * @brief Based off section 1.2.
 * 
 * @return esp_err_t 
 */
esp_err_t STUSB4500::init_chip() {
    esp_err_t result;
    uint8_t write_data[2];
    uint8_t read_data;

    //return read_nvm();

    return exit_test_mode();

    ESP_LOGD(STUSB4500_TAG, "reading regs");
    // Clear all interrupts by reading all 10 registers from address 0x0D to 0x16 according to document.
    // Code on the Internet reads 12 registers starting at ALERT_STATUS_1.
    for (uint8_t reg = ALERT_STATUS_1; reg <= PRT_STATUS; reg++) {
        result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                        I2C_MASTER_READ,
                                                        &reg, sizeof(reg),
                                                        &read_data, sizeof(read_data));
        CHECK_ERROR(result, "Could not clear IRQs.");
    }

    // Unmask interrupts
    STUSB_GEN1S_ALERT_STATUS_MASK_RegTypeDef alert_mask;
    alert_mask.d8 = 0xFF;  
    alert_mask.b.PRT_STATUS_AL_MASK = 0;
    alert_mask.b.PD_TYPEC_STATUS_AL_MASK = 0;
    alert_mask.b.MONITORING_STATUS_AL_MASK = 0;
    alert_mask.b.CC_DETECTION_STATUS_AL_MASK = 0;
    alert_mask.b.HARD_RESET_AL_MASK = 0;
    write_data[0] = ALERT_STATUS_MASK;
    write_data[1] = alert_mask.d8;
    ESP_LOGD(STUSB4500_TAG, "unmask irqs");
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    write_data, sizeof(write_data),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not unmask IRQs.");

    write_data[0] = PORT_STATUS_1;
    ESP_LOGD(STUSB4500_TAG, "read port status");
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_READ,
                                                    write_data, 1,
                                                    &read_data, sizeof(read_data));
    CHECK_ERROR(result, "Could not read port status.");
    ESP_LOGD(STUSB4500_TAG, "port status: 0x%x", read_data);

    write_data[0] = STUSB_GEN1S_RESET_CTRL_REG;
    ESP_LOGD(STUSB4500_TAG, "read reset ctrl reg");
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_READ,
                                                    write_data, 1,
                                                    &read_data, sizeof(read_data));
    CHECK_ERROR(result, "Could not reset CTRL_REG.");
    ESP_LOGD(STUSB4500_TAG, "reset reg: 0x%x", read_data);

    write_data[0] = DPM_PDO_NUMB;
    ESP_LOGD(STUSB4500_TAG, "read num pdo");
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_READ,
                                                    write_data, 1,
                                                    &read_data, sizeof(read_data));
    CHECK_ERROR(result, "Could not read PDO num.");
    ESP_LOGD(STUSB4500_TAG, "num pdo: 0x%x", read_data);

    ESP_LOGD(STUSB4500_TAG, "read device id");
    /*do { // wait for NVM to be reloaded
        result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                        I2C_MASTER_READ,
                                                        &DEVICE_ID, sizeof(DEVICE_ID),
                                                        &read_data, sizeof(read_data));
        if (result != ESP_OK) {
            return result;
        }
    } while (read_data != ST_EVAL_BOARD || read_data != PRODUCT);*/

    return result;
}

/**
 * @brief Reads a sector from NVM.
 * 
 * @param sector_number Which sector to read, 0 .. 4.
 * @param bytes Four byte buffer to place the result in.
 * @return esp_err_t 
 */
esp_err_t STUSB4500::read_sector(const uint8_t sector_number, uint8_t *bytes) {
    esp_err_t result;
    NVM_CTRL_LOW_RegTypeDef nvm_ctrl_low = {.byte = 0x00};
    NVM_CTRL_HIGH_RegTypeDef nvm_ctrl_high = {.byte = 0x00};
    uint8_t buffer[2];

    // Set PWR and RST_N bits 0x96->0xC0
    nvm_ctrl_low.b.RST_N = 1;
    nvm_ctrl_low.b.PWR = 1;
    buffer[0] = NVM_CTRL_LOW;
    buffer[1] = nvm_ctrl_low.byte;
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    buffer, sizeof(buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not reset PWR and RST_N bits on NVM controller.");

    // Set Read Sectors Opcode 0x97->0x00
    nvm_ctrl_high.b.OPCode = READ;
    result = execute_nvm_opcode(nvm_ctrl_high, 2000);
    CHECK_ERROR(result, "Could not set execute READ opcode.");

    // Read the sectors' data.
    return this->i2c_interface->read_multi_regs(this->WRITE_ADDR,
                                                NVM_SECTOR_0 + sector_number * (NVM_SECTOR_1 - NVM_SECTOR_0),
                                                NVM_SECTOR_1 - NVM_SECTOR_0,
                                                bytes);
}

/**
 * @brief Read NVM
 * 
 * @return esp_err_t 
 */
esp_err_t STUSB4500::read_nvm(void) {
    esp_err_t result;
    uint8_t buffer[2];
    NVM_CTRL_LOW_RegTypeDef nvm_ctrl_low;

    //Read Current Parameters
    //-Enter Read Mode
    //-Read Sector[x][-]
    //---------------------------------
    //Enter Read Mode
    buffer[0] = NVM_PASSWD;
    buffer[1] = NVM_CUST_PASSWORD;  /* Set Password 0x95->0x47*/
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    buffer, sizeof(buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not unlock NVM.");

    buffer[0] = NVM_CTRL_LOW;
    buffer[1] = 0; /* NVM internal controller reset 0x96->0x00*/
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    buffer, sizeof(buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not reset NVM controller.");

    nvm_ctrl_low.b.RST_N = 1;
    nvm_ctrl_low.b.PWR = 1;
    buffer[0] = NVM_CTRL_LOW;
    buffer[1] = nvm_ctrl_low.byte;
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    buffer, sizeof(buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not set PWR and RST_N bits on NVM controller.");
    //--- End of CUST_EnterReadMode

    for (uint8_t sector = 0; sector < 5; sector++) {
        result = read_sector(sector, &sectors.bytes[sector * 8]);
        CHECK_ERROR(result, "Could not read sector %d.", sector);
    }
    
    exit_test_mode();

    // NVM settings get loaded into the volatile registers after a hard reset or power cycle.
    // Below we will copy over some of the saved NVM settings to the I2C registers

    result = set_pdo_number(sectors.b.SNK_PDO_NUMB);
    CHECK_ERROR(result, "Could not set PDO number.");

    //PDO1 - fixed at 5V and is unable to change
    result = set_voltage(1, 5000);
    CHECK_ERROR(result, "Could not set voltage for PDO1.");

    result = set_current(1, four_bit_current_to_m_amps(sectors.b.I_SNK_PDO1));
    CHECK_ERROR(result, "Could not set current for PDO 1.");

    //PDO2
    result = set_voltage(2, sectors.b.V_SNK_PDO2 * (1000 / 20));
    CHECK_ERROR(result, "Could not set voltage for PDO2.");

    result = set_current(2, four_bit_current_to_m_amps(sectors.b.I_SNK_PDO2));
    CHECK_ERROR(result, "Could not set current for PDO 2.");

    //PDO3
    result = set_voltage(2, sectors.b.V_SNK_PDO3 * (1000 / 20));
    CHECK_ERROR(result, "Could not set voltage for PDO 3.");

    result = set_current(3, four_bit_current_to_m_amps(sectors.b.I_SNK_PDO3));
    CHECK_ERROR(result, "Could not set current for PDO 3.");

    return result;
}

esp_err_t STUSB4500::write_nvm(const bool default_vals) {
    esp_err_t result;

    if (!default_vals) {
        USB_PD_SNK_PDO_TypeDef pdos[3];
        unsigned int milliamps;
        uint8_t four_bit_amp[3];
        unsigned int millivolts[3];
        
        // Load current values into NVM
        for (uint8_t idx = 0; idx < 3; idx++) {
            result = this->i2c_interface->read_multi_regs(this->WRITE_ADDR,
                                                          DPM_SNK_PDO1_0 + idx * sizeof(USB_PD_SNK_PDO_TypeDef),
                                                          sizeof(USB_PD_SNK_PDO_TypeDef),
                                                          pdos[idx].bytes);
            CHECK_ERROR(result, "Could not read PDOs.");

            milliamps = pdos[idx].b.Operational_Current * 10;  // 10mA resolution
            millivolts[idx] = pdos[idx].b.Voltage * 50;  // 50mV resolution

            if (milliamps > 5000) milliamps = 5000;  // Constrain current value to 5A max

            /*Convert current to 4-bit value
            -current from 0.5-3.0A is set in 0.25A steps
            -current from 3.0-5.0A is set in 0.50A steps
            */
            if (milliamps < 500)        four_bit_amp[idx] = 0;
            else if (milliamps <= 3000) four_bit_amp[idx] = (4 * milliamps / 1000) - 1;
            else                        four_bit_amp[idx] = (2 * milliamps / 1000) + 5;

            // Make sure the minimum voltage is between 5-20V
            if (millivolts[idx] < 5000)       millivolts[idx] = 5000;
            else if (millivolts[idx] > 20000) millivolts[idx] = 20000;
        }

      	// load current for PDO1 (sector 3, byte 2, bits 4:7)
        this->sectors.b.I_SNK_PDO1 = four_bit_amp[0];

        // load current for PDO2 (sector 3, byte 4, bits 0:3)
        this->sectors.b.I_SNK_PDO2 = four_bit_amp[1];

        // load current for PDO3 (sector 3, byte 5, bits 4:7)
        this->sectors.b.I_SNK_PDO3 = four_bit_amp[2];

        // The voltage for PDO1 is 5V and cannot be changed

        // PDO2
        // Load voltage (10-bit)
        // -bit 9:2 - sector 4, byte 1, bits 0:7
        // -bit 0:1 - sector 4, byte 0, bits 6:7	
        this->sectors.b.V_SNK_PDO2 = millivolts[1] / 50;

        // PDO3
        // Load voltage (10-bit)
        // -bit 8:9 - sector 4, byte 3, bits 0:1
        // -bit 0:7 - sector 4, byte 2, bits 0:7
        this->sectors.b.V_SNK_PDO3 = millivolts[2] / 50;

        // Load highest priority PDO number from memory
        uint8_t write_buffer;
        uint8_t read_buffer;
        write_buffer = DPM_PDO_NUMB;
        result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                        I2C_MASTER_READ,
                                                        &write_buffer, sizeof(write_buffer),
                                                        &read_buffer, sizeof(read_buffer));
        CHECK_ERROR(result, "Could not read active PDO.");

        //load PDO number (sector 3, byte 2, bits 2:3) for NVM saving
        this->sectors.b.SNK_PDO_NUMB = read_buffer;

        result = enter_write_mode(true, true, true, true, true);
        CHECK_ERROR(result, "Could not enter write mode.");

        for (uint8_t sector = 0; sector < 5; sector++) {
            write_sector(sector, &sectors.bytes[sector * 8]);
            CHECK_ERROR(result, "Could not write sector: %d.", sector);
        }
    } else {
        uint8_t default_sector[5][8] = 
        {
            {0x00,0x00,0xB0,0xAA,0x00,0x45,0x00,0x00},
            {0x10,0x40,0x9C,0x1C,0xFF,0x01,0x3C,0xDF},
            {0x02,0x40,0x0F,0x00,0x32,0x00,0xFC,0xF1},
            {0x00,0x19,0x56,0xAF,0xF5,0x35,0x5F,0x00},
            {0x00,0x4B,0x90,0x21,0x43,0x00,0x40,0xFB}
        };

        result = enter_write_mode(true, true, true, true, true);
        CHECK_ERROR(result, "Could not enter write mode.");

        for (uint8_t sector = 0; sector < 5; sector++) {
            write_sector(sector, default_sector[sector * 8]);
            CHECK_ERROR(result, "Could not write default_sector: %d.", sector);
        }
    }

    return exit_test_mode();
}

esp_err_t STUSB4500::write_sector(uint8_t sector, uint8_t *data) {
    esp_err_t result;
    NVM_CTRL_LOW_RegTypeDef nvm_ctrl_low {.byte = 0x00};
    NVM_CTRL_HIGH_RegTypeDef nvm_ctrl_high;
    uint8_t buffer[2];
  
    // Write the 64-bit data to be written in the sector
    result = this->i2c_interface->write_multi_regs(this->WRITE_ADDR,
                                                   NVM_RW_BUFFER_0,
                                                   8,
                                                   data);
    CHECK_ERROR(result, "Could not write sector data to RW buffer.");

    nvm_ctrl_low.b.RST_N = 1;
    nvm_ctrl_low.b.PWR = 1;
    buffer[0] = NVM_CTRL_LOW;
    buffer[1] = nvm_ctrl_low.byte;
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    buffer, sizeof(buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not power up NVM controller.");

    // NVM Program Load Register to write with the 64-bit data to be written in sector.
    nvm_ctrl_high.b.OPCode = WRITE_PLR;
    result = execute_nvm_opcode(nvm_ctrl_high, 500);
    CHECK_ERROR(result, "Could not execute WRITE_PLR opcode.");

    // NVM "Word Program" operation to write the Program Load Register in the sector to be written.
    nvm_ctrl_high.b.OPCode = PROG_SECTOR;
    buffer[0] = NVM_CTRL_HIGH;
    buffer[1] = nvm_ctrl_high.byte;
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    buffer, sizeof(buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not set PROG_SECTOR opcode.");

    nvm_ctrl_low.b.REQ = 1;
    // RST_N and PWR are already set.
    nvm_ctrl_low.b.Sector = sector;
    buffer[0] = NVM_CTRL_LOW;
    buffer[1] = nvm_ctrl_low.byte;
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    buffer, sizeof(buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not load PROG_SECTOR opcode.");

    do {
        vTaskDelay(8000 / portTICK_PERIOD_MS);  // Wait 8 seconds for NVM command to execute.
        result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                        I2C_MASTER_READ,
                                                        &buffer[0], sizeof(buffer[0]),
                                                        &nvm_ctrl_low.byte, sizeof(nvm_ctrl_low.byte));
        CHECK_ERROR(result, "Could not read NVM_CTRL_LOW for opcode execution.");
    } while (nvm_ctrl_low.b.REQ);  // Wait for NVM to execute command.
  
  return result;
}

esp_err_t STUSB4500::execute_nvm_opcode(const NVM_CTRL_HIGH_RegTypeDef nvm_ctrl_high,
                                        const unsigned int timeout_ms) {
    esp_err_t result;
    uint8_t write_buffer[2];
    NVM_CTRL_LOW_RegTypeDef nvm_ctrl_low {.byte = 0x00};
    
    write_buffer[0] = NVM_CTRL_HIGH;
    write_buffer[1] = nvm_ctrl_high.byte;
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    write_buffer, sizeof(write_buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not write NVM opcode.");

    // Tell NVM to load and execute the opcode.
    nvm_ctrl_low.b.REQ = 1;
    nvm_ctrl_low.b.RST_N = 1;
    nvm_ctrl_low.b.PWR = 1;
    write_buffer[0] = NVM_CTRL_LOW;
    write_buffer[1] = nvm_ctrl_low.byte;
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    write_buffer, sizeof(write_buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not load NVM opcode.");

    do {
        vTaskDelay(timeout_ms / portTICK_PERIOD_MS);
        result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                        I2C_MASTER_READ,
                                                        &write_buffer[0], sizeof(write_buffer[0]),
                                                        &nvm_ctrl_low.byte, sizeof(nvm_ctrl_low.byte));
        CHECK_ERROR(result, "Could not read NVM_CTRL_LOW for opcode execution.");
    } while (nvm_ctrl_low.b.REQ);  // Wait for NVM to execute command.

    return result;
}

/**
 * @brief Erase NVM sectors.
 * 
 * @param sec0 Erase sector 0?
 * @param sec1 Erase sector 1?
 * @param sec2 Erase sector 2?
 * @param sec3 Erase sector 3?
 * @param sec4 Erase sector 4?
 * @return esp_err_t 
 */
esp_err_t STUSB4500::enter_write_mode(const bool sec0,
                                      const bool sec1,
                                      const bool sec2,
                                      const bool sec3,
                                      const bool sec4) {
    esp_err_t result;
    NVM_CTRL_LOW_RegTypeDef nvm_ctrl_low;
    NVM_CTRL_HIGH_RegTypeDef nvm_ctrl_high;
    uint8_t write_buffer[2];

    write_buffer[0] = NVM_PASSWD;
    write_buffer[1] = NVM_CUST_PASSWORD;  // Set password
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    write_buffer, sizeof(write_buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not enter customer mode.");

    write_buffer[0] = NVM_RW_BUFFER_0;
    write_buffer[1] = 0;  // This register must be NULL for Partial Erase feature.
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    write_buffer, sizeof(write_buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not erase RW_BUFFER.");

    // NVM Power-up Sequence
    // After STUSB start-up sequence, the NVM is powered off.
    write_buffer[0] = NVM_CTRL_LOW;
    write_buffer[1] = 0;  // NVM internal controller reset.
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    write_buffer, sizeof(write_buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not reset NVM controller.");
    nvm_ctrl_low.b.RST_N = 1;
    nvm_ctrl_low.b.PWR = 1;
    write_buffer[1] = nvm_ctrl_low.byte;
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    write_buffer, sizeof(write_buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not power-up NVM controller.");
    // End NVM Power-up sequence

    // Load 0xF1 to erase all sectors of NVM and Write SER Opcode
    nvm_ctrl_high.b.OPCode = WRITE_SER;
    if (sec0) nvm_ctrl_high.b.SER_0 = 1;
    if (sec1) nvm_ctrl_high.b.SER_1 = 1;
    if (sec2) nvm_ctrl_high.b.SER_2 = 1;
    if (sec3) nvm_ctrl_high.b.SER_3 = 1;
    if (sec4) nvm_ctrl_high.b.SER_4 = 1;
    result = execute_nvm_opcode(nvm_ctrl_high, 500);
    CHECK_ERROR(result, "Could not execute WRITE_SER opcode.");

    nvm_ctrl_high.byte = 0x00;  // Clear SER_X bits.
    nvm_ctrl_high.b.OPCode = SOFT_PROG_SECTOR;  // Set Soft Prog Opcode
    result = execute_nvm_opcode(nvm_ctrl_high, 2000);
    CHECK_ERROR(result, "Could not execute SOFT_PROG_SECTOR opcode.");

    nvm_ctrl_high.b.OPCode = ERASE_SECTOR;  // Set Erase Sectors Opcode
    result = execute_nvm_opcode(nvm_ctrl_high, 500);
    CHECK_ERROR(result, "Could not execute ERASE_SECTOR opcode.");
    
    return result;
}

esp_err_t STUSB4500::exit_test_mode(void) {
    esp_err_t result;
    NVM_CTRL_LOW_RegTypeDef nvm_ctrl_low = {.byte = 0x00};
    uint8_t buffer[2];
  
    nvm_ctrl_low.b.RST_N = 1;
    buffer[0] = NVM_CTRL_LOW;
    buffer[1] = nvm_ctrl_low.byte;
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    buffer, sizeof(buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not reset NVM.");

    // Clear password
    buffer[0] = NVM_PASSWD;
    buffer[1] = 0x00;
    return this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                  I2C_MASTER_WRITE,
                                                  buffer, sizeof(buffer),
                                                  NULL, 0);
}

/**
 * @brief Based off section 1.8.
 * 
 * @param m_volt 
 * @param m_amp 
 * @param mismatch 
 * @return esp_err_t 
 */
esp_err_t STUSB4500::read_power(unsigned int *m_volt, unsigned int *m_amp, bool *mismatch) {
    esp_err_t result;
    STUSB_GEN1S_RDO_REG_STATUS_RegTypeDef rdo_reg;
    uint8_t buffer;

    *m_volt = 0;
    *m_amp = 0;
    *mismatch = false;

    for (uint8_t reg = RDO_REG_STATUS_0; reg <= RDO_REG_STATUS_3; reg++) {
        result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                        I2C_MASTER_READ,
                                                        &reg, sizeof(reg),
                                                        &rdo_reg.bytes[reg - RDO_REG_STATUS_0], 1);
        CHECK_ERROR(result, "Could not read RDO reg 0x%x", reg);
    }

    ESP_LOGD(STUSB4500_TAG, "op current: %d, mismatch: %d", rdo_reg.b.OperatingCurrent, rdo_reg.b.CapaMismatch);
    *m_amp = rdo_reg.b.OperatingCurrent * 10;
    *mismatch = (rdo_reg.b.CapaMismatch == 1);

    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_READ,
                                                    &STUSB_GEN1S_MONITORING_CTRL_1, sizeof(STUSB_GEN1S_MONITORING_CTRL_1),
                                                    &buffer, sizeof(buffer));
    CHECK_ERROR(result, "Could not read mon ctrl 1 reg.");

    *m_volt = buffer * 100;

    ESP_LOGD(STUSB4500_TAG, "m_volt: %d, m_amp: %d, mismatch: %d", *m_volt, *m_amp, *mismatch);
    return result;
}

/**
 * @brief Based off section 1.4
 * 
 * @param m_volt 
 * @param m_amp 
 * @return esp_err_t 
 */
esp_err_t STUSB4500::set_power(unsigned int m_volt, unsigned int m_amp) {
    esp_err_t result;
    uint8_t buffer[2];
    NVM_CTRL_LOW_RegTypeDef nvm_ctrl_low = {.byte = 0x00};
    USB_PD_SNK_PDO_TypeDef pdo_reg = {.bytes = {0x00, 0x00, 0x00, 0x00}};
    
    ESP_LOGD(STUSB4500_TAG, "setting power, m_volt: %d, m_amp: %d", m_volt, m_amp);
    if (5000 <= m_volt && m_volt <= 20000 && 500 <= m_amp && m_amp <= 5000) {
        buffer[0] = NVM_PASSWD;
        buffer[1] = NVM_CUST_PASSWORD;  /* Set Password 0x95->0x47*/
        result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                        I2C_MASTER_WRITE,
                                                        buffer, sizeof(buffer),
                                                        NULL, 0);
        CHECK_ERROR(result, "Could not unlock NVM.");

        buffer[0] = NVM_CTRL_LOW;
        buffer[1] = 0; /* NVM internal controller reset 0x96->0x00*/
        result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                        I2C_MASTER_WRITE,
                                                        buffer, sizeof(buffer),
                                                        NULL, 0);
        CHECK_ERROR(result, "Could not reset NVM controller.");

        nvm_ctrl_low.b.RST_N = 1;
        nvm_ctrl_low.b.PWR = 1;
        buffer[0] = NVM_CTRL_LOW;
        buffer[1] = nvm_ctrl_low.byte;
        result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                        I2C_MASTER_WRITE,
                                                        buffer, sizeof(buffer),
                                                        NULL, 0);
        CHECK_ERROR(result, "Could not set PWR and RST_N bits on NVM controller.");

        result = this->i2c_interface->read_multi_regs(this->WRITE_ADDR,
                                                      DPM_SNK_PDO2_0,
                                                      sizeof(USB_PD_SNK_PDO_TypeDef),
                                                      pdo_reg.bytes);
        CHECK_ERROR(result, "Could not read PDO2.");

        pdo_reg.b.Operational_Current = m_amp / 10;
        pdo_reg.b.Voltage = m_volt / 50;

        result = this->i2c_interface->write_multi_regs(this->WRITE_ADDR,
                                                       DPM_SNK_PDO2_0,
                                                       sizeof(USB_PD_SNK_PDO_TypeDef),
                                                       pdo_reg.bytes);
        CHECK_ERROR(result, "Could not write PDO2.");

        result = this->i2c_interface->read_multi_regs(this->WRITE_ADDR,
                                                      DPM_SNK_PDO2_0,
                                                      sizeof(USB_PD_SNK_PDO_TypeDef),
                                                      pdo_reg.bytes);
        CHECK_ERROR(result, "Could not read back PDO2.");
        ESP_LOGD(STUSB4500_TAG, "pdo2 read back: amp: 0x%x volt: 0x%x", pdo_reg.b.Operational_Current, pdo_reg.b.Voltage);
        ESP_LOGD(STUSB4500_TAG, "pdo2 read back: m_amp: %d m_volt: %d", pdo_reg.b.Operational_Current * 10, pdo_reg.b.Voltage * 50);

        result = exit_test_mode();
        CHECK_ERROR(result, "Could not exit test mode.");

        result = renegotiate();

    } else {
        result = ESP_ERR_INVALID_ARG;
    }

    return result;
}

esp_err_t STUSB4500::renegotiate() {
    esp_err_t result;

    uint8_t soft_rst_buffer[] = {TX_HEADER_LOW, SOFT_RESET};
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    soft_rst_buffer, sizeof(soft_rst_buffer),
                                                    NULL, 0);
    CHECK_ERROR(result, "Could not issue soft reset.");
    /*soft_rst_buffer[0] = TX_HEADER_HIGH;
    soft_rst_buffer[1] = 0x00;
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    soft_rst_buffer, sizeof(soft_rst_buffer),
                                                    NULL, 0);
    if (result != ESP_OK) {
        return result;
    }*/

    uint8_t cmd_buffer[] = {PD_COMMAND_CTRL, SEND_COMMAND};
    result = this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                    I2C_MASTER_WRITE,
                                                    cmd_buffer, sizeof(cmd_buffer),
                                                    NULL, 0);
    
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    return result;
}

esp_err_t STUSB4500::set_pdo_number(uint8_t value) {
    uint8_t buffer[2];

    buffer[0] = DPM_PDO_NUMB;
    if (value > 3) {
        buffer[1] = 3;
    } else {
        buffer[1] = value;
    }

    return this->i2c_interface->send_i2c_commands(this->WRITE_ADDR,
                                                  I2C_MASTER_WRITE,
                                                  buffer, sizeof(buffer),
                                                  NULL, 0);
}

esp_err_t STUSB4500::set_voltage(uint8_t pdo_numb, unsigned int m_volts) {
    esp_err_t result;
    USB_PD_SNK_PDO_TypeDef pdo;

    if (pdo_numb < 1) pdo_numb = 1;
    else if (pdo_numb > 3) pdo_numb = 3;

    //Constrain voltage variable to 5-20V
    if (m_volts < 5000) m_volts = 5000;
    else if (m_volts > 20000) m_volts = 20000;

    // Load voltage to volatile PDO memory (PDO1 needs to remain at 5V)
    if (pdo_numb == 1) m_volts = 5000;

    result = this->i2c_interface->read_multi_regs(this->WRITE_ADDR,
                                                  DPM_SNK_PDO1_0 + (pdo_numb - 1) * sizeof(USB_PD_SNK_PDO_TypeDef),
                                                  sizeof(USB_PD_SNK_PDO_TypeDef),
                                                  pdo.bytes);
    CHECK_ERROR(result, "Could not read PDO.");

    pdo.b.Voltage = m_volts / 50;

    return this->i2c_interface->write_multi_regs(this->WRITE_ADDR,
                                                 DPM_SNK_PDO1_0 + (pdo_numb - 1) * sizeof(USB_PD_SNK_PDO_TypeDef),
                                                 sizeof(USB_PD_SNK_PDO_TypeDef),
                                                 pdo.bytes);
}

esp_err_t STUSB4500::set_current(uint8_t pdo_numb, unsigned int m_amps) {
    esp_err_t result;
    USB_PD_SNK_PDO_TypeDef pdo;

    result = this->i2c_interface->read_multi_regs(this->WRITE_ADDR,
                                                  DPM_SNK_PDO1_0 + (pdo_numb - 1) * sizeof(USB_PD_SNK_PDO_TypeDef),
                                                  sizeof(USB_PD_SNK_PDO_TypeDef),
                                                  pdo.bytes);
    CHECK_ERROR(result, "Could not read PDO.");

    pdo.b.Operational_Current = m_amps / 10;

    return this->i2c_interface->write_multi_regs(this->WRITE_ADDR,
                                                 DPM_SNK_PDO1_0 + (pdo_numb - 1) * sizeof(USB_PD_SNK_PDO_TypeDef),
                                                 sizeof(USB_PD_SNK_PDO_TypeDef),
                                                 pdo.bytes);
}

/**
 * @brief 
 *     From 4-bit value to milliamps.
 *     Current from 0.5-3.0A is set in 0.25A steps.
 *     Current from 3.0-5.0A is set in 0.50A steps.
 * 
 * @param current 
 * @return unsigned int 
 */
unsigned int STUSB4500::four_bit_current_to_m_amps(uint8_t current) {
    if (current == 0) {
        return 0;
    } else if (current < 11) {
        return current * 250 + 250;
    } else {
        return current * 500 - 2500;
    }
}

// Destructor
STUSB4500::~STUSB4500() {
    exit_test_mode();
}
