/**
 * @file STUSB4500.hpp
 * @author Adam Puleo (adam.puleo@icloud.com)
 * @brief 
 * @version 0.1
 * @date 2022-11-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "i2c-wrapper.hpp"

/** 
 * @brief Class for managing the STUSB4500 USB controller.
 * @author Adam Puleo
 * @copyright Apache License Version 2
*/
class STUSB4500 {
    public:
        /**
         * @brief Construct a new STUSB4500 object. The I2C subsystem has already been initialized.
         * 
         * @param i2c_lib i2c_wrapper object.
         * @param write_address Full 8-bit address. Constructor sets the LSB to one to form the read address.
         */
        STUSB4500(i2c_wrapper *i2c_interface);

        /**
         * @brief Retrieve the negotiated power contract. If there is no contract zero is returned for both m_volt and m_amp.
         * 
         * @param m_volt Millivolts
         * @param m_amp Milliamps
         * @param mismatch Capability Mismatch
         * @return esp_err_t 
         */
        esp_err_t read_power(unsigned int *m_volt, unsigned int *m_amp, bool *mismatch);
        
        /**
         * @brief Set the PDO 2 register
         * 
         * @param m_vol Millivolts
         * @param m_amp Milliamps
         * @return esp_err_t 
         */
        esp_err_t set_power(unsigned int m_volt, unsigned int m_amp);

        // Destructor
        ~STUSB4500();

    private:
        i2c_wrapper *i2c_interface;

        // Constants
        const uint8_t ST_EVAL_BOARD = 0x21;
        const uint8_t PRODUCT = 0x25;
        const uint8_t SOFT_RESET = 0x0D;
        const uint8_t SEND_COMMAND = 0x26;
        const uint8_t NVM_CUST_PASSWORD = 0x47;  // For NVM_PASSWD reg
        // OP Codes
        const uint8_t READ = 0;
        const uint8_t WRITE_PLR = 1;
        const uint8_t WRITE_SER = 2;
        const uint8_t READ_PLR = 3;
        const uint8_t READ_SER = 4;
        const uint8_t ERASE_SECTOR = 5;
        const uint8_t PROG_SECTOR = 6;
        const uint8_t SOFT_PROG_SECTOR = 7;

        
        // Registers for I2C data streams.
        const uint8_t ALERT_STATUS_1 = 0x0B;
        const uint8_t ALERT_STATUS_MASK = 0x0C;
        union STUSB_GEN1S_ALERT_STATUS_MASK_RegTypeDef {
            uint8_t d8;
            struct {
                uint8_t PHY_STATUS_AL_MASK          : 1; 
                uint8_t PRT_STATUS_AL_MASK          : 1; 
                uint8_t _Reserved_2                 : 1; 
                uint8_t PD_TYPEC_STATUS_AL_MASK     : 1;    
                uint8_t HW_FAULT_STATUS_AL_MASK     : 1;    
                uint8_t MONITORING_STATUS_AL_MASK   : 1;    
                uint8_t CC_DETECTION_STATUS_AL_MASK : 1;    
                uint8_t HARD_RESET_AL_MASK          : 1;
            } b;
        };
        const uint8_t PORT_STATUS_0 = 0x0D;
        const uint8_t PORT_STATUS_1 = 0x0E;
        const uint8_t PRT_STATUS = 0x16;
        const uint8_t PD_COMMAND_CTRL = 0x1A;
        const uint8_t STUSB_GEN1S_MONITORING_CTRL_1 = 0x21;
        const uint8_t STUSB_GEN1S_RESET_CTRL_REG = 0x23;
        const uint8_t PE_FSM = 0x29;
        const uint8_t DEVICE_ID = 0x2F;
        const uint8_t TX_HEADER_LOW = 0x51;
        const uint8_t TX_HEADER_HIGH = 0x52;
        const uint8_t NVM_RW_BUFFER_0 = 0x53;
        const uint8_t NVM_RW_BUFFER_1 = 0x54;
        const uint8_t NVM_RW_BUFFER_2 = 0x55;
        const uint8_t NVM_RW_BUFFER_3 = 0x56;
        const uint8_t NVM_RW_BUFFER_4 = 0x57;
        const uint8_t NVM_RW_BUFFER_5 = 0x58;
        const uint8_t NVM_RW_BUFFER_6 = 0x59;
        const uint8_t NVM_RW_BUFFER_7 = 0x5A;
        const uint8_t DPM_PDO_NUMB = 0x70;

        const uint8_t DPM_SNK_PDO1_0 = 0x85;
        const uint8_t DPM_SNK_PDO1_1 = 0x86;
        const uint8_t DPM_SNK_PDO1_2 = 0x87;
        const uint8_t DPM_SNK_PDO1_3 = 0x88;
        const uint8_t DPM_SNK_PDO2_0 = 0x89;
        const uint8_t DPM_SNK_PDO2_1 = 0x8A;
        const uint8_t DPM_SNK_PDO2_2 = 0x8B;
        const uint8_t DPM_SNK_PDO2_3 = 0x8C;
        const uint8_t DPM_SNK_PDO3_0 = 0x8D;
        const uint8_t DPM_SNK_PDO3_1 = 0x8E;
        const uint8_t DPM_SNK_PDO3_2 = 0x8F;
        const uint8_t DPM_SNK_PDO3_3 = 0x90;
        union USB_PD_SNK_PDO_TypeDef {
            uint8_t bytes[4];
            struct {
                uint32_t Operational_Current       : 10; // Bits 9..0
                uint32_t Voltage                   : 10;
                uint8_t _Reserved_22_20            : 3;
                uint8_t Fast_Role_Req_cur          : 2;  /* must be set to 0 in 2.0*/
                uint8_t Dual_Role_Data             : 1;
                uint8_t USB_Communications_Capable : 1;
                uint8_t Unconstrained_Power        : 1;
                uint8_t Higher_Capability          : 1;
                uint8_t Dual_Role_Power            : 1;
                uint8_t Fixed_Supply               : 2; // Bits 30..31
            } b;
        };

        const uint8_t RDO_REG_STATUS_0 = 0x91;
        const uint8_t RDO_REG_STATUS_1 = 0x92;
        const uint8_t RDO_REG_STATUS_2 = 0x93;
        const uint8_t RDO_REG_STATUS_3 = 0x94;
        union STUSB_GEN1S_RDO_REG_STATUS_RegTypeDef {
            uint8_t bytes[4];
            struct {
                uint32_t MaxCurrent       : 10; // Bits 9..0
                uint32_t OperatingCurrent : 10;
                uint8_t _Reserved_22_20   : 3;
                uint8_t UnchunkedMess_sup : 1;
                uint8_t UsbSuspend        : 1;
                uint8_t UsbComCap         : 1;
                uint8_t CapaMismatch      : 1;
                uint8_t GiveBack          : 1;
                uint8_t Object_Pos        : 3; // Bits 30..28 (3-bit)
                uint8_t _Reserved_31	  : 1; // Bit 31
            } b;
        };

        // https://github.com/timkruse/stusb4500/blob/master/docs/STUSB45_RegisterDescription_inofficial.pdf
        const uint8_t NVM_PASSWD = 0x95;

        const uint8_t NVM_CTRL_LOW = 0x96;
        union NVM_CTRL_LOW_RegTypeDef {
            uint8_t byte;
            struct {
                uint8_t Sector      : 3; // Bits 2..0
                uint8_t _Reserved_3 : 1;
                uint8_t REQ         : 1;
                uint8_t _Reserved_5 : 1;
                uint8_t RST_N       : 1;
                uint8_t PWR         : 1; // Bit 7
            } b;
        };

        const uint8_t NVM_CTRL_HIGH = 0x97;
        union NVM_CTRL_HIGH_RegTypeDef {
            uint8_t byte;
            struct {
                uint8_t OPCode      : 3; // Bits 2..0
                uint8_t SER_0       : 1; // Erase sector 0
                uint8_t SER_1       : 1; // Erase sector 1
                uint8_t SER_2       : 1; // Erase sector 2
                uint8_t SER_3       : 1; // Erase sector 3
                uint8_t SER_4       : 1; // Erase sector 4 - Bit 7
            } b;
        };

        const uint8_t NVM_SECTOR_0 = 0xC0;
        const uint8_t NVM_SECTOR_1 = 0xC8;
        const uint8_t NVM_SECTOR_2 = 0xD0;
        const uint8_t NVM_SECTOR_3 = 0xD8;
        const uint8_t NVM_SECTOR_4 = 0xE0;
        union NVM_Sectors {
            uint8_t bytes[5 * 8];  // Five sectors with eight bytes each.
            struct {
                // Sector 0
                uint8_t _Reserved_0[8];

                // Sector 1
                uint8_t _Reserved_3_0      : 4; // Bits 3..0
                uint8_t GPIO_CFG           : 4;
                uint8_t _Reserved_8        : 5;
                uint8_t VBUS_DISCH_DISABLE : 1;
                uint8_t _Reserved_15_14    : 2; // Bits 15..14
                uint16_t _Reserved_31_16   : 16;
                uint32_t _Reserved_63_32   : 32;

                // Sector 2
                uint8_t _Reserved_2[8];

                // Sector 3
                uint16_t _Reserved_15_0  : 16; // Bytes 1..0
                uint8_t USB_COMM_CAPABLE : 1;
                uint8_t SNK_PDO_NUMB     : 2;
                uint8_t SNK_UNCONS_POWER : 1;
                uint8_t I_SNK_PDO1       : 4;
                uint8_t _Reserved_24     : 4;
                uint8_t SHIFT_VBUS_HL1   : 4;
                uint8_t I_SNK_PDO2       : 4;
                uint8_t SHIFT_VBUS_LL2   : 4;
                uint8_t SHIFT_VBUS_HL2   : 4;
                uint8_t I_SNK_PDO3       : 4;
                uint8_t SHIFT_VBUS_LL3   : 4;
                uint8_t SHIFT_VBUS_HL3   : 4;
                uint8_t _Reserved_63_56  : 8;

                // Sector 4
                uint8_t _Reserved_5_0       : 6; // Bits 5..0
                uint16_t V_SNK_PDO2         : 10;
                uint16_t V_SNK_PDO3         : 10;
                uint16_t I_SNK_PDO_FLEX     : 10;
                uint8_t _Reserved_36        : 1;
                uint8_t POWER_OK_CFG        : 2;
                uint8_t _Reserved_39        : 1;
                uint8_t _Reserved_47_40     : 8;
                uint8_t _Reserved_50_48     : 3;
                uint8_t POWER_ONLY_ABOVE_5V : 1;
                uint8_t REQ_SRC_CURRENT     : 1;
                uint8_t _Reserved_55_53     : 3;
                uint8_t _Reserved_59_56     : 4;
                uint8_t Alarm_IRQ_Mask      : 3;
                uint8_t _Reserved_63        : 1;
            } b;
        };
        NVM_Sectors sectors;

        esp_err_t renegotiate(void);
        esp_err_t exit_test_mode(void);

        /* Unused functions */
        esp_err_t init_chip();
        esp_err_t read_sector(uint8_t sector_number, uint8_t *bytes);
        esp_err_t write_sector(uint8_t sector, uint8_t *data);
        esp_err_t read_nvm(void);
        esp_err_t write_nvm(const bool default_vals);
        esp_err_t set_pdo_number(uint8_t value);
        esp_err_t set_voltage(uint8_t pdo_numb, unsigned int m_volts);
        esp_err_t set_current(uint8_t pdo_numb, unsigned int m_amps);
        unsigned int four_bit_current_to_m_amps(uint8_t current);
        esp_err_t execute_nvm_opcode(const NVM_CTRL_HIGH_RegTypeDef nvm_ctrl_high,
                                     const unsigned int timeout_ms);
        esp_err_t enter_write_mode(const bool sec0,
                                   const bool sec1,
                                   const bool sec2,
                                   const bool sec3,
                                   const bool sec4);
};
