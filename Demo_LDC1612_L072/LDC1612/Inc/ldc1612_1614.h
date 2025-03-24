/*
 * LDC1612_1614.h
 *
 *  Created on: Mar 10, 2025
 *      Author: Alex Bakshaev (aka Дрочеслав in Blue Archive)
 */
#ifndef INC_LDC1612_1614_H_
#define INC_LDC1612_1614_H_

/*-------------------- S E L E C T --------------------*/
// USE_LDC1612 or USE_LDC1614
#define USE_LDC1612

/*-------------------- I N C L U D E S --------------------*/
#include "stdint.h"
#include "main.h"

/*-------------------- D E F I N E S --------------------*/
//Interrupt
//#define ENABLE_INTERRUPT

// Sleep
#define LDC1612_1614_EN_SLEEP_MODE (1)

// HAL or CMSIS
#define LDC1612_1614_USE_HAL
#define LDC1612_1614_USE_CMSIS

// I2C Address
#define LDC1612_I2C_ADDRESS_LOW  (0x2A)  // ADDR = L, I2C address = 0x2A
#define LDC1612_I2C_ADDRESS_HIGH (0x2B)  // ADDR = H, I2C address = 0x2B
// Address for .c file
#define LDC1612_I2C_ADDRESS  LDC1612_I2C_ADDRESS_LOW  // ADDR = L or ADDR = H

// GPIO pin value for shutdown pin
#define LDC1612_1614_SHUTDOWN (1)
#define LDC1612_1614_ACTIVE   (0)

#define LDC1612_1614_MANUFACTURER_ID (0x5449) // Manufacturer ID in register 0x7E
#define LDC1612_1614_DEVICE_ID 		 (0x3055) // Device ID in register 0x7F (same for LDC1612 and LDC1614)

// GPIOs
// Shutdown
#define LDC1612_1614_SD_GPIO_PORT LDC1612_SD_GPIO_Port
#define LDC1612_1614_SD_PIN 	  LDC1612_SD_Pin
// Interrupt
#define LDC1612_1614_INT_GPIO_PORT LD1612_INT_GPIO_Port
#define LDC1612_1614_INT_PIN 	   LD1612_INT_Pin

/*-------------------- E N U M S --------------------*/
// Error channel number in register 0x18, STATUS
typedef enum {
	LDC1612_1614_ERR_CHAN_0 = 0b00,
	LDC1612_1614_ERR_CHAN_1 = 0b01,
#ifdef USE_LDC1614
	LDC1612_1614_ERR_CHAN_2 = 0b10,
	LDC1612_1614_ERR_CHAN_3 = 0b11
#endif
}LDC1612_1614_Status_Error_channel_TypeDef;

// Active channel number in register 0x1A, CONFIG
typedef enum {
	LDC1612_1614_ACTIVE_CHAN_0 = 0b00,
	LDC1612_1614_ACTIVE_CHAN_1 = 0b01,
#ifdef USE_LDC1614
	LDC1612_1614_ACTIVE_CHAN_2 = 0b10,
	LDC1612_1614_ACTIVE_CHAN_3 = 0b11
#endif
}LDC1612_1614_Active_Channel_TypeDef;

/*-------------------- S T R U C T U R E S --------------------*/
typedef struct {
	// Registers
    uint16_t data_0_msb;        // Channel 0 MSB Conversion Result and Error Status
    uint16_t data_0_lsb;        // Channel 0 LSB Conversion Result
    uint16_t data_1_msb;        // Channel 1 MSB Conversion Result and Error Status
    uint16_t data_1_lsb;        // Channel 1 LSB Conversion Result
#ifdef USE_LDC1614
    uint16_t data_2_msb;        // Channel 2 MSB Conversion Result and Error Status
    uint16_t data_2_lsb;        // Channel 2 LSB Conversion Result
    uint16_t data_3_msb;        // Channel 3 MSB Conversion Result and Error Status
    uint16_t data_4_lsb;        // Channel 3 LSB Conversion Result
#endif
    uint16_t rcount_0;          // Reference Count setting for Channel 0
    uint16_t rcount_1;          // Reference Count setting for Channel 1
#ifdef USE_LDC1614
    uint16_t rcount_2;          // Reference Count setting for Channel 2
    uint16_t rcount_3;          // Reference Count setting for Channel 3
#endif
    uint16_t offset_0;          // Offset value for Channel 0
    uint16_t offset_1;          // Offset value for Channel 1
#ifdef USE_LDC1614
    uint16_t offset_2;          // Offset value for Channel 2
    uint16_t offset_3;          // Offset value for Channel 3
#endif
    uint16_t settlecount_0;     // Channel 0 Settling Reference Count
    uint16_t settlecount_1;     // Channel 1 Settling Reference Count
#ifdef USE_LDC1614
    uint16_t settlecount_2;     // Channel 2 Settling Reference Count
    uint16_t settlecount_3;     // Channel 3 Settling Reference Count
#endif
    uint16_t clock_dividers_0;  // Reference and Sensor Divider settings for Channel 0
    uint16_t clock_dividers_1;  // Reference and Sensor Divider settings for Channel 1
#ifdef USE_LDC1614
    uint16_t clock_dividers_2;  // Reference and Sensor Divider settings for Channel 2
    uint16_t clock_dividers_3;  // Reference and Sensor Divider settings for Channel 3
#endif
    uint16_t status;            // Device Status Report
    uint16_t error_config;      // Error Reporting Configuration
    uint16_t config;            // Conversion Configuration
    uint16_t mux_config;        // Channel Multiplexing Configuration
    uint16_t reset_dev;         // Reset Device
    uint16_t drive_current_0;   // Channel 0 sensor current drive configuration
    uint16_t drive_current_1;   // Channel 1 sensor current drive configuration
#ifdef USE_LDC1614
    uint16_t drive_current_2;   // Channel 2 sensor current drive configuration
    uint16_t drive_current_3;   // Channel 3 sensor current drive configuration
#endif
    uint16_t manufacturer_id;   // Manufacturer ID
    uint16_t device_id;         // Device ID

    // Combined data
    uint32_t data_channel_0;	// Combined 28 bits of data for channel 0 (without ERR_URx, ERR_ORx,ERR_WDx, ERR_AEx)
    uint32_t data_channel_1;	// Combined 28 bits of data for channel 1
#ifdef USE_LDC1614
    uint32_t data_channel_2;	// Combined 28 bits of data for channel 2
	uint32_t data_channel_3;	// Combined 28 bits of data for channel 3
#endif
	uint16_t error_channel_0;	// Channel 0 value of ERR_URx, ERR_ORx,ERR_WDx, ERR_AEx bits
	uint16_t error_channel_1;	// Channel 1 value of ERR_URx, ERR_ORx,ERR_WDx, ERR_AEx bits
#ifdef USE_LDC1614
	uint16_t error_channel_2;	// Channel 2 value of ERR_URx, ERR_ORx,ERR_WDx, ERR_AEx bits
	uint16_t error_channel_3;	// Channel 3 value of ERR_URx, ERR_ORx,ERR_WDx, ERR_AEx bits
#endif
	uint8_t error_channel;		// Error channel number
	uint8_t active_channel;		// Active channel number
}LDC1612_1614_All_Data_TypeDef;

/*-------------------- R E G I S T E R S --------------------*/
// List of registers
#define LDC1612_1614_REG_DATA_0_MSB_ADDRESS           (0x00)  // Channel 0 MSB Conversion Result and Error Status
#define LDC1612_1614_REG_DATA_0_LSB_ADDRESS           (0x01)  // Channel 0 LSB Conversion Result. Must be read after Register address 0x00.
#define LDC1612_1614_REG_DATA_1_MSB_ADDRESS           (0x02)  // Channel 1 MSB Conversion Result and Error Status
#define LDC1612_1614_REG_DATA_1_LSB_ADDRESS           (0x03)  // Channel 1 LSB Conversion Result. Must be read after Register address 0x02.
#ifdef USE_LDC1614
#define LDC1612_1614_REG_DATA_2_MSB_ADDRESS           (0x04)  // (LDC1614 ONLY) Channel 2 MSB Conversion Result and Error Status
#define LDC1612_1614_REG_DATA_2_LSB_ADDRESS           (0x06)  // (LDC1614 ONLY) Channel 2 LSB Conversion Result. Must be read after Register address 0x04.
#define LDC1612_1614_REG_DATA_3_MSB_ADDRESS           (0x06)  // (LDC1614 ONLY) Channel 3 MSB Conversion Result and Error Status
#define LDC1612_1614_REG_DATA_3_LSB_ADDRESS           (0x07)  // (LDC1614 ONLY) Channel 3 LSB Conversion Result. Must be read after Register address 0x06.
#endif

#define LDC1612_1614_REG_RCOUNT_0_ADDRESS             (0x08)  // Reference Count setting for Channel 0
#define LDC1612_1614_REG_RCOUNT_1_ADDRESS             (0x09)  // Reference Count setting for Channel 1
#ifdef USE_LDC1614
#define LDC1612_1614_REG_RCOUNT_2_ADDRESS             (0x0A)  // (LDC1614 ONLY) Reference Count setting for Channel 2
#define LDC1612_1614_REG_RCOUNT_3_ADDRESS             (0x0B)  // (LDC1614 ONLY) Reference Count setting for Channel 3
#endif

#define LDC1612_1614_REG_OFFSET_0_ADDRESS             (0x0C)  // Offset value for Channel 0
#define LDC1612_1614_REG_OFFSET_1_ADDRESS             (0x0D)  // Offset value for Channel 1
#ifdef USE_LDC1614
#define LDC1612_1614_REG_OFFSET_2_ADDRESS             (0x0E)  // (LDC1614 ONLY) Offset value for Channel 2
#define LDC1612_1614_REG_OFFSET_3_ADDRESS             (0x0F)  // (LDC1614 ONLY) Offset value for Channel 3
#endif

#define LDC1612_1614_REG_SETTLECOUNT_0_ADDRESS        (0x10)  // Channel 0 Settling Reference Count
#define LDC1612_1614_REG_SETTLECOUNT_1_ADDRESS        (0x11)  // Channel 1 Settling Reference Count
#ifdef USE_LDC1614
#define LDC1612_1614_REG_SETTLECOUNT_2_ADDRESS        (0x12)  // (LDC1614 ONLY) Channel 2 Settling Reference Count
#define LDC1612_1614_REG_SETTLECOUNT_3_ADDRESS        (0x13)  // (LDC1614 ONLY) Channel 3 Settling Reference Count
#endif

#define LDC1612_1614_REG_CLOCK_DIVIDERS_0_ADDRESS     (0x14)  // Reference and Sensor Divider settings for Channel 0
#define LDC1612_1614_REG_CLOCK_DIVIDERS_1_ADDRESS     (0x15)  // Reference and Sensor Divider settings for Channel 1
#ifdef USE_LDC1614
#define LDC1612_1614_REG_CLOCK_DIVIDERS_2_ADDRESS     (0x16)  // (LDC1614 ONLY) Reference and Sensor Divider settings for Channel 2
#define LDC1612_1614_REG_CLOCK_DIVIDERS_3_ADDRESS     (0x17)  // (LDC1614 ONLY) Reference and Sensor Divider settings for Channel 3
#endif

#define LDC1612_1614_REG_STATUS_ADDRESS               (0x18)  // Device Status Report
#define LDC1612_1614_REG_ERROR_CONFIG_ADDRESS         (0x19)  // Error Reporting Configuration
#define LDC1612_1614_REG_CONFIG_ADDRESS               (0x1A)  // Conversion Configuration
#define LDC1612_1614_REG_MUX_CONFIG_ADDRESS           (0x1B)  // Channel Multiplexing Configuration
#define LDC1612_1614_REG_RESET_DEV_ADDRESS            (0x1C)  // Reset Device

#define LDC1612_1614_REG_DRIVE_CURRENT_0_ADDRESS      (0x1E)  // Channel 0 sensor current drive configuration
#define LDC1612_1614_REG_DRIVE_CURRENT_1_ADDRESS      (0x1F)  // Channel 1 sensor current drive configuration
#ifdef USE_LDC1614
#define LDC1612_1614_REG_DRIVE_CURRENT_2_ADDRESS      (0x20)  // (LDC1614 ONLY) Channel 2 sensor current drive configuration
#define LDC1612_1614_REG_DRIVE_CURRENT_3_ADDRESS      (0x21)  // (LDC1614 ONLY) Channel 3 sensor current drive configuration
#endif

#define LDC1612_1614_REG_MANUFACTURER_ID_ADDRESS      (0x7E)  // Manufacturer ID
#define LDC1612_1614_REG_DEVICE_ID_ADDRESS            (0x7F)  // Device ID

/*-------------------- M A S K S --------------------*/

// Masks for MSB errors bits for channels 0 .. 3
#define LDC1612_1614_DATA_MSB_ERR_UR0_MSK     (0x8000)  // For Channels 0 .. 3 Conversion Under-range Error Flag (Bit 15)
#define LDC1612_1614_DATA_MSB_ERR_OR0_MSK     (0x4000)  // For Channels 0 .. 3 Conversion Over-range Error Flag (Bit 14)
#define LDC1612_1614_DATA_MSB_ERR_WD0_MSK     (0x2000)  // For Channels 0 .. 3 Conversion Watchdog Timeout Error Flag (Bit 13)
#define LDC1612_1614_DATA_MSB_ERR_AE0_MSK     (0x1000)  // For Channels 0 .. 3 Conversion Amplitude Error Flag (Bit 12)

// Mask for 28 bits of data from MSB + LSB
#define LDC1612_1614_DATA_28_BIT_MSK		  (0xFFFFFFF) // Mask for Channels 0 .. 3 Data 28-bit of value

// Masks for STATUS: Address 0x18, STATUS Field Descriptions
#define LDC1612_1614_STATUS_ERR_CHAN_MSK_15   (0x8000)  // Error Channel (bit 15)
#define LDC1612_1614_STATUS_ERR_CHAN_MSK_14   (0x4000)  // Error Channel (bit 14)
#define LDC1612_1614_STATUS_ERR_UR_MSK        (0x2000)  // Conversion Under-range Error (Bit 13)
#define LDC1612_1614_STATUS_ERR_OR_MSK        (0x1000)  // Conversion Over-range Error (Bit 12)
#define LDC1612_1614_STATUS_ERR_WD_MSK        (0x0800)  // Watchdog Timeout Error (Bit 11)
#define LDC1612_1614_STATUS_ERR_AHE_MSK       (0x0400)  // Sensor Amplitude High Error (Bit 10)
#define LDC1612_1614_STATUS_ERR_ALE_MSK       (0x0200)  // Sensor Amplitude Low Error (Bit 9)
#define LDC1612_1614_STATUS_ERR_ZC_MSK        (0x0100)  // Zero Count Error (Bit 8)
#define LDC1612_1614_STATUS_DRDY_MSK          (0x0040)  // Data Ready Flag (Bit 6)
#define LDC1612_1614_STATUS_UNREADCONV0_MSK   (0x0008)  // Channel 0 Unread Conversion (Bit 3)
#define LDC1612_1614_STATUS_UNREADCONV1_MSK   (0x0004)  // Channel 1 Unread Conversion (Bit 2)
#ifdef USE_LDC1614
#define LDC1612_1614_STATUS_UNREADCONV2_MSK   (0x0002)  // (LDC1614 ONLY) Channel 2 Unread Conversion (Bit 1)
#define LDC1612_1614_STATUS_UNREADCONV3_MSK   (0x0001)  // (LDC1614 ONLY) Channel 3 Unread Conversion (Bit 0)
#endif

// Masks for ERROR_CONFIG: Address 0x19, ERROR_CONFIG Field Descriptions
#define LDC1612_1614_ERROR_CONFIG_UR_ERR2OUT_MSK    (0x8000)  // Under-range Error to Output Register (Bit 15)
#define LDC1612_1614_ERROR_CONFIG_OR_ERR2OUT_MSK    (0x4000)  // Over-range Error to Output Register (Bit 14)
#define LDC1612_1614_ERROR_CONFIG_WD_ERR2OUT_MSK    (0x2000)  // Watchdog Timeout Error to Output Register (Bit 13)
#define LDC1612_1614_ERROR_CONFIG_AH_ERR2OUT_MSK    (0x1000)  // Amplitude High Error to Output Register (Bit 12)
#define LDC1612_1614_ERROR_CONFIG_AL_ERR2OUT_MSK    (0x0800)  // Amplitude Low Error to Output Register (Bit 11)
#define LDC1612_1614_ERROR_CONFIG_UR_ERR2INT_MSK    (0x0080)  // Under-range Error to INTB (Bit 7)
#define LDC1612_1614_ERROR_CONFIG_OR_ERR2INT_MSK    (0x0040)  // Over-range Error to INTB (Bit 6)
#define LDC1612_1614_ERROR_CONFIG_WD_ERR2INT_MSK    (0x0020)  // Watchdog Timeout Error to INTB (Bit 5)
#define LDC1612_1614_ERROR_CONFIG_AH_ERR2INT_MSK    (0x0010)  // Amplitude High Error to INTB (Bit 4)
#define LDC1612_1614_ERROR_CONFIG_AL_ERR2INT_MSK    (0x0008)  // Amplitude Low Error to INTB (Bit 3)
#define LDC1612_1614_ERROR_CONFIG_ZC_ERR2INT_MSK    (0x0004)  // Zero Count Error to INTB (Bit 2)
#define LDC1612_1614_ERROR_CONFIG_DRDY_2INT_MSK     (0x0001)  // Data Ready Flag to INTB (Bit 0)

// Masks for CONFIG: Address 0x1A, CONFIG Field Descriptions
#define LDC1612_1614_CONFIG_ACTIVE_CHAN_MSK_15     	(0x8000)  // Active Channel Selection (Bits 15:14)
#define LDC1612_1614_CONFIG_ACTIVE_CHAN_MSK_14     	(0x4000)  // Active Channel Selection (Bits 15:14)
#define LDC1612_1614_CONFIG_SLEEP_MODE_EN_MSK     	(0x2000)  // Sleep Mode Enable (Bit 13)
#define LDC1612_1614_CONFIG_RP_OVERRIDE_EN_MSK    	(0x1000)  // Sensor RP Override Enable (Bit 12)
#define LDC1612_1614_CONFIG_SENSOR_ACTIVATE_SEL_MSK (0x0800)  // Sensor Activation Mode Selection (Bit 11)
#define LDC1612_1614_CONFIG_AUTO_AMP_DIS_MSK      	(0x0400)  // Automatic Sensor Amplitude Correction Disable (Bit 10)
#define LDC1612_1614_CONFIG_REF_CLK_SRC_MSK        	(0x0200)  // Select Reference Frequency Source (Bit 9)
#define LDC1612_1614_CONFIG_INTB_DIS_MSK           	(0x0080)  // INTB Disable (Bit 7)
#define LDC1612_1614_CONFIG_HIGH_CURRENT_DRV_MSK   	(0x0040)  // High Current Sensor Drive (Bit 6)

// Masks for MUX_CONFIG: Address 0x1B, MUX_CONFIG Field Descriptions
#define LDC1612_1614_MUX_CONFIG_AUTOSCAN_EN_MSK     (0x8000)  // Auto-Scan Mode Enable (Bit 15)
#define LDC1612_1614_MUX_CONFIG_RR_SEQUENCE_MSK     (0x6000)  // Auto-Scan Sequence Configuration (Bits 14:13)
#define LDC1612_1614_MUX_CONFIG_DEGLITCH_MSK        (0x0007)  // Input Deglitch Filter Bandwidth (Bits 2:0)

// Mask for RESET_DEV: Address 0x1C, RESET_DEV Field Descriptions
#define LDC1612_1614_RESET_DEV_MSK (0x8000)  // Reset Device (Bit 15)

/*-------------------- F U N C T I O N S --------------------*/
// General Functions:
// Base function for read register
uint16_t ldc1612_1614_read_register(uint8_t register_number);
// Base function for write register
void ldc1612_1614_write_register(uint8_t register_number, uint16_t data);
// Base function for read bit register, returns bit with offset
uint16_t ldc1612_1614_read_bit(uint8_t register_number, uint16_t bit_mask);
// Base function for write bit in register
void ldc1612_1614_write_bit(uint8_t register_number, uint16_t bit_mask, uint16_t bit_val);

// Read channel function with error bits
void ldc1612_1614_read_channel(uint8_t channel_number, uint32_t *p_data, uint16_t *p_error);

// All registers function
LDC1612_1614_All_Data_TypeDef ldc1612_1614_read_all_data(void);
void ldc1612_1614_write_all_registers(LDC1612_1614_All_Data_TypeDef input_ldc);

// Get IC's registers data information
uint16_t ldc1612_1614_get_rcount(uint8_t channel_number);
uint16_t ldc1612_1614_get_offset(uint8_t channel_number);
uint16_t ldc1612_1614_get_settlecount(uint8_t channel_number);
uint16_t ldc1612_1614_get_clock_dividers(uint8_t channel_number);
uint16_t ldc1612_1614_get_status(void);
uint16_t ldc1612_1614_get_error_config(void);
uint16_t ldc1612_1614_get_mux_config(void);
uint16_t ldc1612_1614_get_drive_current(uint8_t channel_number);
uint16_t ldc1612_1614_get_manufacturer_id(void);
uint16_t ldc1612_1614_get_device_id(void);

// Set data to IC's registers
void ldc1612_1614_set_rcount(uint8_t channel_number, uint16_t value);
void ldc1612_1614_set_offset(uint8_t channel_number, uint16_t value);
void ldc1612_1614_set_settlecount(uint8_t channel_number, uint16_t value);
void ldc1612_1614_set_clock_dividers(uint8_t channel_number, uint16_t value);
void ldc1612_1614_set_status(uint16_t value);
void ldc1612_1614_set_error_config(uint16_t value);
void ldc1612_1614_set_config(uint16_t value);
void ldc1612_1614_set_mux_config(uint16_t value);
void ldc1612_1614_set_drive_current(uint8_t channel_number, uint16_t value);

// Sleep mode, Shutdown, Reset
void ldc1612_1614_enter_sleep_mode(void);
void ldc1612_1614_exit_sleep_mode(void);
void ldc1612_1614_write_sd_pin(uint8_t value);
void ldc1612_1614_reset_device(void);

// Error and active channel statusese
uint8_t ldc1612_1614_get_error_channel(void);
uint8_t ldc1612_1614_get_active_channel(void);

#endif
