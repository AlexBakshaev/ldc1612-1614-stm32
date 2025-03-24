/*
 * ldc1612.c
 *
 *  Created on: Mar 15, 2025
 *  	Author: Alex Bakshaev (aka Дрочеслав in Blue Archive)
 */
#include <ldc1612_1614.h>

/*-------------------- I N C L U D E S --------------------*/
#include "main.h"
#include <stdio.h>
#include "stm32l0xx_hal_i2c.h"

/*-------------------- E X T E R N A L --------------------*/
extern I2C_HandleTypeDef hi2c2;
#define LDC1612_1614_I2C hi2c2

/*-------------------- I N T E R R U P T  C A L L B A C K --------------------*/
#ifdef ENABLE_INTERRUPT

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == LDC1612_1614_INT_PIN) {

	}
}
#endif

/*-------------------- F U N C T I O N S --------------------*/
// General Functions:
// Base function for read register
uint16_t ldc1612_1614_read_register(uint8_t register_number) {
	uint8_t temp_reg_arr[2] = {0,0};
#ifdef LDC1612_1614_USE_HAL
	HAL_I2C_Mem_Read(&LDC1612_1614_I2C, LDC1612_I2C_ADDRESS<<1, register_number, 1, temp_reg_arr, 2, HAL_MAX_DELAY);
#elif defined(LDC1612_1614_USE_CMSIS)
#endif
	uint16_t temp_reg_data = (temp_reg_arr[0] << 8) | temp_reg_arr[1];
	return temp_reg_data;
}
// Base function for write register
void ldc1612_1614_write_register(uint8_t register_number, uint16_t data) {
	uint8_t temp_reg_arr[2] = {0,0};
	temp_reg_arr[0] = data >> 8;
	temp_reg_arr[1] = data;
#ifdef LDC1612_1614_USE_HAL
	HAL_I2C_Mem_Write(&LDC1612_1614_I2C, LDC1612_I2C_ADDRESS<<1, register_number, 1, temp_reg_arr, 2, HAL_MAX_DELAY);
#elif defined(LDC1612_1614_USE_CMSIS)
#endif
}

// Base function for read bit register, returns bit with offset
uint16_t ldc1612_1614_read_bit(uint8_t register_number, uint16_t bit_mask) {
	uint16_t temp_reg_data = 0;
#ifdef LDC1612_1614_USE_HAL
	temp_reg_data = ldc1612_1614_read_register(register_number);
#elif defined(LDC1612_1614_USE_CMSIS)
#endif
	temp_reg_data &= bit_mask;
	return temp_reg_data;
}
// Base function for write bit in register
void ldc1612_1614_write_bit(uint8_t register_number, uint16_t bit_mask, uint16_t bit_val) {
	uint16_t temp_reg_data = 0;
#ifdef LDC1612_1614_USE_HAL
	temp_reg_data = ldc1612_1614_read_register(register_number);
#elif defined(LDC1612_1614_USE_CMSIS)
#endif
	if(bit_val) {
		temp_reg_data |= bit_mask;
	}
	else {
		temp_reg_data &= (~bit_mask);
	}
#ifdef LDC1612_1614_USE_HAL
	ldc1612_1614_write_register(register_number, temp_reg_data);
#elif defined(LDC1612_1614_USE_CMSIS)
#endif
}

/* @brief Read channel function with error bits
 * @param channel_number channel 0/1 for ldc1612 | 0/1/2/3 for ldc1614
 * @param *p_data 32-bit pointer for 28-bit data
 * @param *p_error pointer to 16-bit error
*/
void ldc1612_1614_read_channel(uint8_t channel_number, uint32_t *p_data, uint16_t *p_error) {
	uint16_t temp_msb = 0;
	uint16_t temp_lsb = 0;
	temp_msb = ldc1612_1614_read_register(LDC1612_1614_REG_DATA_0_MSB_ADDRESS + (2*channel_number));
	temp_lsb = ldc1612_1614_read_register(1 + LDC1612_1614_REG_DATA_0_MSB_ADDRESS + (2*channel_number));
	*p_data = (temp_msb << 16) | temp_lsb;
	*p_data &= LDC1612_1614_DATA_28_BIT_MSK;

	// bit mask for 4 error bits
	*p_error = (0xF000) | temp_msb;
}

// All registers function
LDC1612_1614_All_Data_TypeDef ldc1612_1614_read_all_data(void) {
	LDC1612_1614_All_Data_TypeDef temp_ldc = {0};
	// Read registers
	temp_ldc.data_0_msb = ldc1612_1614_read_register(LDC1612_1614_REG_DATA_0_MSB_ADDRESS);
	temp_ldc.data_0_lsb = ldc1612_1614_read_register(LDC1612_1614_REG_DATA_0_LSB_ADDRESS);
	temp_ldc.data_1_msb = ldc1612_1614_read_register(LDC1612_1614_REG_DATA_1_MSB_ADDRESS);
	temp_ldc.data_1_lsb = ldc1612_1614_read_register(LDC1612_1614_REG_DATA_1_LSB_ADDRESS);
#ifdef USE_LDC1614
	temp_ldc.data_2_msb = ldc1612_1614_read_register(LDC1612_1614_REG_DATA_2_MSB_ADDRESS);
	temp_ldc.data_2_lsb = ldc1612_1614_read_register(LDC1612_1614_REG_DATA_2_LSB_ADDRESS);
	temp_ldc.data_3_msb = ldc1612_1614_read_register(LDC1612_1614_REG_DATA_3_MSB_ADDRESS);
	temp_ldc.data_3_lsb = ldc1612_1614_read_register(LDC1612_1614_REG_DATA_3_LSB_ADDRESS);
#endif
	temp_ldc.rcount_0 = ldc1612_1614_read_register(LDC1612_1614_REG_RCOUNT_0_ADDRESS);
	temp_ldc.rcount_1 = ldc1612_1614_read_register(LDC1612_1614_REG_RCOUNT_1_ADDRESS);
#ifdef USE_LDC1614
	temp_ldc.rcount_2 = ldc1612_1614_read_register(LDC1612_1614_REG_RCOUNT_2_ADDRESS);
	temp_ldc.rcount_3 = ldc1612_1614_read_register(LDC1612_1614_REG_RCOUNT_3_ADDRESS);
#endif
	temp_ldc.offset_0 = ldc1612_1614_read_register(LDC1612_1614_REG_OFFSET_0_ADDRESS);
	temp_ldc.offset_1 = ldc1612_1614_read_register(LDC1612_1614_REG_OFFSET_1_ADDRESS);
#ifdef USE_LDC1614
	temp_ldc.offset_2 = ldc1612_1614_read_register(LDC1612_1614_REG_OFFSET_2_ADDRESS);
	temp_ldc.offset_3 = ldc1612_1614_read_register(LDC1612_1614_REG_OFFSET_3_ADDRESS);
#endif
	temp_ldc.settlecount_0 = ldc1612_1614_read_register(LDC1612_1614_REG_SETTLECOUNT_0_ADDRESS);
	temp_ldc.settlecount_1 = ldc1612_1614_read_register(LDC1612_1614_REG_SETTLECOUNT_1_ADDRESS);
#ifdef USE_LDC1614
	temp_ldc.settlecount_2 = ldc1612_1614_read_register(LDC1612_1614_REG_SETTLECOUNT_2_ADDRESS);
	temp_ldc.settlecount_3 = ldc1612_1614_read_register(LDC1612_1614_REG_SETTLECOUNT_3_ADDRESS);
#endif
	temp_ldc.clock_dividers_0 = ldc1612_1614_read_register(LDC1612_1614_REG_CLOCK_DIVIDERS_0_ADDRESS);
	temp_ldc.clock_dividers_1 = ldc1612_1614_read_register(LDC1612_1614_REG_CLOCK_DIVIDERS_1_ADDRESS);
#ifdef USE_LDC1614
	temp_ldc.clock_dividers_2 = ldc1612_1614_read_register(LDC1612_1614_REG_CLOCK_DIVIDERS_2_ADDRESS);
	temp_ldc.clock_dividers_3 = ldc1612_1614_read_register(LDC1612_1614_REG_CLOCK_DIVIDERS_3_ADDRESS);
#endif
	temp_ldc.status = ldc1612_1614_read_register(LDC1612_1614_REG_STATUS_ADDRESS);
	temp_ldc.error_config = ldc1612_1614_read_register(LDC1612_1614_REG_ERROR_CONFIG_ADDRESS);
	temp_ldc.config = ldc1612_1614_read_register(LDC1612_1614_REG_CONFIG_ADDRESS);
	temp_ldc.mux_config = ldc1612_1614_read_register(LDC1612_1614_REG_MUX_CONFIG_ADDRESS);
	temp_ldc.reset_dev = ldc1612_1614_read_register(LDC1612_1614_REG_RESET_DEV_ADDRESS);
	temp_ldc.drive_current_0 = ldc1612_1614_read_register(LDC1612_1614_REG_DRIVE_CURRENT_0_ADDRESS);
	temp_ldc.drive_current_1 = ldc1612_1614_read_register(LDC1612_1614_REG_DRIVE_CURRENT_1_ADDRESS);
#ifdef USE_LDC1614
	temp_ldc.drive_current_2 = ldc1612_1614_read_register(LDC1612_1614_REG_DRIVE_CURRENT_2_ADDRESS);
	temp_ldc.drive_current_3 = ldc1612_1614_read_register(LDC1612_1614_REG_DRIVE_CURRENT_3_ADDRESS);
#endif
	temp_ldc.manufacturer_id = ldc1612_1614_read_register(LDC1612_1614_REG_MANUFACTURER_ID_ADDRESS);
	temp_ldc.device_id = ldc1612_1614_read_register(LDC1612_1614_REG_DEVICE_ID_ADDRESS);

	// Combined data
	temp_ldc.data_channel_0 = LDC1612_1614_DATA_28_BIT_MSK & ((temp_ldc.data_0_msb << 16) | temp_ldc.data_0_lsb);
	temp_ldc.data_channel_1 = LDC1612_1614_DATA_28_BIT_MSK & ((temp_ldc.data_1_msb << 16) | temp_ldc.data_1_lsb);
#ifdef USE_LDC1614
	temp_ldc.data_channel_2 = LDC1612_1614_DATA_28_BIT_MSK & ((temp_ldc.data_2_msb << 16) | temp_ldc.data_2_lsb);
	temp_ldc.data_channel_3 = LDC1612_1614_DATA_28_BIT_MSK & ((temp_ldc.data_3_msb << 16) | temp_ldc.data_3_lsb);
#endif
	temp_ldc.error_channel_0 = 0xF000 & temp_ldc.data_0_msb;
	temp_ldc.error_channel_1 = 0xF000 & temp_ldc.data_1_msb;
#ifdef USE_LDC1614
	temp_ldc.error_channel_2 = 0xF000 & temp_ldc.data_2_msb;
	temp_ldc.error_channel_3 = 0xF000 & temp_ldc.data_3_msb;
#endif
	temp_ldc.error_channel = ldc1612_1614_get_error_channel();
	temp_ldc.active_channel = ldc1612_1614_get_active_channel();
	return temp_ldc;
}

void ldc1612_1614_write_all_registers(LDC1612_1614_All_Data_TypeDef input_ldc) {
	ldc1612_1614_write_register(LDC1612_1614_REG_RCOUNT_0_ADDRESS, input_ldc.rcount_0);
	ldc1612_1614_write_register(LDC1612_1614_REG_RCOUNT_1_ADDRESS, input_ldc.rcount_1);
#ifdef USE_LDC1614
	ldc1612_1614_write_register(LDC1612_1614_REG_RCOUNT_2_ADDRESS, input_ldc.rcount_2);
	ldc1612_1614_write_register(LDC1612_1614_REG_RCOUNT_3_ADDRESS, input_ldc.rcount_3);
#endif
	ldc1612_1614_write_register(LDC1612_1614_REG_OFFSET_0_ADDRESS, input_ldc.offset_0);
	ldc1612_1614_write_register(LDC1612_1614_REG_OFFSET_1_ADDRESS, input_ldc.offset_1);
#ifdef USE_LDC1614
	ldc1612_1614_write_register(LDC1612_1614_REG_OFFSET_2_ADDRESS, input_ldc.offset_2);
	ldc1612_1614_write_register(LDC1612_1614_REG_OFFSET_3_ADDRESS, input_ldc.offset_3);
#endif
	ldc1612_1614_write_register(LDC1612_1614_REG_SETTLECOUNT_0_ADDRESS, input_ldc.settlecount_0);
	ldc1612_1614_write_register(LDC1612_1614_REG_SETTLECOUNT_1_ADDRESS, input_ldc.settlecount_1);
#ifdef USE_LDC1614
	ldc1612_1614_write_register(LDC1612_1614_REG_SETTLECOUNT_2_ADDRESS, input_ldc.settlecount_2);
	ldc1612_1614_write_register(LDC1612_1614_REG_SETTLECOUNT_3_ADDRESS, input_ldc.settlecount_3);
#endif
	ldc1612_1614_write_register(LDC1612_1614_REG_CLOCK_DIVIDERS_0_ADDRESS, input_ldc.clock_dividers_0);
	ldc1612_1614_write_register(LDC1612_1614_REG_CLOCK_DIVIDERS_1_ADDRESS, input_ldc.clock_dividers_1);
#ifdef USE_LDC1614
	ldc1612_1614_write_register(LDC1612_1614_REG_CLOCK_DIVIDERS_2_ADDRESS, input_ldc.clock_dividers_2);
	ldc1612_1614_write_register(LDC1612_1614_REG_CLOCK_DIVIDERS_3_ADDRESS, input_ldc.clock_dividers_3);
#endif
	ldc1612_1614_write_register(LDC1612_1614_REG_STATUS_ADDRESS, input_ldc.status);
	ldc1612_1614_write_register(LDC1612_1614_REG_ERROR_CONFIG_ADDRESS, input_ldc.error_config);
	ldc1612_1614_write_register(LDC1612_1614_REG_CONFIG_ADDRESS, input_ldc.config);
	ldc1612_1614_write_register(LDC1612_1614_REG_MUX_CONFIG_ADDRESS, input_ldc.mux_config);
	ldc1612_1614_write_register(LDC1612_1614_REG_RESET_DEV_ADDRESS, input_ldc.reset_dev);
	ldc1612_1614_write_register(LDC1612_1614_REG_DRIVE_CURRENT_0_ADDRESS, input_ldc.drive_current_0);
	ldc1612_1614_write_register(LDC1612_1614_REG_DRIVE_CURRENT_1_ADDRESS, input_ldc.drive_current_1);
#ifdef USE_LDC1614
	ldc1612_1614_write_register(LDC1612_1614_REG_DRIVE_CURRENT_2_ADDRESS, input_ldc.drive_current_2);
	ldc1612_1614_write_register(LDC1612_1614_REG_DRIVE_CURRENT_3_ADDRESS, input_ldc.drive_current_3);
#endif
}

// Get IC's registers data information
uint16_t ldc1612_1614_get_rcount(uint8_t channel_number) {
	return ldc1612_1614_read_register(LDC1612_1614_REG_RCOUNT_0_ADDRESS + channel_number);
}
uint16_t ldc1612_1614_get_offset(uint8_t channel_number) {
	return ldc1612_1614_read_register(LDC1612_1614_REG_OFFSET_0_ADDRESS + channel_number);
}

uint16_t ldc1612_1614_get_settlecount(uint8_t channel_number) {
	return ldc1612_1614_read_register(LDC1612_1614_REG_SETTLECOUNT_0_ADDRESS + channel_number);
}

uint16_t ldc1612_1614_get_clock_dividers(uint8_t channel_number) {
	return ldc1612_1614_read_register(LDC1612_1614_REG_CLOCK_DIVIDERS_0_ADDRESS + channel_number);
}

uint16_t ldc1612_1614_get_status(void) {
	return ldc1612_1614_read_register(LDC1612_1614_REG_STATUS_ADDRESS);
}

uint16_t ldc1612_1614_get_error_config(void) {
	return ldc1612_1614_read_register(LDC1612_1614_REG_ERROR_CONFIG_ADDRESS);
}

uint16_t ldc1612_1614_get_mux_config(void) {
	return ldc1612_1614_read_register(LDC1612_1614_REG_MUX_CONFIG_ADDRESS);
}

uint16_t ldc1612_1614_get_drive_current(uint8_t channel_number) {
	return ldc1612_1614_read_register(LDC1612_1614_REG_DRIVE_CURRENT_0_ADDRESS + channel_number);
}

uint16_t ldc1612_1614_get_manufacturer_id(void) {
	return ldc1612_1614_read_register(LDC1612_1614_REG_MANUFACTURER_ID_ADDRESS);
}

uint16_t ldc1612_1614_get_device_id(void) {
	return ldc1612_1614_read_register(LDC1612_1614_REG_DEVICE_ID_ADDRESS);
}

// Set data to IC's registers
void ldc1612_1614_set_rcount(uint8_t channel_number, uint16_t value) {
	ldc1612_1614_write_register(LDC1612_1614_REG_RCOUNT_0_ADDRESS + channel_number, value);
}
void ldc1612_1614_set_offset(uint8_t channel_number, uint16_t value) {
	ldc1612_1614_write_register(LDC1612_1614_REG_OFFSET_0_ADDRESS + channel_number, value);
}

void ldc1612_1614_set_settlecount(uint8_t channel_number, uint16_t value) {
	ldc1612_1614_write_register(LDC1612_1614_REG_SETTLECOUNT_0_ADDRESS + channel_number, value);
}

void ldc1612_1614_set_clock_dividers(uint8_t channel_number, uint16_t value) {
	ldc1612_1614_write_register(LDC1612_1614_REG_CLOCK_DIVIDERS_0_ADDRESS + channel_number, value);
}

void ldc1612_1614_set_status(uint16_t value) {
	ldc1612_1614_write_register(LDC1612_1614_REG_STATUS_ADDRESS, value);
}

void ldc1612_1614_set_error_config(uint16_t value) {
	ldc1612_1614_write_register(LDC1612_1614_REG_ERROR_CONFIG_ADDRESS, value);
}

void ldc1612_1614_set_config(uint16_t value) {
	ldc1612_1614_write_register(LDC1612_1614_REG_CONFIG_ADDRESS, value);
}

void ldc1612_1614_set_mux_config(uint16_t value) {
	ldc1612_1614_write_register(LDC1612_1614_REG_MUX_CONFIG_ADDRESS, value);
}

void ldc1612_1614_set_drive_current(uint8_t channel_number, uint16_t value) {
	ldc1612_1614_write_register(LDC1612_1614_REG_DRIVE_CURRENT_0_ADDRESS + channel_number, value);
}

// Sleep mode, Shutdown, Reset
void ldc1612_1614_enter_sleep_mode(void) {
	ldc1612_1614_write_bit(LDC1612_1614_REG_CONFIG_ADDRESS, LDC1612_1614_CONFIG_SLEEP_MODE_EN_MSK, LDC1612_1614_EN_SLEEP_MODE);
}

void ldc1612_1614_exit_sleep_mode(void) {
	ldc1612_1614_write_bit(LDC1612_1614_REG_CONFIG_ADDRESS, LDC1612_1614_CONFIG_SLEEP_MODE_EN_MSK, !LDC1612_1614_EN_SLEEP_MODE);
}

void ldc1612_1614_write_sd_pin(uint8_t value) {
	HAL_GPIO_WritePin(LDC1612_1614_SD_GPIO_PORT, LDC1612_1614_SD_PIN, value);
}

void ldc1612_1614_reset_device(void) {
	ldc1612_1614_write_register(LDC1612_1614_REG_RESET_DEV_ADDRESS, LDC1612_1614_RESET_DEV_MSK);
}

// Error and active channel statusese
uint8_t ldc1612_1614_get_error_channel(void) {
	uint16_t temp_reg_data = 0;
	temp_reg_data = ldc1612_1614_read_register(LDC1612_1614_REG_STATUS_ADDRESS);
	temp_reg_data &= (LDC1612_1614_STATUS_ERR_CHAN_MSK_15 | LDC1612_1614_STATUS_ERR_CHAN_MSK_14);
	return (temp_reg_data >> 14);
}

uint8_t ldc1612_1614_get_active_channel(void) {
	uint16_t temp_reg_data = 0;
	temp_reg_data = ldc1612_1614_read_register(LDC1612_1614_REG_CONFIG_ADDRESS);
	temp_reg_data &= (LDC1612_1614_CONFIG_ACTIVE_CHAN_MSK_15 | LDC1612_1614_CONFIG_ACTIVE_CHAN_MSK_14);
	return (temp_reg_data >> 14);
}
