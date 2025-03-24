# LDC1612 library for STM32
### IC:  LDC1612DNTR
### MCU: STM32L072RZT6
### Callback in ldc1612_1614.c
## Functions:
### General Functions for registers:
- uint16_t ldc1612_1614_read_register(uint8_t register_number)
- void ldc1612_1614_write_register(uint8_t register_number, uint16_t data)
- uint16_t ldc1612_1614_read_bit(uint8_t register_number, uint16_t bit_mask)
- void ldc1612_1614_write_bit(uint8_t register_number, uint16_t bit_mask, uint16_t bit_val)
- void ldc1612_1614_read_channel(uint8_t channel_number, uint32_t *p_data, uint16_t *p_error);
- LDC1612_1614_All_Data_TypeDef ldc1612_1614_read_all_data(void);
- void ldc1612_1614_write_all_registers(LDC1612_1614_All_Data_TypeDef input_ldc);
### Get IC's registers data information:
- uint16_t ldc1612_1614_get_rcount(uint8_t channel_number);
- uint16_t ldc1612_1614_get_offset(uint8_t channel_number);
- uint16_t ldc1612_1614_get_settlecount(uint8_t channel_number);
- uint16_t ldc1612_1614_get_clock_dividers(uint8_t channel_number);
- uint16_t ldc1612_1614_get_status(void);
- uint16_t ldc1612_1614_get_error_config(void);
- uint16_t ldc1612_1614_get_mux_config(void);
- uint16_t ldc1612_1614_get_drive_current(uint8_t channel_number);
- uint16_t ldc1612_1614_get_manufacturer_id(void);
- uint16_t ldc1612_1614_get_device_id(void);
### Set data to IC's registers
- void ldc1612_1614_set_rcount(uint8_t channel_number, uint16_t value);
- void ldc1612_1614_set_offset(uint8_t channel_number, uint16_t value);
- void ldc1612_1614_set_settlecount(uint8_t channel_number, uint16_t value);
- void ldc1612_1614_set_clock_dividers(uint8_t channel_number, uint16_t value);
- void ldc1612_1614_set_status(uint16_t value);
- void ldc1612_1614_set_error_config(uint16_t value);
- void ldc1612_1614_set_config(uint16_t value);
- void ldc1612_1614_set_mux_config(uint16_t value);
- void ldc1612_1614_set_drive_current(uint8_t channel_number, uint16_t value);
### Sleep mode, Shutdown, Reset
- void ldc1612_1614_enter_sleep_mode(void);
- void ldc1612_1614_exit_sleep_mode(void);
- void ldc1612_1614_write_sd_pin(uint8_t value);
- void ldc1612_1614_reset_device(void);
### Error and active channel statusese
- uint8_t ldc1612_1614_get_error_channel(void);
- uint8_t ldc1612_1614_get_active_channel(void);
## Setup
### LDC1612 HelpURL page TI:
https://www.ti.com/product/LDC1612#all
### Configuration spreadsheet:
https://www.ti.com/tool/LDC-DESIGN-TOOLS
#### Enter your coil parameters in LDC_Tools-ext51.xlsx
#### Enter parameters from spreadsheet in LDC1612_1614_All_Data_TypeDef my_ldc
## Code examples
### Init device
```c
LDC1612_1614_All_Data_TypeDef my_ldc;
  // parameters
  // 2L 14T QUARTZ
  my_ldc.rcount_0 = 0x03e8;
  my_ldc.offset_0 = 0;
  my_ldc.settlecount_0 = 0x0005;
  my_ldc.clock_dividers_0 = 0x3002;
  my_ldc.config = 0x1600;
  my_ldc.mux_config = 0x020e;
  my_ldc.drive_current_0 = 0xa800; //2L 150pF 9mm 14t
  ldc1612_1614_write_all_registers(my_ldc);
```
### Read Channel data & error
```c
ldc1612_1614_read_channel(0, &my_ldc.data_channel_0, &my_ldc.error_channel_0);
```