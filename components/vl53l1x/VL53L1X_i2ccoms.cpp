#include "VL53L1X_i2ccoms.h"
#include "vl53l1_error_codes.h"

// Temporary stubs for ESP-IDF migration; real implementation will use ESPHome I2C

int8_t i2c_init() { return VL53L1_ERROR_NONE; }

int8_t i2c_write_multi(uint8_t, uint16_t, uint8_t *, uint32_t) { return VL53L1_ERROR_NONE; }
int8_t i2c_read_multi(uint8_t, uint16_t, uint8_t *, uint32_t) { return VL53L1_ERROR_NONE; }

int8_t i2c_write_byte(uint8_t deviceAddress, uint16_t registerAddress, uint8_t data) {
  return i2c_write_multi(deviceAddress, registerAddress, &data, 1);
}

int8_t i2c_write_word(uint8_t deviceAddress, uint16_t registerAddress, uint16_t data) {
  uint8_t buff[2];
  buff[1] = data & 0xFF;
  buff[0] = data >> 8;
  return i2c_write_multi(deviceAddress, registerAddress, buff, 2);
}

int8_t i2c_write_Dword(uint8_t deviceAddress, uint16_t registerAddress, uint32_t data) {
  uint8_t buff[4];
  buff[3] = data & 0xFF;
  buff[2] = data >> 8;
  buff[1] = data >> 16;
  buff[0] = data >> 24;
  return i2c_write_multi(deviceAddress, registerAddress, buff, 4);
}

int8_t i2c_read_byte(uint8_t deviceAddress, uint16_t registerAddress, uint8_t *data) {
  return i2c_read_multi(deviceAddress, registerAddress, data, 1);
}

int8_t i2c_read_word(uint8_t deviceAddress, uint16_t registerAddress, uint16_t *data) {
  uint8_t buff[2];
  int r = i2c_read_multi(deviceAddress, registerAddress, buff, 2);
  uint16_t tmp = (buff[0] << 8) | buff[1];
  *data = tmp;
  return r;
}

int8_t i2c_read_Dword(uint8_t deviceAddress, uint16_t registerAddress, uint32_t *data) {
  uint8_t buff[4];
  int r = i2c_read_multi(deviceAddress, registerAddress, buff, 4);
  uint32_t tmp = (uint32_t(buff[0]) << 24) | (uint32_t(buff[1]) << 16) | (uint32_t(buff[2]) << 8) | buff[3];
  *data = tmp;
  return r;
}
