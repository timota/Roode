#include "VL53L1X_i2ccoms.h"
#include "vl53l1x.h"
#include "vl53l1_error_codes.h"
#include <vector>

namespace {

using esphome::i2c::ERROR_OK;
using esphome::vl53l1x::VL53L1X;

int8_t to_status(esphome::i2c::ErrorCode code) { return code == ERROR_OK ? 0 : -1; }

VL53L1X *active_sensor() { return VL53L1X::get_active_sensor(); }

uint8_t to_addr7(uint8_t device_address) { return (device_address >> 1) & 0x7F; }

}  // namespace

int8_t i2c_init() { return active_sensor() == nullptr ? VL53L1_ERROR_CONTROL_INTERFACE : VL53L1_ERROR_NONE; }

int8_t i2c_write_multi(uint8_t deviceAddress, uint16_t registerAddress, uint8_t *pdata, uint32_t count) {
  auto *sensor = active_sensor();
  if (sensor == nullptr)
    return -1;

  std::vector<uint8_t> tx;
  tx.reserve(count + 2);
  tx.push_back(registerAddress >> 8);
  tx.push_back(registerAddress & 0xFF);
  for (uint32_t i = 0; i < count; i++) {
    tx.push_back(pdata[i]);
  }

  return to_status(sensor->bridge_write_read(to_addr7(deviceAddress), tx.data(), tx.size(), nullptr, 0));
}

int8_t i2c_read_multi(uint8_t deviceAddress, uint16_t registerAddress, uint8_t *pdata, uint32_t count) {
  auto *sensor = active_sensor();
  if (sensor == nullptr)
    return -1;

  uint8_t reg[2] = {static_cast<uint8_t>(registerAddress >> 8), static_cast<uint8_t>(registerAddress & 0xFF)};
  return to_status(sensor->bridge_write_read(to_addr7(deviceAddress), reg, sizeof(reg), pdata, count));
}

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

  uint16_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  *data = tmp;

  return r;
}

int8_t i2c_read_Dword(uint8_t deviceAddress, uint16_t registerAddress, uint32_t *data) {
  uint8_t buff[4];
  int r = i2c_read_multi(deviceAddress, registerAddress, buff, 4);

  uint32_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  tmp <<= 8;
  tmp |= buff[2];
  tmp <<= 8;
  tmp |= buff[3];

  *data = tmp;

  return r;
}
