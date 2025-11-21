#include "vl53l1x_idf.h"

#include <algorithm>
#include "esp_log.h"
#include "esp_err.h"

namespace esphome {
namespace vl53l1x_idf {

static const char *TAG = "vl53l1x_idf";

// Helpers -----------------------------------------------------------------

static inline uint8_t hi(uint16_t v) { return (v >> 8) & 0xFF; }
static inline uint8_t lo(uint16_t v) { return v & 0xFF; }

VL53L1XIDF::VL53L1XIDF(i2c_port_t port, uint8_t i2c_addr, TickType_t i2c_timeout_ticks)
    : port_(port), addr_(i2c_addr), i2c_timeout_(i2c_timeout_ticks) {}

// I2C primitives -----------------------------------------------------------

esp_err_t VL53L1XIDF::write_u8(uint16_t reg, uint8_t value) {
  uint8_t buf[3] = {hi(reg), lo(reg), value};
  return i2c_master_write_to_device(port_, addr_, buf, sizeof(buf), i2c_timeout_);
}

esp_err_t VL53L1XIDF::write_u16(uint16_t reg, uint16_t value) {
  uint8_t buf[4] = {hi(reg), lo(reg), hi(value), lo(value)};
  return i2c_master_write_to_device(port_, addr_, buf, sizeof(buf), i2c_timeout_);
}

esp_err_t VL53L1XIDF::read_u8(uint16_t reg, uint8_t &value) {
  uint8_t regbuf[2] = {hi(reg), lo(reg)};
  esp_err_t err = i2c_master_write_read_device(port_, addr_, regbuf, sizeof(regbuf), &value, 1, i2c_timeout_);
  return err;
}

esp_err_t VL53L1XIDF::read_u16(uint16_t reg, uint16_t &value) {
  uint8_t regbuf[2] = {hi(reg), lo(reg)};
  uint8_t data[2]{};
  esp_err_t err = i2c_master_write_read_device(port_, addr_, regbuf, sizeof(regbuf), data, 2, i2c_timeout_);
  if (err == ESP_OK) {
    value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  }
  return err;
}

// Core driver --------------------------------------------------------------

esp_err_t VL53L1XIDF::soft_reset() {
  // Datasheet reset sequence: write 0x00 then 0x01 to 0x0000
  ESP_RETURN_ON_ERROR(write_u8(REG_SOFT_RESET, 0x00), TAG, "reset step1");
  vTaskDelay(pdMS_TO_TICKS(1));
  ESP_RETURN_ON_ERROR(write_u8(REG_SOFT_RESET, 0x01), TAG, "reset step2");
  vTaskDelay(pdMS_TO_TICKS(1));
  return ESP_OK;
}

esp_err_t VL53L1XIDF::init() {
  // Minimal bring-up: reset then basic config per UM2555 expectations.
  ESP_RETURN_ON_ERROR(soft_reset(), TAG, "soft reset failed");

  // Clear interrupts and stop ranging to start from a known state.
  clear_interrupt();
  stop_ranging();

  // Recommended: set interrupt polarity active low (default 0x01 -> active high; keep default for ESP boards)
  return ESP_OK;
}

esp_err_t VL53L1XIDF::set_i2c_address(uint8_t new_addr_7bit) {
  // Write reg expects 7-bit in LSBs
  ESP_RETURN_ON_ERROR(write_u8(REG_I2C_SLAVE_DEVICE_ADDR, new_addr_7bit & 0x7F), TAG, "set addr");
  addr_ = new_addr_7bit;
  return ESP_OK;
}

esp_err_t VL53L1XIDF::set_roi(const RoiCfg &roi) {
  // size register packs (height-1)<<4 | (width-1)
  uint8_t w = std::max<uint8_t>(4, std::min<uint8_t>(16, roi.width));
  uint8_t h = std::max<uint8_t>(4, std::min<uint8_t>(16, roi.height));
  uint8_t size = (uint8_t)((h - 1) << 4) | (uint8_t)(w - 1);

  ESP_RETURN_ON_ERROR(write_u8(REG_ROI_CONFIG__MODE_ROI_XY_SIZE, size), TAG, "roi size");
  ESP_RETURN_ON_ERROR(write_u8(REG_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, roi.center), TAG, "roi center");
  return ESP_OK;
}

esp_err_t VL53L1XIDF::set_timing_budget_us(uint32_t budget_us) {
  // For brevity we reuse ULD helper equations: timeout A/B = budget/2 adjusted for guard.
  // Guard time (~4.5 ms) as in ST examples.
  const uint32_t timing_guard_us = 4528;
  if (budget_us <= timing_guard_us) return ESP_ERR_INVALID_ARG;
  uint32_t range_config_timeout_us = (budget_us - timing_guard_us) / 2;

  // Macro period derived from current VCSEL settings; we approximate using defaults: ~1.65us per macro.
  // This is a simplification adequate for initial integration; for production mirror ULD calc.
  const uint32_t macro_period_us = 1650;
  auto to_mclks = [&](uint32_t us) { return (us << 12) / macro_period_us; };

  auto encode_timeout = [](uint32_t mclks) {
    uint32_t ls = 0; uint16_t ms = 0;
    if (mclks > 0) {
      ls = mclks - 1;
      while (ls & 0xFFFFFF00) { ls >>= 1; ms++; }
    }
    return static_cast<uint16_t>((ms << 8) | (ls & 0xFF));
  };

  uint16_t timeout_a = encode_timeout(to_mclks(range_config_timeout_us));
  ESP_RETURN_ON_ERROR(write_u16(REG_RANGE_CONFIG__TIMEOUT_A, timeout_a), TAG, "timeout A");
  ESP_RETURN_ON_ERROR(write_u16(REG_RANGE_CONFIG__TIMEOUT_B, timeout_a), TAG, "timeout B");
  return ESP_OK;
}

esp_err_t VL53L1XIDF::set_intermeasurement_us(uint32_t interval_us) {
  // intermeasurement is 32-bit in multiples of oscillator periods; for simplicity store raw microseconds.
  // ULD converts using oscillator calibration; here we write ms equivalent for common 19.2MHz calibration (approx).
  uint32_t period_ms = interval_us / 1000;
  uint8_t buf[6] = {hi(REG_SYSTEM__INTERMEASUREMENT_PERIOD), lo(REG_SYSTEM__INTERMEASUREMENT_PERIOD),
                    (uint8_t)((period_ms >> 24) & 0xFF), (uint8_t)((period_ms >> 16) & 0xFF),
                    (uint8_t)((period_ms >> 8) & 0xFF), (uint8_t)(period_ms & 0xFF)};
  return i2c_master_write_to_device(port_, addr_, buf, sizeof(buf), i2c_timeout_);
}

esp_err_t VL53L1XIDF::start_ranging() {
  return write_u8(REG_SYSTEM__MODE_START, 0x40);  // back-to-back mode
}

esp_err_t VL53L1XIDF::stop_ranging() {
  return write_u8(REG_SYSTEM__MODE_START, 0x00);
}

esp_err_t VL53L1XIDF::check_data_ready(bool &ready) {
  uint8_t gpio_status = 0;
  ESP_RETURN_ON_ERROR(read_u8(REG_GPIO_TIO_HV_STATUS, gpio_status), TAG, "read status");
  ready = (gpio_status & 0x01) != 0;  // Bit0 = 1 when new data ready (matches ULD CheckForDataReady)
  return ESP_OK;
}

esp_err_t VL53L1XIDF::clear_interrupt() {
  return write_u8(REG_SYSTEM__INTERRUPT_CLEAR, 0x01);
}

esp_err_t VL53L1XIDF::read_measurement(Measurement &m) {
  uint16_t dist = 0;
  uint16_t ambient = 0;
  uint16_t signal = 0;
  uint8_t status_raw = 0;

  ESP_RETURN_ON_ERROR(read_u16(REG_RESULT__DISTANCE, dist), TAG, "read dist");
  ESP_RETURN_ON_ERROR(read_u16(REG_RESULT__AMBIENT_RATE, ambient), TAG, "read ambient");
  ESP_RETURN_ON_ERROR(read_u16(REG_RESULT__SIGRATE_MCPS, signal), TAG, "read signal");
  ESP_RETURN_ON_ERROR(read_u8(REG_RESULT__INTERRUPT_STATUS, status_raw), TAG, "read status");

  m.distance_mm = dist;
  m.ambient_rate_mcps = ambient;
  m.signal_rate_mcps = signal;
  m.status = static_cast<RangeStatus>(status_raw & 0x1F);  // lower 5 bits contain range status per ULD
  return ESP_OK;
}

}  // namespace vl53l1x_idf
}  // namespace esphome
