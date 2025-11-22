#include "vl53l1x_idf.h"

#include <algorithm>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esp_err.h"
#include "esp_check.h"

namespace esphome {
namespace vl53l1x_idf {

static const char *TAG = "vl53l1x_idf";

static inline uint8_t hi(uint16_t v) { return (v >> 8) & 0xFF; }
static inline uint8_t lo(uint16_t v) { return v & 0xFF; }

VL53L1XIDF::VL53L1XIDF(i2c::I2CBus *bus, uint8_t i2c_addr) : bus_(bus), addr_(i2c_addr) {}

esp_err_t VL53L1XIDF::bus_write(const uint8_t *data, size_t len) {
  if (bus_ == nullptr) return ESP_ERR_INVALID_STATE;
  return bus_->write(this->addr_, data, len, true) == i2c::ERROR_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t VL53L1XIDF::bus_write_read(const uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen) {
  if (bus_ == nullptr) return ESP_ERR_INVALID_STATE;
  return bus_->write_readv(this->addr_, wdata, wlen, rdata, rlen) == i2c::ERROR_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t VL53L1XIDF::write_u8(uint16_t reg, uint8_t value) {
  uint8_t buf[3] = {hi(reg), lo(reg), value};
  return bus_write(buf, sizeof(buf));
}

esp_err_t VL53L1XIDF::write_u16(uint16_t reg, uint16_t value) {
  uint8_t buf[4] = {hi(reg), lo(reg), hi(value), lo(value)};
  return bus_write(buf, sizeof(buf));
}

esp_err_t VL53L1XIDF::read_u8(uint16_t reg, uint8_t &value) {
  uint8_t regbuf[2] = {hi(reg), lo(reg)};
  return bus_write_read(regbuf, sizeof(regbuf), &value, 1);
}

esp_err_t VL53L1XIDF::read_u16(uint16_t reg, uint16_t &value) {
  uint8_t regbuf[2] = {hi(reg), lo(reg)};
  uint8_t data[2]{};
  auto err = bus_write_read(regbuf, sizeof(regbuf), data, 2);
  if (err == ESP_OK) value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  return err;
}

esp_err_t VL53L1XIDF::soft_reset() {
  ESP_RETURN_ON_ERROR(write_u8(REG_SOFT_RESET, 0x00), TAG, "reset step1");
  delay(1);
  ESP_RETURN_ON_ERROR(write_u8(REG_SOFT_RESET, 0x01), TAG, "reset step2");
  delay(1);
  return ESP_OK;
}

esp_err_t VL53L1XIDF::init() {
  if (bus_ == nullptr) return ESP_ERR_INVALID_STATE;
  ESP_RETURN_ON_ERROR(soft_reset(), TAG, "soft reset failed");
  clear_interrupt();
  stop_ranging();
  return ESP_OK;
}

esp_err_t VL53L1XIDF::set_i2c_address(uint8_t new_addr_7bit) {
  ESP_RETURN_ON_ERROR(write_u8(REG_I2C_SLAVE_DEVICE_ADDR, new_addr_7bit & 0x7F), TAG, "set addr");
  addr_ = new_addr_7bit;
  return ESP_OK;
}

esp_err_t VL53L1XIDF::set_roi(const RoiCfg &roi) {
  uint8_t w = std::max<uint8_t>(4, std::min<uint8_t>(16, roi.width));
  uint8_t h = std::max<uint8_t>(4, std::min<uint8_t>(16, roi.height));
  uint8_t size = static_cast<uint8_t>(((h - 1) << 4) | (w - 1));
  ESP_RETURN_ON_ERROR(write_u8(REG_ROI_CONFIG__MODE_ROI_XY_SIZE, size), TAG, "roi size");
  ESP_RETURN_ON_ERROR(write_u8(REG_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, roi.center), TAG, "roi center");
  return ESP_OK;
}

esp_err_t VL53L1XIDF::set_timing_budget_us(uint32_t budget_us) {
  const uint32_t timing_guard_us = 4528;
  if (budget_us <= timing_guard_us) return ESP_ERR_INVALID_ARG;
  uint32_t range_timeout_us = (budget_us - timing_guard_us) / 2;

  auto calc_macro_period = [&](uint8_t vcsel_period_reg) {
    uint16_t fast_osc_frequency = 0;
    if (read_u16(0x0006, fast_osc_frequency) != ESP_OK) return (uint32_t)0;
    uint32_t pll_period_us = ((uint32_t)1 << 30) / fast_osc_frequency;
    uint8_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
    uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;
    return macro_period_us;
  };

  auto timeout_microseconds_to_mclks = [](uint32_t timeout_us, uint32_t macro_period_us) {
    return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
  };

  auto encode_timeout = [](uint32_t timeout_mclks) {
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;
    if (timeout_mclks > 0) {
      ls_byte = timeout_mclks - 1;
      while (ls_byte & 0xFFFFFF00) {
        ls_byte >>= 1;
        ms_byte++;
      }
    }
    return static_cast<uint16_t>((ms_byte << 8) | (ls_byte & 0xFF));
  };

  uint8_t vcsel_a = 0, vcsel_b = 0;
  read_u8(0x0060, vcsel_a);
  read_u8(0x0063, vcsel_b);
  uint32_t macro_a = calc_macro_period(vcsel_a);
  uint32_t macro_b = calc_macro_period(vcsel_b);
  if (macro_a == 0 || macro_b == 0) return ESP_FAIL;

  uint32_t phasecal_timeout_mclks = timeout_microseconds_to_mclks(1000, macro_a);
  if (phasecal_timeout_mclks > 0xFF) phasecal_timeout_mclks = 0xFF;
  ESP_RETURN_ON_ERROR(write_u8(REG_PHASECAL_CONFIG__TIMEOUT, (uint8_t)phasecal_timeout_mclks), TAG, "phasecal");

  uint32_t range_timeout_mclks_a = timeout_microseconds_to_mclks(range_timeout_us, macro_a);
  uint32_t range_timeout_mclks_b = timeout_microseconds_to_mclks(range_timeout_us, macro_b);

  ESP_RETURN_ON_ERROR(write_u16(REG_RANGE_CONFIG__TIMEOUT_A, encode_timeout(range_timeout_mclks_a)), TAG, "toa");
  ESP_RETURN_ON_ERROR(write_u16(REG_RANGE_CONFIG__TIMEOUT_B, encode_timeout(range_timeout_mclks_b)), TAG, "tob");
  return ESP_OK;
}

esp_err_t VL53L1XIDF::set_intermeasurement_us(uint32_t interval_us) {
  uint32_t period_ms = interval_us / 1000;
  uint8_t buf[6] = {hi(REG_SYSTEM__INTERMEASUREMENT_PERIOD), lo(REG_SYSTEM__INTERMEASUREMENT_PERIOD),
                    (uint8_t)((period_ms >> 24) & 0xFF), (uint8_t)((period_ms >> 16) & 0xFF),
                    (uint8_t)((period_ms >> 8) & 0xFF), (uint8_t)(period_ms & 0xFF)};
  return bus_write(buf, sizeof(buf));
}

esp_err_t VL53L1XIDF::set_offset_mm(int16_t offset_mm) {
  uint16_t regval = static_cast<uint16_t>(offset_mm);
  return write_u16(REG_ALGO__PART_TO_PART_RANGE_OFFSET_MM, regval);
}

esp_err_t VL53L1XIDF::set_xtalk(uint16_t xtalk_cps) {
  uint32_t mcps = xtalk_cps / 1000;
  uint16_t regval = static_cast<uint16_t>(mcps << 7);
  return write_u16(REG_ALGO__CROSSTALK_COMPENSATION_RATE, regval);
}

esp_err_t VL53L1XIDF::set_sigma_threshold_mm(uint16_t sigma_mm) {
  return write_u16(REG_SIGMA_THRESHOLD, sigma_mm << 2);
}

esp_err_t VL53L1XIDF::set_signal_threshold_cps(uint16_t kcps) {
  uint16_t mcps_9_7 = static_cast<uint16_t>((kcps * 1000) >> 7);
  return write_u16(REG_MIN_COUNT_RATE_RTN_LIMIT, mcps_9_7);
}

esp_err_t VL53L1XIDF::calibrate_offset_once(uint16_t target_distance_mm, uint16_t &written_offset_mm) {
  ESP_RETURN_ON_ERROR(start_ranging(), TAG, "cal start");
  bool ready = false;
  uint32_t start = millis();
  // Allow a longer window during calibration to account for slower first measurements
  while (!ready && (millis() - start) < 400) {
    check_data_ready(ready);
    if (!ready) delay(5);
  }
  if (!ready) {
    stop_ranging();
    return ESP_ERR_TIMEOUT;
  }
  Measurement m;
  auto err = read_measurement(m);
  clear_interrupt();
  stop_ranging();
  if (err != ESP_OK) return err;
  int16_t offset = static_cast<int16_t>(target_distance_mm) - static_cast<int16_t>(m.distance_mm);
  written_offset_mm = offset;
  return set_offset_mm(offset);
}

esp_err_t VL53L1XIDF::start_ranging() { return write_u8(REG_SYSTEM__MODE_START, 0x40); }

esp_err_t VL53L1XIDF::stop_ranging() { return write_u8(REG_SYSTEM__MODE_START, 0x00); }

esp_err_t VL53L1XIDF::check_data_ready(bool &ready) {
  uint8_t gpio_status = 0;
  ESP_RETURN_ON_ERROR(read_u8(REG_GPIO_TIO_HV_STATUS, gpio_status), TAG, "read status");
  ready = (gpio_status & 0x01) != 0;
  return ESP_OK;
}

esp_err_t VL53L1XIDF::clear_interrupt() { return write_u8(REG_SYSTEM__INTERRUPT_CLEAR, 0x01); }

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
  m.status = static_cast<RangeStatus>(status_raw & 0x1F);
  return ESP_OK;
}

}  // namespace vl53l1x_idf
}  // namespace esphome
