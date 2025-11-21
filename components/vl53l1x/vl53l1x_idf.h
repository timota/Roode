#pragma once

// ESP-IDF native VL53L1X minimal driver (register level)
// Covers the sequencing described in UM2555 for multi-zone ranging.

#include <cstdint>
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace vl53l1x_idf {

// Raw register addresses used by the VL53L1X ULD
constexpr uint16_t REG_SOFT_RESET              = 0x0000;
constexpr uint16_t REG_I2C_SLAVE_DEVICE_ADDR   = 0x0001;
constexpr uint16_t REG_GPIO_HV_MUX_CTRL        = 0x0030;
constexpr uint16_t REG_GPIO_TIO_HV_STATUS      = 0x0031;
constexpr uint16_t REG_SYSTEM__MODE_START      = 0x0087;
constexpr uint16_t REG_SYSTEM__INTERRUPT_CLEAR = 0x0086;
constexpr uint16_t REG_RESULT__INTERRUPT_STATUS= 0x0089;
constexpr uint16_t REG_RESULT__DISTANCE        = 0x0096;
constexpr uint16_t REG_RESULT__AMBIENT_RATE    = 0x0090;
constexpr uint16_t REG_RESULT__SIGMA_MM        = 0x0098;
constexpr uint16_t REG_RESULT__SIGRATE_MCPS    = 0x009E;
constexpr uint16_t REG_RESULT__OSC_CALIBRATE   = 0x00DE;
constexpr uint16_t REG_ALGO__PART_TO_PART_RANGE_OFFSET_MM = 0x001E;
constexpr uint16_t REG_ALGO__CROSSTALK_COMPENSATION_RATE  = 0x0016;
constexpr uint16_t REG_SIGMA_THRESHOLD           = 0x0064;
constexpr uint16_t REG_MIN_COUNT_RATE_RTN_LIMIT  = 0x0066;
constexpr uint16_t REG_ROI_CONFIG__MODE_ROI_CENTRE_SPAD = 0x007F;
constexpr uint16_t REG_ROI_CONFIG__MODE_ROI_XY_SIZE     = 0x0080;
constexpr uint16_t REG_SYSTEM__INTERMEASUREMENT_PERIOD  = 0x006C;
constexpr uint16_t REG_RANGE_CONFIG__TIMEOUT_A           = 0x005E;
constexpr uint16_t REG_RANGE_CONFIG__TIMEOUT_B           = 0x0061;
constexpr uint16_t REG_PHASECAL_CONFIG__TIMEOUT          = 0x004B;

// Range status mapping aligns with UM2555/ULD
enum class RangeStatus : uint8_t {
  RANGE_VALID = 0,
  SIGMA_FAIL = 1,
  SIGNAL_FAIL = 2,
  RANGE_VALID_MIN_RANGE_CLIPPED = 3,
  OUT_OF_BOUNDS_FAIL = 4,
  HARDWARE_FAIL = 5,
  RANGE_VALID_NO_WRAP_CHECK_FAIL = 6,
  WRAP_TARGET_FAIL = 7,
  XTALK_SIGNAL_FAIL = 9,
  SYNCHRONISATION_INT = 10,
  RANGE_VALID_MERGED_PULSE = 11,
  TARGET_PRESENT_LACK_OF_SIGNAL = 12,
  MIN_RANGE_FAIL = 13,
  RANGE_INVALID = 14,
  NONE = 255
};

struct RoiCfg {
  uint8_t width;   // 4-16
  uint8_t height;  // 4-16
  uint8_t center;  // SPAD index 0-255
};

struct Measurement {
  uint16_t distance_mm{0};
  uint16_t ambient_rate_mcps{0};
  uint16_t signal_rate_mcps{0};
  RangeStatus status{RangeStatus::NONE};
};

class VL53L1XIDF {
 public:
  VL53L1XIDF(i2c::I2CBus *bus, uint8_t i2c_addr = 0x29);

  esp_err_t init();
  esp_err_t soft_reset();
  esp_err_t set_i2c_address(uint8_t new_addr_7bit);

  esp_err_t set_roi(const RoiCfg &roi);
  esp_err_t set_timing_budget_us(uint32_t budget_us);
  esp_err_t set_intermeasurement_us(uint32_t interval_us);
  esp_err_t set_offset_mm(int16_t offset_mm);
  esp_err_t set_xtalk(uint16_t xtalk_cps);
  esp_err_t set_sigma_threshold_mm(uint16_t sigma_mm);
  esp_err_t set_signal_threshold_cps(uint16_t kcps);

  esp_err_t calibrate_offset_once(uint16_t target_distance_mm, uint16_t &written_offset_mm);

  esp_err_t start_ranging();
  esp_err_t stop_ranging();
  esp_err_t check_data_ready(bool &ready);
  esp_err_t clear_interrupt();
  esp_err_t read_measurement(Measurement &m);

 private:
  i2c::I2CBus *bus_{nullptr};
  uint8_t addr_;  // 7-bit address

  esp_err_t write_u8(uint16_t reg, uint8_t value);
  esp_err_t write_u16(uint16_t reg, uint16_t value);
  esp_err_t read_u8(uint16_t reg, uint8_t &value);
  esp_err_t read_u16(uint16_t reg, uint16_t &value);
};

}  // namespace vl53l1x_idf
}  // namespace esphome
