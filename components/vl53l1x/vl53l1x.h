#pragma once
#include <math.h>

#include <vector>
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include <memory>
#include "ranging.h"
#include "roi.h"
#include "vl53l1x_idf.h"

// Map legacy ULD error codes to ESP-IDF style for compatibility with existing logic
using VL53L1_Error = esp_err_t;
constexpr VL53L1_Error VL53L1_ERROR_NONE = ESP_OK;
constexpr VL53L1_Error VL53L1_ERROR_TIME_OUT = ESP_ERR_TIMEOUT;

namespace esphome {
namespace vl53l1x {
static const char *const TAG = "VL53L1X";

/**
 * A wrapper for the VL53L1X, Time-of-Flight (ToF), laser-ranging sensor.
 * This stores user calibration info.
 */
class VL53L1X : public i2c::I2CDevice, public Component, public api::CustomAPIDevice {
 public:
  VL53L1X();
  void setup() override;
  void dump_config() override;
  ~VL53L1X();
  /** This connects directly to a sensor */
  float get_setup_priority() const override { return setup_priority::DATA; };

  optional<uint16_t> read_distance(ROI *roi, VL53L1_Error &error);
  void set_ranging_mode(const RangingMode *mode);

  optional<bool> get_xshut_state() {
    if (this->xshut_pin.has_value())
      return this->xshut_pin.value()->digital_read();
    return {};
  }
  optional<bool> get_interrupt_state() {
    if (this->interrupt_pin.has_value())
      return this->interrupt_pin.value()->digital_read();
    return {};
  }
  int get_recovery_count() const { return recovery_count_; }
  void set_sensor_id(uint8_t id) { sensor_id_ = id; }
  void set_desired_address(uint8_t addr) { desired_address_ = addr; }
  void restart();

  void set_xshut_pin(GPIOPin *pin) { this->xshut_pin = pin; }
  void set_interrupt_pin(InternalGPIOPin *pin) { this->interrupt_pin = pin; }
  optional<const RangingMode *> get_ranging_mode_override() { return this->ranging_mode_override; }
  void set_ranging_mode_override(const RangingMode *mode) { this->ranging_mode_override = {mode}; }
  void set_offset(int16_t val) { this->offset = val; }
  void set_xtalk(uint16_t val) { this->xtalk = val; }
  void set_timeout(uint16_t val) { this->timeout = val; }
  void set_sigma_threshold(uint16_t mm) { this->sigma_threshold_mm = mm; }
  void set_signal_threshold_kcps(uint16_t kcps) { this->signal_threshold_kcps = kcps; }
  void set_interrupt_state_sensor(binary_sensor::BinarySensor *sensor) { this->interrupt_state_sensor_ = sensor; }
  void enable_calibration_services(bool enabled) { this->calibration_services_enabled_ = enabled; }
  void enable_auto_calibration(bool enabled) { this->auto_calibration_enabled_ = enabled; }
  void calibrate_offset();
  void calibrate_xtalk();
  bool is_interrupt_enabled() const { return false; }

 protected:
  enum class AutoCalPhase {
    IDLE,
    OFFSET_WARM_START,
    OFFSET_WARM_WAIT,
    OFFSET_SAMPLE_START,
    OFFSET_WAIT,
    XTALK_SAMPLE_START,
    XTALK_WAIT,
    DONE,
  };

  struct AutoCalState {
    AutoCalPhase phase{AutoCalPhase::IDLE};
    uint8_t offset_idx{0};
    uint8_t offset_ok{0};
    int32_t offset_acc{0};
    uint8_t xtalk_idx{0};
    uint8_t xtalk_ok{0};
    uint32_t xtalk_acc{0};
    uint32_t deadline_ms{0};
  };

  std::unique_ptr<vl53l1x_idf::VL53L1XIDF> sensor_;
  optional<GPIOPin *> xshut_pin{};
  optional<InternalGPIOPin *> interrupt_pin{};
  optional<binary_sensor::BinarySensor *> interrupt_state_sensor_{};
  const RangingMode * ranging_mode{};
  /** Mode from user config, which can be get/set independently of current mode */
  optional<const RangingMode *> ranging_mode_override{};
  optional<int16_t> offset{};
  optional<uint16_t> xtalk{};
  optional<uint16_t> sigma_threshold_mm{};
  optional<uint16_t> signal_threshold_kcps{};
  uint16_t timeout{};
  int recovery_count_{0};
  uint8_t bus_reset_count_{0};
  uint8_t sensor_id_{0};
  uint8_t desired_address_{0x29};
  bool calibration_services_enabled_{false};
  bool auto_calibration_enabled_{true};
  bool auto_cal_scheduled_{false};
  bool auto_cal_running_{false};
  bool auto_cal_done_{false};
  bool auto_cal_retry_pending_{false};
  uint8_t auto_cal_retries_{0};
  uint32_t auto_cal_delay_ms_{2500};  // on_boot delay before first auto-cal
  uint8_t auto_cal_warm_failures_{0};
  const RangingMode *auto_cal_saved_mode_{nullptr};
  AutoCalState auto_cal_state_{};
  static std::vector<VL53L1X *> sensors;

  VL53L1_Error init();
  VL53L1_Error wait_for_boot();
  VL53L1_Error get_device_state(uint8_t *device_state);
  /**
   * Validate optional pins and log their status.
   *
   * @return false when the sensor becomes unresponsive while testing pins.
   */
  bool check_features();
  bool validate_interrupt();
  void schedule_interrupt_retry();
  void coordinated_startup_sequence();
  void coordinated_bus_reset();
  void apply_calibration_and_thresholds();
  void log_reason(const char *reason);
  void schedule_timeout_recovery();
  void update_interrupt_diagnostic();

  void soft_reset();
  void record_failure();
  uint8_t consecutive_failures_{0};
  bool interrupt_active_{false};
  bool interrupt_active_low_{true};
  uint8_t interrupt_miss_count_{0};
  bool interrupt_retry_scheduled_{false};
  uint8_t consecutive_timeouts_{0};
  bool timeout_recovery_scheduled_{false};
  uint8_t consecutive_i2c_errors_{0};
  bool bus_reset_cooldown_{false};

  // Calibration helpers
  VL53L1_Error calibrate_offset_runtime(uint16_t target_distance_mm, uint8_t samples, int16_t &result_mm);
  VL53L1_Error calibrate_xtalk_runtime(uint16_t target_distance_mm, uint8_t samples, uint16_t &result_cps);
  void calibrate_offset_service();
  void calibrate_xtalk_service();
  void schedule_default_calibration();
  void run_default_calibration();
  void start_auto_cal_async();
  void auto_cal_step();
};

}  // namespace vl53l1x
}  // namespace esphome
