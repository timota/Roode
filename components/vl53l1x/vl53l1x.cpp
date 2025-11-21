#include "vl53l1x.h"
#include "vl53l1x_idf.h"
#include "../roode/roode.h"
#include <cstdio>

namespace esphome {
namespace vl53l1x {

std::vector<VL53L1X *> VL53L1X::sensors{};

VL53L1X::VL53L1X() : sensor_(I2C_NUM_0) {}

VL53L1X::~VL53L1X() {
  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);
    ESP_LOGD(TAG, "XShut pin set LOW - powering down sensor");
    roode::Roode::log_event("xshut_sensor_" + std::to_string(sensor_id_) + "_off");
    roode::Roode::log_event("xshut_toggled_off");
    roode::Roode::log_event("xshut_toggled");
  }
  sensor_.stop_ranging();
}

void VL53L1X::dump_config() {
  ESP_LOGCONFIG(TAG, "VL53L1X:");
  LOG_I2C_DEVICE(this);
  if (this->ranging_mode != nullptr) {
    ESP_LOGCONFIG(TAG, "  Ranging: %s", this->ranging_mode->name);
  }
  if (offset.has_value()) {
    ESP_LOGCONFIG(TAG, "  Offset: %dmm", this->offset.value());
  }
  if (xtalk.has_value()) {
    ESP_LOGCONFIG(TAG, "  XTalk: %dcps", this->xtalk.value());
  }
  LOG_PIN("  Interrupt Pin: ", this->interrupt_pin.value());
  LOG_PIN("  XShut Pin: ", this->xshut_pin.value());
}

void VL53L1X::setup() {
  ESP_LOGD(TAG, "Beginning setup");

  sensors.push_back(this);
  // Bring down other sensors via XSHUT for address changes
  for (auto *s : sensors) {
    if (s != this && s->xshut_pin.has_value()) {
      s->xshut_pin.value()->digital_write(false);
      roode::Roode::log_event("xshut_sensor_" + std::to_string(s->sensor_id_) + "_off");
      roode::Roode::log_event("xshut_toggled_off");
      roode::Roode::log_event("xshut_toggled");
    }
  }

  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLUP);
    this->xshut_pin.value()->setup();
    this->xshut_pin.value()->digital_write(true);
    delay(2);
  }

  auto status = this->init();
  if (status != VL53L1_ERROR_NONE) {
    this->mark_failed();
    return;
  }

  // Restore other sensors
  for (auto *s : sensors) {
    if (s != this && s->xshut_pin.has_value()) {
      s->xshut_pin.value()->digital_write(true);
      delay(2);
    }
  }

  ESP_LOGI(TAG, "Setup complete");
}

VL53L1_Error VL53L1X::init() {
  ESP_LOGD(TAG, "Trying to initialize");

  // Configure low-level driver with current address
  sensor_ = vl53l1x_idf::VL53L1XIDF(I2C_NUM_0, this->address_);

  auto err = sensor_.init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Could not initialize device, error code: %d", err);
    return err;
  }

  // Apply desired I2C address if different
  if (desired_address_ != address_) {
    err = sensor_.set_i2c_address(desired_address_);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to change address. error: %d", err);
      return err;
    }
    address_ = desired_address_;
  }

  // Set default ranging mode
  const RangingMode *mode = ranging_mode_override.value_or(Ranging::Long);
  set_ranging_mode(mode);

  return ESP_OK;
}

VL53L1_Error VL53L1X::wait_for_boot() { return ESP_OK; }

VL53L1_Error VL53L1X::get_device_state(uint8_t *device_state) {
  *device_state = 0x01;  // assume ready
  return ESP_OK;
}

void VL53L1X::set_ranging_mode(const RangingMode *mode) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot set ranging mode while component is failed");
    return;
  }

  auto status = sensor_.set_timing_budget_us(mode->timing_budget * 1000);
  if (status != ESP_OK) {
    ESP_LOGE(TAG, "Could not set timing budget: %d", status);
  }

  status = sensor_.set_intermeasurement_us((mode->delay_between_measurements) * 1000);
  if (status != ESP_OK) {
    ESP_LOGE(TAG, "Could not set measurement delay: %d", status);
  }

  this->ranging_mode = mode;
  ESP_LOGI(TAG, "Set ranging mode: %s", mode->name);
}

optional<uint16_t> VL53L1X::read_distance(ROI *roi, VL53L1_Error &status) {
  if (this->is_failed()) {
    status = ESP_FAIL;
    return {};
  }

  ESP_RETURN_ON_FALSE(roi != nullptr, ESP_ERR_INVALID_ARG, TAG, "ROI is null");

  status = sensor_.set_roi({roi->width, roi->height, roi->center});
  if (status != ESP_OK) {
    ESP_LOGE(TAG, "Could not set ROI, error: %d", status);
    return {};
  }

  status = sensor_.start_ranging();
  if (status != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start ranging, error: %d", status);
    return {};
  }

  bool ready = false;
  auto start_time = millis();
  while (!ready && (millis() - start_time) < this->timeout) {
    sensor_.check_data_ready(ready);
    if (!ready) {
      delay(1);
      App.feed_wdt();
    }
  }

  if (!ready) {
    status = ESP_ERR_TIMEOUT;
    sensor_.stop_ranging();
    return {};
  }

  vl53l1x_idf::Measurement m;
  status = sensor_.read_measurement(m);
  sensor_.clear_interrupt();
  sensor_.stop_ranging();

  if (status != ESP_OK) {
    ESP_LOGE(TAG, "Could not get distance, error: %d", status);
    return {};
  }

  if (m.status != vl53l1x_idf::RangeStatus::RANGE_VALID) {
    ESP_LOGW(TAG, "Range status not valid: %d", static_cast<int>(m.status));
    status = ESP_FAIL;
    return {};
  }

  status = ESP_OK;
  return {m.distance_mm};
}

bool VL53L1X::check_features() { return true; }
bool VL53L1X::validate_interrupt() { return false; }
void VL53L1X::restart() { sensor_.soft_reset(); }
void VL53L1X::soft_reset() { sensor_.soft_reset(); }
void VL53L1X::record_failure() { this->consecutive_failures_++; }

}  // namespace vl53l1x
}  // namespace esphome

