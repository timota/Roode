#include "vl53l1x.h"
#include <cstdio>

namespace esphome {
namespace vl53l1x {

std::vector<VL53L1X *> VL53L1X::sensors{};
VL53L1X *VL53L1X::active_sensor_{nullptr};

VL53L1X::~VL53L1X() {
  if (get_active_sensor() == this) {
    set_active_sensor(nullptr);
  }
  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);
    ESP_LOGD(TAG, "XShut pin set LOW - powering down sensor");
  }
  this->sensor.StopRanging();
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
  set_active_sensor(this);

  // Prepare calibration storage unique per sensor_id
  cal_pref_ = global_preferences->make_preference<CalibrationData>(0x5300 + this->sensor_id_);

  sensors.push_back(this);
  for (auto *s : sensors) {
    if (s != this && s->xshut_pin.has_value()) {
      s->xshut_pin.value()->digital_write(false);
      ESP_LOGI(TAG, "XSHUT: temporarily powering off sensor %u while %u initializes", s->sensor_id_,
               this->sensor_id_);
      ESP_LOGD(TAG, "XSHUT toggled off");
    }
  }

  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLUP);
    this->xshut_pin.value()->setup();
    ESP_LOGD(TAG, "XShut pin configured");
    this->xshut_pin.value()->digital_write(true);
    ESP_LOGD(TAG, "XShut pin set HIGH - sensor powered on");
    ESP_LOGD(TAG, "XSHUT toggled on");
    delay(2);
  }

  if (this->interrupt_pin.has_value()) {
    // Active level inferred from config (set_interrupt_active_high). Apply mode as provided by pin setup.
    this->interrupt_pin.value()->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
    this->interrupt_pin.value()->setup();
    ESP_LOGD(TAG, "Interrupt pin configured (active=%s)", inferred_active_high_ ? "HIGH" : "LOW");
  }

  auto status = this->init();
  if (status != VL53L1_ERROR_NONE) {
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "Device initialized");

  // Load stored calibration if present before applying overrides
  if (this->load_calibration()) {
    ESP_LOGI(TAG, "Applied stored calibration: offset=%dmm xtalk=%ucps", this->offset.value_or(0),
             this->xtalk.value_or(0));
  }
  if (desired_address_ != 0x29) {
    status = this->sensor.SetI2CAddress(desired_address_ << 1);
    if (status == VL53L1_ERROR_NONE) {
      char buf[5];
      snprintf(buf, sizeof(buf), "%02X", desired_address_);
      ESP_LOGI(TAG, "Sensor %u I2C address changed to 0x%s", sensor_id_, buf);
    } else {
      ESP_LOGE(TAG, "Failed to change address. Error: %d", status);
    }
  }

  if (this->offset.has_value()) {
    ESP_LOGI(TAG, "Setting offset calibration to %d", this->offset.value());
    status = this->sensor.SetOffsetInMm(this->offset.value());
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set offset calibration, error code: %d", status);
      this->mark_failed();
      return;
    }
  }

  if (this->xtalk.has_value()) {
    ESP_LOGI(TAG, "Setting crosstalk calibration to %d", this->xtalk.value());
    status = this->sensor.SetXTalk(this->xtalk.value());
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set crosstalk calibration, error code: %d", status);
      this->mark_failed();
      return;
    }
  }

  if (!this->check_features()) {
    ESP_LOGE(TAG, "Feature check failed. Sensor disabled");
    return;
  }

  for (auto *s : sensors) {
    if (s != this && s->xshut_pin.has_value()) {
      s->xshut_pin.value()->digital_write(true);
      delay(2);
      ESP_LOGI(TAG, "XSHUT: restoring power to sensor %u after %u init", s->sensor_id_, this->sensor_id_);
      ESP_LOGD(TAG, "XSHUT toggled on");
    }
  }

  ESP_LOGI(TAG, "Setup complete");
}

VL53L1_Error VL53L1X::init() {
  ESP_LOGD(TAG, "Trying to initialize");

  VL53L1_Error status;

  // If address is non-default, set and try again.
  if (address_ != (sensor.GetI2CAddress() >> 1)) {
    ESP_LOGD(TAG, "Setting different address");
    status = sensor.SetI2CAddress(address_ << 1);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Failed to change address. Error: %d", status);
      return status;
    }
  }

  status = wait_for_boot();
  if (status != VL53L1_ERROR_NONE) {
    return status;
  }

  ESP_LOGD(TAG, "Found device, initializing...");
  status = sensor.Init();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not initialize device, error code: %d", status);
    return status;
  }

  return status;
}

VL53L1_Error VL53L1X::reinitialize_after_reset() {
  auto status = wait_for_boot();
  if (status != VL53L1_ERROR_NONE) {
    return status;
  }

  status = this->sensor.Init();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not reinitialize device after reset, error code: %d", status);
    return status;
  }

  if (this->offset.has_value()) {
    status = this->sensor.SetOffsetInMm(this->offset.value());
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGW(TAG, "Could not restore offset after reset, error code: %d", status);
    }
  }

  if (this->xtalk.has_value()) {
    status = this->sensor.SetXTalk(this->xtalk.value());
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGW(TAG, "Could not restore xtalk after reset, error code: %d", status);
    }
  }

  if (this->ranging_mode != nullptr) {
    status = this->sensor.SetDistanceMode(this->ranging_mode->mode);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGW(TAG, "Could not restore distance mode after reset, error code: %d", status);
    }
    status = this->sensor.SetTimingBudgetInMs(this->ranging_mode->timing_budget);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGW(TAG, "Could not restore timing budget after reset, error code: %d", status);
    }
    status = this->sensor.SetInterMeasurementInMs(this->ranging_mode->delay_between_measurements);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGW(TAG, "Could not restore inter-measurement after reset, error code: %d", status);
    }
  }

  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1X::wait_for_boot() {
  // Wait for firmware to copy NVM device_state into registers
  delayMicroseconds(1200);

  uint8_t device_state;
  VL53L1_Error status;
  auto start = millis();
  while ((millis() - start) < this->timeout) {
    status = get_device_state(&device_state);
    if (status != VL53L1_ERROR_NONE) {
      return status;
    }
    if ((device_state & 0x01) == 0x01) {
      ESP_LOGD(TAG, "Finished waiting for boot. Device state: %d", device_state);
      return VL53L1_ERROR_NONE;
    }
    App.feed_wdt();
  }

  ESP_LOGW(TAG, "Timed out waiting for boot. state: %d", device_state);
  return VL53L1_ERROR_TIME_OUT;
}

VL53L1_Error VL53L1X::get_device_state(uint8_t *device_state) {
  VL53L1_Error status = sensor.GetBootState(device_state);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Failed to read device state. error: %d", status);
    return status;
  }

  // Our own logic...device_state is 255 when unable to complete read
  // Not sure why and why other libraries don't account for this.
  // Maybe somehow this is supposed to be 0, and it is getting messed up in I2C layer.
  if (*device_state == 255) {
    *device_state = 98;  // Unknown
  }

  ESP_LOGV(TAG, "Device state: %d", *device_state);

  return VL53L1_ERROR_NONE;
}

void VL53L1X::set_ranging_mode(const RangingMode *mode) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot set ranging mode while component is failed");
    return;
  }

  auto status = this->sensor.SetDistanceMode(mode->mode);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not set distance mode: %d, error code: %d", mode->mode, status);
  }

  status = this->sensor.SetTimingBudgetInMs(mode->timing_budget);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not set timing budget: %d ms, error code: %d", mode->timing_budget, status);
  }

  status = this->sensor.SetInterMeasurementInMs(mode->delay_between_measurements);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not set measurement delay: %d ms, error code: %d", mode->delay_between_measurements, status);
  }

  this->ranging_mode = mode;
  ESP_LOGI(TAG, "Set ranging mode: %s", mode->name);
}

optional<uint16_t> VL53L1X::read_distance(ROI *roi, VL53L1_Error &status) {
  if (this->is_failed()) {
    ESP_LOGW(TAG, "Cannot read distance while component is failed");
    record_failure();
    return {};
  }

  ESP_LOGVV(TAG, "Beginning distance read");

  if (last_roi == nullptr || *roi != *last_roi) {
    ESP_LOGVV(TAG, "Setting new ROI: { width: %d, height: %d, center: %d }", roi->width, roi->height, roi->center);

    status = this->sensor.SetROI(roi->width, roi->height);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set ROI width/height, error code: %d", status);
      record_failure();
      return {};
    }
    status = this->sensor.SetROICenter(roi->center);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set ROI center, error code: %d", status);
      record_failure();
      return {};
    }
    last_roi = roi;
  }

  // Decide whether we can use the interrupt pin for this reading
  uint8_t dataReady = false;
  bool use_int = is_interrupt_enabled();
  if (!use_int && this->interrupt_pin.has_value() &&
      (millis() - last_interrupt_retry_ >= 1800000UL)) {
    if (validate_interrupt()) {
      interrupt_active_ = true;
      interrupt_miss_count_ = 0;
      use_int = true;
    } else {
      last_interrupt_retry_ = millis();
    }
  }

  status = this->sensor.StartRanging();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Failed to start ranging, error code: %d", status);
    record_failure();
    return {};
  }

  // Wait for measurement ready using interrupt pin when available
  bool initial_state = false;
  if (use_int) {
    initial_state = this->interrupt_pin.value()->digital_read();
  }
  auto start_time = millis();
  while (!dataReady && (millis() - start_time) < this->timeout) {
    if (use_int) {
      if (this->interrupt_pin.value()->digital_read() != initial_state) {
        dataReady = true;
      }
    } else {
      status = this->sensor.CheckForDataReady(&dataReady);
      if (status != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to check if data is ready, error code: %d", status);
        record_failure();
        return {};
      }
    }
    delay(1);
    App.feed_wdt();
  }
  if (use_int && !dataReady) {
    interrupt_miss_count_++;
    if (interrupt_miss_count_ >= 5) {
      interrupt_active_ = false;
      last_interrupt_retry_ = millis();
    } else {
    }
    // Fallback to polling for this measurement
    start_time = millis();
    while (!dataReady && (millis() - start_time) < this->timeout) {
      status = this->sensor.CheckForDataReady(&dataReady);
      if (status != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to check if data is ready, error code: %d", status);
        record_failure();
        return {};
      }
      delay(1);
      App.feed_wdt();
    }
  }
  if (!dataReady) {
    ESP_LOGW(TAG, "Timed out waiting for measurement ready");
    status = VL53L1_ERROR_TIME_OUT;
    this->sensor.StopRanging();
    soft_reset();
    record_failure();
    return {};
  }

  // Get the results
  uint16_t distance;
  status = this->sensor.GetDistanceInMm(&distance);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not get distance, error code: %d", status);
    record_failure();
    return {};
  }

  // After reading the results reset the interrupt to be able to take another measurement
  status = this->sensor.ClearInterrupt();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not clear interrupt, error code: %d", status);
    record_failure();
    return {};
  }
  status = this->sensor.StopRanging();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not stop ranging, error code: %d", status);
    record_failure();
    return {};
  }

  if (use_int)
    interrupt_miss_count_ = 0;

  ESP_LOGV(TAG, "Finished distance read: %d", distance);
  consecutive_failures_ = 0;
  recovery_window_count_ = 0;
  next_recovery_allowed_ = 0;
  backoff_ms_ = BASE_BACKOFF_MS;
  return {distance};
}

bool VL53L1X::check_features() {
  ESP_LOGI(TAG, "Validating optional pins");
  bool xshut_ok = false;
  bool int_ok = false;

  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);
    ESP_LOGD(TAG, "XShut pin set LOW - validating pin");
    delay(10);
    this->xshut_pin.value()->digital_write(true);
    ESP_LOGD(TAG, "XShut pin set HIGH - validation reset complete");
    xshut_ok = (this->reinitialize_after_reset() == VL53L1_ERROR_NONE);
    if (!xshut_ok) {
      ESP_LOGE(TAG, "XShut pin validation failed, disabling power cycle support");
      this->xshut_pin.reset();
      ESP_LOGW(TAG, "XShut pin disabled due to validation failure");
      if (this->wait_for_boot() != VL53L1_ERROR_NONE) {
        this->mark_failed();
        return false;
      }
    } else {
      ESP_LOGI(TAG, "XShut pin working");
    }
  }

  if (!this->xshut_pin.has_value()) {
    ESP_LOGI(TAG, "XShut disabled");
  }

  if (this->interrupt_pin.has_value()) {
    int_ok = validate_interrupt();
    if (!int_ok) {
      ESP_LOGW(TAG, "Interrupt validation failed, falling back to polling");
      interrupt_active_ = false;
      interrupt_miss_count_ = 0;
      last_interrupt_retry_ = millis();
    } else {
      ESP_LOGI(TAG, "Interrupt pin working; using INT mode");
      interrupt_active_ = true;
      interrupt_miss_count_ = 0;
    }
  } else {
    interrupt_active_ = false;
  }

  if (!this->interrupt_pin.has_value()) {
    ESP_LOGI(TAG, "Interrupt disabled");
  }

  if (this->xshut_pin.has_value()) {
    ESP_LOGI(TAG, "XShut %s", xshut_ok ? "working" : "disabled");
  }
  if (this->interrupt_pin.has_value()) {
    ESP_LOGI(TAG, "Interrupt %s", int_ok ? "working" : "disabled");
  }

  return !this->is_failed();
}

bool VL53L1X::validate_interrupt() {
  bool ok = false;
  if (!this->interrupt_pin.has_value())
    return false;
  bool active_level = inferred_active_high_ ? true : false;
  bool initial = this->interrupt_pin.value()->digital_read();
  ESP_LOGD(TAG, "Interrupt pin initial state: %d (active=%d)", initial, active_level);
  auto status = this->sensor.StartRanging();
  if (status == VL53L1_ERROR_NONE) {
    auto start = millis();
    while ((millis() - start) < this->timeout) {
      bool level = this->interrupt_pin.value()->digital_read();
      if (is_int_active_level(level)) {
        ESP_LOGD(TAG, "Interrupt pin at active level - measurement ready");
        ok = true;
        break;
      }
      App.feed_wdt();
    }
    if (!ok)
      ESP_LOGD(TAG, "Interrupt pin did not change state during validation");
    this->sensor.ClearInterrupt();
    this->sensor.StopRanging();
  }
  return ok;
}

bool VL53L1X::is_int_active_level(bool level) const { return inferred_active_high_ ? level : !level; }

void VL53L1X::restart() {
  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);
    ESP_LOGW(TAG, "XShut pin set LOW - restarting sensor");
    delay(100);
    this->xshut_pin.value()->digital_write(true);
    ESP_LOGD(TAG, "XShut pin set HIGH - restart complete");
    this->reinitialize_after_reset();
    recovery_count_++;
  } else {
    ESP_LOGW(TAG, "Restarting sensor without XSHUT pin");
    this->init();
  }
}

bool VL53L1X::save_calibration(int16_t offset_mm, uint16_t xtalk_cps) {
  CalibrationData data{offset_mm, xtalk_cps, 0xC411BEEF};
  bool ok = cal_pref_.save(&data);
  ESP_LOGI(TAG, ok ? "Saved calibration: offset=%dmm xtalk=%ucps" : "Failed to save calibration", offset_mm,
           xtalk_cps);
  return ok;
}

bool VL53L1X::load_calibration() {
  CalibrationData data{};
  if (cal_pref_.load(&data) && data.magic == 0xC411BEEF) {
    if (!this->offset.has_value())
      this->offset = data.offset_mm;
    if (!this->xtalk.has_value())
      this->xtalk = data.xtalk_cps;
    cal_loaded_ = true;
    return true;
  }
  return false;
}

bool VL53L1X::calibrate_and_store(uint16_t offset_target_mm, uint16_t xtalk_target_mm) {
  ESP_LOGI(TAG, "Starting VL53L1X calibration (offset %umm, xtalk %umm)", offset_target_mm, xtalk_target_mm);

  // Ensure sensor is idle
  this->sensor.StopRanging();

  int16_t found_offset = 0;
  uint16_t found_xtalk = 0;

  auto status = this->sensor.CalibrateOffset(offset_target_mm, &found_offset);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Offset calibration failed: %d", status);
    return false;
  }

  status = this->sensor.CalibrateXTalk(xtalk_target_mm, &found_xtalk);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "XTalk calibration failed: %d", status);
    return false;
  }

  // Apply and persist
  this->sensor.SetOffsetInMm(found_offset);
  this->sensor.SetXTalk(found_xtalk);
  this->offset = found_offset;
  this->xtalk = found_xtalk;
  this->save_calibration(found_offset, found_xtalk);

  ESP_LOGI(TAG, "Calibration complete: offset=%dmm xtalk=%ucps", found_offset, found_xtalk);
  return true;
}

void VL53L1X::soft_reset() {

  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);

    ESP_LOGW(TAG, "XShut pin set LOW - resetting sensor");

    delay(100);
    this->xshut_pin.value()->digital_write(true);

    ESP_LOGD(TAG, "XShut pin set HIGH - reset complete");

    this->reinitialize_after_reset();
    recovery_count_++;
  } else {
    ESP_LOGW(TAG, "Restarting sensor without XSHUT pin");
    this->init();
  }
}


void VL53L1X::record_failure() {
  if (++consecutive_failures_ < FAILURE_THRESHOLD)
    return;

  uint32_t now = millis();

  // Window tracking
  if (recovery_window_start_ == 0 || (now - recovery_window_start_) > RECOVERY_WINDOW_MS) {
    recovery_window_start_ = now;
    recovery_window_count_ = 0;
  }

  if (now < next_recovery_allowed_) {
    ESP_LOGW(TAG, "Recovery cooldown active; skipping reset (next in %ums)", next_recovery_allowed_ - now);
    consecutive_failures_ = 0;
    return;
  }

  if (recovery_window_count_ >= MAX_RECOVERIES_PER_WINDOW) {
    ESP_LOGW(TAG, "Recovery limit reached (%u in %us); marking sensor failed until window resets",
             MAX_RECOVERIES_PER_WINDOW, RECOVERY_WINDOW_MS / 1000);
    this->mark_failed();
    consecutive_failures_ = 0;
    return;
  }

  ESP_LOGW(TAG, "Triggering XSHUT recovery (failures=%u)", consecutive_failures_);
  soft_reset();
  recovery_window_count_++;
  consecutive_failures_ = 0;

  backoff_ms_ = std::min<uint32_t>(MAX_BACKOFF_MS, backoff_ms_ * 2);
  next_recovery_allowed_ = now + backoff_ms_;
}


}  // namespace vl53l1x
}  // namespace esphome
