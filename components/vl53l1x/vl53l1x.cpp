#include "vl53l1x.h"
#include "vl53l1x_idf.h"
#include <cstdio>
#include <memory>
#include "esp_check.h"
#include "esphome/core/hal.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#if __has_include("../roode/roode.h")
#define USE_ROODE_LOG 1
#include "../roode/roode.h"
#endif

namespace esphome {
namespace vl53l1x {

std::vector<VL53L1X *> VL53L1X::sensors{};

// Forward declaration for data-ready wait helper
static bool wait_ready(vl53l1x_idf::VL53L1XIDF *sensor, uint32_t timeout_ms);

VL53L1X::VL53L1X() = default;

VL53L1X::~VL53L1X() {
  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);
    ESP_LOGD(TAG, "XShut pin set LOW - powering down sensor");
#ifdef USE_ROODE_LOG
    roode::Roode::log_event("xshut_sensor_" + std::to_string(sensor_id_) + "_off");
    roode::Roode::log_event("xshut_toggled_off");
    roode::Roode::log_event("xshut_toggled");
#endif
  }
  if (sensor_) sensor_->stop_ranging();
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
  ESP_LOGCONFIG(TAG, "  INT active: %s", interrupt_active_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  INT polarity (derived): %s", interrupt_active_low_ ? "active-low" : "active-high");
  ESP_LOGCONFIG(TAG, "  INT miss count: %u", interrupt_miss_count_);
  ESP_LOGCONFIG(TAG, "  Recovery count: %u", recovery_count_);
  ESP_LOGCONFIG(TAG, "  Bus reset count: %u", bus_reset_count_);
}

void VL53L1X::setup() {
  ESP_LOGD(TAG, "Beginning setup");

  sensors.push_back(this);
  coordinated_startup_sequence();

  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLUP);
    this->xshut_pin.value()->setup();
    this->xshut_pin.value()->digital_write(true);
    delay(2);
    ESP_LOGD(TAG, "XSHUT initial state: %d", this->xshut_pin.value()->digital_read());
  }

  if (this->interrupt_pin.has_value()) {
    auto flags = this->interrupt_pin.value()->get_flags();
    // ensure input flag is set
    flags = static_cast<gpio::Flags>(flags | gpio::FLAG_INPUT);
    this->interrupt_pin.value()->pin_mode(flags);
    this->interrupt_pin.value()->setup();
    interrupt_active_low_ = !(flags & gpio::FLAG_PULLDOWN);
    ESP_LOGD(TAG, "INT initial state: %d (flags: 0x%02x, inferred polarity: %s)",
             this->interrupt_pin.value()->digital_read(), static_cast<int>(flags),
             interrupt_active_low_ ? "active-low (pull-up/default)" : "active-high (pull-down)");
    update_interrupt_diagnostic();
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

  // Validate INT if present; otherwise fall back to polling with scheduled retries.
  if (this->interrupt_pin.has_value()) {
    interrupt_active_ = validate_interrupt();
    interrupt_miss_count_ = 0;
    if (!interrupt_active_) {
      ESP_LOGW(TAG, "INT validation failed, using polling and scheduling retry");
      schedule_interrupt_retry();
    } else {
      ESP_LOGI(TAG, "INT validation succeeded");
    }
  }

  if (auto_calibration_enabled_) {
    ESP_LOGI(TAG, "Auto-calibration enabled; scheduling default routine");
    schedule_default_calibration();
  } else {
    ESP_LOGI(TAG, "Auto-calibration disabled for this sensor");
  }

  if (calibration_services_enabled_) {
    this->register_service(&VL53L1X::calibrate_offset_service, "vl53l1x_calibrate_offset");
    this->register_service(&VL53L1X::calibrate_xtalk_service, "vl53l1x_calibrate_xtalk");
  }

  ESP_LOGI(TAG, "Setup complete");
}

VL53L1_Error VL53L1X::init() {
  ESP_LOGD(TAG, "Trying to initialize");

  auto *bus = this->bus_;
  if (bus == nullptr) {
    ESP_LOGE(TAG, "No I2C bus bound to VL53L1X device");
    return ESP_FAIL;
  }
  sensor_ = std::make_unique<vl53l1x_idf::VL53L1XIDF>(bus, this->address_);

  auto err = sensor_->init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Could not initialize device, error code: %d", err);
    return err;
  }

  // Apply desired I2C address if different
  if (desired_address_ != address_) {
    err = sensor_->set_i2c_address(desired_address_);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to change address. error: %d", err);
      return err;
    }
    address_ = desired_address_;
  }

  // Set default ranging mode
  const RangingMode *mode = ranging_mode_override.value_or(Ranging::Long);
  set_ranging_mode(mode);

  apply_calibration_and_thresholds();

  return ESP_OK;
}

VL53L1_Error VL53L1X::wait_for_boot() { return ESP_OK; }

VL53L1_Error VL53L1X::get_device_state(uint8_t *device_state) {
  *device_state = 0x01;  // assume ready
  return ESP_OK;
}

void VL53L1X::apply_calibration_and_thresholds() {
  if (this->offset.has_value()) {
    sensor_->set_offset_mm(this->offset.value());
  }
  if (this->xtalk.has_value()) {
    sensor_->set_xtalk(this->xtalk.value());
  }
  if (this->sigma_threshold_mm.has_value()) {
    sensor_->set_sigma_threshold_mm(this->sigma_threshold_mm.value());
  }
  if (this->signal_threshold_kcps.has_value()) {
    sensor_->set_signal_threshold_cps(this->signal_threshold_kcps.value());
  }
}

void VL53L1X::set_ranging_mode(const RangingMode *mode) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot set ranging mode while component is failed");
    return;
  }

  auto status = sensor_->set_timing_budget_us(mode->timing_budget * 1000);
  if (status != ESP_OK) {
    ESP_LOGE(TAG, "Could not set timing budget: %d", status);
  }

  status = sensor_->set_intermeasurement_us((mode->delay_between_measurements) * 1000);
  if (status != ESP_OK) {
    ESP_LOGE(TAG, "Could not set measurement delay: %d", status);
  }

  this->ranging_mode = mode;
  ESP_LOGI(TAG, "Set ranging mode: %s", mode->name);
}

void VL53L1X::coordinated_startup_sequence() {
  // Pull peers low if they have XSHUT so address changes don't clash
  for (auto *s : sensors) {
    if (s != this && s->xshut_pin.has_value()) {
      s->xshut_pin.value()->digital_write(false);
#ifdef USE_ROODE_LOG
      roode::Roode::log_event("xshut_sensor_" + std::to_string(s->sensor_id_) + "_off");
#endif
    }
  }
  // Our own XSHUT high already set in setup()
}

void VL53L1X::coordinated_bus_reset() {
  bus_reset_count_++;
  for (auto *s : sensors) {
    if (s->xshut_pin.has_value()) s->xshut_pin.value()->digital_write(false);
  }
  delay(5);
  for (auto *s : sensors) {
    if (s->xshut_pin.has_value()) s->xshut_pin.value()->digital_write(true);
  }
  delay(2);
  for (auto *s : sensors) {
    s->init();
    if (s->auto_calibration_enabled_) {
      s->schedule_default_calibration();
    }
  }
}

void VL53L1X::log_reason(const char *reason) {
  ESP_LOGW(TAG, "%s", reason);
#ifdef USE_ROODE_LOG
  roode::Roode::log_event(reason);
#endif
}

void VL53L1X::schedule_timeout_recovery() {
  if (timeout_recovery_scheduled_) return;
  timeout_recovery_scheduled_ = true;

  // small backoff sequence: 0s, 0.5s, 2s -> then bus reset
  std::vector<uint32_t> backoff_ms = {0, 500, 2000};
  auto self = this;
  std::function<void(size_t)> schedule_stage = [&](size_t idx) {
    if (idx >= backoff_ms.size()) {
      // Perform coordinated bus reset if not in cooldown
      if (!bus_reset_cooldown_) {
        ESP_LOGW(TAG, "Executing coordinated bus reset after repeated errors");
        coordinated_bus_reset();
        bus_reset_cooldown_ = true;
        App.scheduler.set_timeout(this, "bus_reset_cooldown", 30 * 60 * 1000, [this]() { bus_reset_cooldown_ = false; });
      } else {
        ESP_LOGW(TAG, "Bus reset cooldown active; skipping bus reset");
      }
      consecutive_timeouts_ = 0;
      consecutive_i2c_errors_ = 0;
      timeout_recovery_scheduled_ = false;
      return;
    }
    uint32_t delay_ms = backoff_ms[idx];
    App.scheduler.set_timeout(self, "timeout_recovery", delay_ms, [self, idx, &schedule_stage]() {
      ESP_LOGW(TAG, "Timeout recovery backoff stage %zu", idx);
      self->restart();
      if (self->consecutive_timeouts_ >= 3 || self->consecutive_i2c_errors_ >= 3) {
        schedule_stage(idx + 1);
      } else {
        self->timeout_recovery_scheduled_ = false;
      }
    });
  };
  schedule_stage(0);
}

optional<uint16_t> VL53L1X::read_distance(ROI *roi, VL53L1_Error &status) {
  if (this->is_failed()) {
    status = ESP_FAIL;
    return {};
  }

  ESP_RETURN_ON_FALSE(roi != nullptr, ESP_ERR_INVALID_ARG, TAG, "ROI is null");

  status = sensor_->set_roi({roi->width, roi->height, roi->center});
  if (status != ESP_OK) {
    ESP_LOGE(TAG, "Could not set ROI, error: %d", status);
    consecutive_i2c_errors_++;
    if (consecutive_i2c_errors_ >= 3) {
      ESP_LOGW(TAG, "Three consecutive I2C errors on set_roi; scheduling recovery");
      schedule_timeout_recovery();
    }
    return {};
  }

  status = sensor_->start_ranging();
  if (status != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start ranging, error: %d", status);
    consecutive_i2c_errors_++;
    if (consecutive_i2c_errors_ >= 3) {
      ESP_LOGW(TAG, "Three consecutive I2C errors on start_ranging; scheduling recovery");
      schedule_timeout_recovery();
    }
    return {};
  }

  bool ready = false;
  auto start_time = millis();
  bool use_int = interrupt_active_ && this->interrupt_pin.has_value();
  bool initial_state = false;
  if (use_int) {
    initial_state = this->interrupt_pin.value()->digital_read();
    update_interrupt_diagnostic();
  }

  // Phase A: small window waiting for INT to reach expected polarity
  auto is_int_active = [&](bool level) {
    return interrupt_active_low_ ? !level : level;
  };

  uint32_t phase_a_ms = std::min<uint32_t>(5, this->timeout / 4);
  while (!ready && (millis() - start_time) < phase_a_ms) {
    if (use_int && is_int_active(this->interrupt_pin.value()->digital_read())) {
      ready = true;
      break;
    }
    delay(1);
    App.feed_wdt();
  }

  // Phase B: fallback to data-ready polling for remaining timeout
  while (!ready && (millis() - start_time) < this->timeout) {
    sensor_->check_data_ready(ready);
    if (!ready) {
      delay(1);
      App.feed_wdt();
    }
  }

  if (!ready) {
    status = ESP_ERR_TIMEOUT;
    sensor_->stop_ranging();
    consecutive_timeouts_++;
    if (this->xshut_pin.has_value()) {
      this->restart();
    }
    if (use_int) {
      interrupt_miss_count_++;
      if (interrupt_miss_count_ >= 3) {
        interrupt_active_ = false;
        interrupt_miss_count_ = 0;
        ESP_LOGW(TAG, "INT missed 3 times; falling back to polling and scheduling retry");
        schedule_interrupt_retry();
      }
    }
    // Escalate to coordinated bus reset if repeated timeouts
    if (consecutive_timeouts_ >= 3) {
      ESP_LOGW(TAG, "Three consecutive timeouts; scheduling bus reset backoff");
      schedule_timeout_recovery();
    }
    return {};
  }

  vl53l1x_idf::Measurement m;
  status = sensor_->read_measurement(m);
  sensor_->clear_interrupt();
  sensor_->stop_ranging();
  update_interrupt_diagnostic();

  if (status != ESP_OK) {
    ESP_LOGE(TAG, "Could not get distance, error: %d", status);
    consecutive_i2c_errors_++;
    if (consecutive_i2c_errors_ >= 3) {
      ESP_LOGW(TAG, "Three consecutive I2C errors on read/clear/stop; scheduling recovery");
      schedule_timeout_recovery();
    }
    return {};
  }

  if (m.status != vl53l1x_idf::RangeStatus::RANGE_VALID) {
    ESP_LOGW(TAG, "Range status not valid: %d", static_cast<int>(m.status));
    status = ESP_FAIL;
    record_failure();
    return {};
  }

  status = ESP_OK;
  consecutive_timeouts_ = 0;
  consecutive_i2c_errors_ = 0;
  return {m.distance_mm};
}

bool VL53L1X::check_features() { return true; }
bool VL53L1X::validate_interrupt() {
  if (!this->interrupt_pin.has_value()) return false;

  bool initial = this->interrupt_pin.value()->digital_read();
  auto status = sensor_->start_ranging();
  if (status != ESP_OK) return false;

  bool ok = false;
  uint32_t start = millis();
  while ((millis() - start) < 25) {  // short validation window ~25ms
    if (interrupt_active_low_ ? !this->interrupt_pin.value()->digital_read()
                              : this->interrupt_pin.value()->digital_read()) {
      ok = true;
      break;
    }
    delay(1);
  }
  sensor_->clear_interrupt();
  sensor_->stop_ranging();
  update_interrupt_diagnostic();
  return ok;
}

void VL53L1X::schedule_interrupt_retry() {
  if (interrupt_retry_scheduled_) return;
  interrupt_retry_scheduled_ = true;
  // retry after 30 minutes
  App.scheduler.set_timeout(this, "int_retry", 30 * 60 * 1000, [this]() {
    interrupt_retry_scheduled_ = false;
    if (this->interrupt_pin.has_value()) {
      bool ok = validate_interrupt();
      if (ok) {
        interrupt_active_ = true;
        interrupt_miss_count_ = 0;
        ESP_LOGI(TAG, "INT recovered after retry");
      } else {
        ESP_LOGW(TAG, "INT retry failed, staying on polling");
        schedule_interrupt_retry();
      }
    }
  });
}
void VL53L1X::restart() {
  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);
    delay(5);
    this->xshut_pin.value()->digital_write(true);
    delay(2);
    this->init();
    if (auto_calibration_enabled_) schedule_default_calibration();
  } else if (sensor_) {
    sensor_->soft_reset();
    if (auto_calibration_enabled_) schedule_default_calibration();
  }
}

void VL53L1X::soft_reset() { restart(); }
void VL53L1X::record_failure() {
  if (++consecutive_failures_ >= 10) {
    restart();
    consecutive_failures_ = 0;
  }
}

void VL53L1X::update_interrupt_diagnostic() {
  if (!interrupt_state_sensor_.has_value() || !this->interrupt_pin.has_value()) return;
  bool level = this->interrupt_pin.value()->digital_read();
  interrupt_state_sensor_.value()->publish_state(level);
}

void VL53L1X::start_auto_cal_async() {
  auto_cal_scheduled_ = false;
  if (auto_cal_done_) {
    ESP_LOGD(TAG, "Auto-calibration already completed; skipping");
    return;
  }
  if (auto_cal_running_) {
    ESP_LOGD(TAG, "Auto-calibration already running; skipping");
    return;
  }
  // Ensure a quick warm measurement before starting; if not ready, defer.
  bool warm_ready = false;
  if (sensor_ != nullptr) {
    auto err = sensor_->start_ranging();
    if (err == ESP_OK) {
      warm_ready = wait_ready(sensor_.get(), 300);
      sensor_->clear_interrupt();
      sensor_->stop_ranging();
    } else {
      ESP_LOGW(TAG, "Auto-cal warm-up start_ranging failed: %d", err);
    }
  }
  if (!warm_ready) {
    auto_cal_warm_failures_++;
    ESP_LOGW(TAG, "Auto-cal warm-up failed %u times; proceeding anyway", auto_cal_warm_failures_);
  } else {
    auto_cal_warm_failures_ = 0;
  }
  if (this->is_failed()) {
    ESP_LOGW(TAG, "Auto-calibration skipped: component is in failed state");
    return;
  }
  auto_cal_running_ = true;
  auto_cal_state_ = {};
  auto_cal_state_.phase = AutoCalPhase::OFFSET_WARM_START;
  ESP_LOGI(TAG, "Auto-calibration: starting (offset@200mm, xtalk@600mm)");
  App.scheduler.set_timeout(this, "auto_cal_step", 0, [this]() { this->auto_cal_step(); });
}

void VL53L1X::auto_cal_step() {
  // If component is failed or sensor missing, abort.
  if (this->is_failed() || sensor_ == nullptr) {
    auto_cal_running_ = false;
    return;
  }

  auto &st = auto_cal_state_;
  switch (st.phase) {
    case AutoCalPhase::OFFSET_WARM_START: {
      sensor_->start_ranging();
      st.deadline_ms = millis() + 400;
      st.phase = AutoCalPhase::OFFSET_WARM_WAIT;
      App.scheduler.set_timeout(this, "auto_cal_step", 5, [this]() { this->auto_cal_step(); });
      break;
    }
    case AutoCalPhase::OFFSET_WARM_WAIT: {
      bool ready = false;
      sensor_->check_data_ready(ready);
      if (ready || millis() > st.deadline_ms) {
        sensor_->clear_interrupt();
        sensor_->stop_ranging();
        st.phase = AutoCalPhase::OFFSET_SAMPLE_START;
        App.scheduler.set_timeout(this, "auto_cal_step", 0, [this]() { this->auto_cal_step(); });
      } else {
        App.scheduler.set_timeout(this, "auto_cal_step", 10, [this]() { this->auto_cal_step(); });
      }
      break;
    }
    case AutoCalPhase::OFFSET_SAMPLE_START: {
      const uint8_t offset_samples = 2;
      if (st.offset_idx >= offset_samples) {
        st.phase = AutoCalPhase::XTALK_SAMPLE_START;
        App.scheduler.set_timeout(this, "auto_cal_step", 0, [this]() { this->auto_cal_step(); });
        break;
      }
      auto err = sensor_->start_ranging();
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Auto-cal offset sample %u start failed: %d", st.offset_idx, err);
        st.offset_idx++;
        App.scheduler.set_timeout(this, "auto_cal_step", 0, [this]() { this->auto_cal_step(); });
        break;
      }
      st.deadline_ms = millis() + 800;
      st.phase = AutoCalPhase::OFFSET_WAIT;
      App.scheduler.set_timeout(this, "auto_cal_step", 10, [this]() { this->auto_cal_step(); });
      break;
    }
    case AutoCalPhase::OFFSET_WAIT: {
      bool ready = false;
      sensor_->check_data_ready(ready);
      if (!ready && millis() <= st.deadline_ms) {
        App.scheduler.set_timeout(this, "auto_cal_step", 10, [this]() { this->auto_cal_step(); });
        break;
      }
      if (!ready) {
        ESP_LOGW(TAG, "Auto-cal offset sample %u timed out", st.offset_idx);
        sensor_->stop_ranging();
        st.offset_idx++;
        st.phase = AutoCalPhase::OFFSET_SAMPLE_START;
        App.scheduler.set_timeout(this, "auto_cal_step", 0, [this]() { this->auto_cal_step(); });
        break;
      }
      vl53l1x_idf::Measurement m;
      auto err = sensor_->read_measurement(m);
      sensor_->clear_interrupt();
      sensor_->stop_ranging();
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Auto-cal offset sample %u read failed: %d", st.offset_idx, err);
      } else {
        int16_t written = static_cast<int16_t>(200) - static_cast<int16_t>(m.distance_mm);
        st.offset_acc += written;
        st.offset_ok++;
        ESP_LOGD(TAG, "Auto-cal offset sample %u ok: raw=%umm written=%d", st.offset_idx, m.distance_mm, written);
      }
      st.offset_idx++;
      st.phase = AutoCalPhase::OFFSET_SAMPLE_START;
      App.scheduler.set_timeout(this, "auto_cal_step", 0, [this]() { this->auto_cal_step(); });
      break;
    }
    case AutoCalPhase::XTALK_SAMPLE_START: {
      const uint8_t xtalk_samples = 3;
      if (st.xtalk_idx >= xtalk_samples) {
        st.phase = AutoCalPhase::DONE;
        App.scheduler.set_timeout(this, "auto_cal_step", 0, [this]() { this->auto_cal_step(); });
        break;
      }
      auto err = sensor_->start_ranging();
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Auto-cal xtalk sample %u start failed: %d", st.xtalk_idx, err);
        st.xtalk_idx++;
        App.scheduler.set_timeout(this, "auto_cal_step", 0, [this]() { this->auto_cal_step(); });
        break;
      }
      st.deadline_ms = millis() + 1000;
      st.phase = AutoCalPhase::XTALK_WAIT;
      App.scheduler.set_timeout(this, "auto_cal_step", 10, [this]() { this->auto_cal_step(); });
      break;
    }
    case AutoCalPhase::XTALK_WAIT: {
      bool ready = false;
      sensor_->check_data_ready(ready);
      if (!ready && millis() <= st.deadline_ms) {
        App.scheduler.set_timeout(this, "auto_cal_step", 10, [this]() { this->auto_cal_step(); });
        break;
      }
      if (!ready) {
        sensor_->stop_ranging();
        ESP_LOGW(TAG, "Auto-cal xtalk sample %u timed out", st.xtalk_idx);
        st.xtalk_idx++;
        st.phase = AutoCalPhase::XTALK_SAMPLE_START;
        App.scheduler.set_timeout(this, "auto_cal_step", 0, [this]() { this->auto_cal_step(); });
        break;
      }
      vl53l1x_idf::Measurement m;
      auto err = sensor_->read_measurement(m);
      sensor_->clear_interrupt();
      sensor_->stop_ranging();
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Auto-cal xtalk sample %u read failed: %d", st.xtalk_idx, err);
      } else {
        uint32_t cps = m.signal_rate_mcps * 1000u;
        st.xtalk_acc += cps;
        st.xtalk_ok++;
        ESP_LOGD(TAG, "Auto-cal xtalk sample %u ok: cps=%u", st.xtalk_idx, cps);
      }
      st.xtalk_idx++;
      st.phase = AutoCalPhase::XTALK_SAMPLE_START;
      App.scheduler.set_timeout(this, "auto_cal_step", 0, [this]() { this->auto_cal_step(); });
      break;
    }
    case AutoCalPhase::DONE: {
      // Apply results if any
      if (st.offset_ok > 0) {
        int16_t result = static_cast<int16_t>(st.offset_acc / st.offset_ok);
        sensor_->set_offset_mm(result);
        this->offset = result;
        ESP_LOGI(TAG, "Auto-calibration offset complete: %dmm", result);
      } else {
        ESP_LOGW(TAG, "Auto-calibration offset produced no valid samples");
      }
      if (st.xtalk_ok > 0) {
        uint16_t result = static_cast<uint16_t>(st.xtalk_acc / st.xtalk_ok);
        sensor_->set_xtalk(result);
        this->xtalk = result;
        ESP_LOGI(TAG, "Auto-calibration xtalk complete: %ucps", result);
      } else {
        ESP_LOGW(TAG, "Auto-calibration xtalk produced no valid samples");
      }
      auto_cal_running_ = false;
      bool success = (st.offset_ok > 0 && st.xtalk_ok > 0);
      auto_cal_done_ = success;
      if (!success && auto_cal_retries_ < 2) {
        auto_cal_retry_pending_ = true;
        auto_cal_retries_++;
        ESP_LOGW(TAG, "Auto-cal failed; scheduling retry #%u in 5s", auto_cal_retries_);
        App.scheduler.set_timeout(this, "auto_cal_retry", 5000, [this]() {
          auto_cal_retry_pending_ = false;
          this->start_auto_cal_async();
        });
      } else if (!success) {
        ESP_LOGW(TAG, "Auto-cal failed after retries; giving up for this boot");
      }
      st.phase = AutoCalPhase::IDLE;
      break;
    }
    case AutoCalPhase::IDLE:
    default:
      auto_cal_running_ = false;
      break;
  }
}
void VL53L1X::schedule_default_calibration() {
  if (auto_cal_scheduled_) return;
  auto_cal_scheduled_ = true;
  // Run shortly after init/restart to avoid blocking setup.
  App.scheduler.set_timeout(this, "auto_cal", auto_cal_delay_ms_, [this]() { this->start_auto_cal_async(); });
}

void VL53L1X::run_default_calibration() {}  // unused now (kept for compatibility)

// Simple helper to wait for data-ready with logging and a given timeout.
static bool wait_ready(vl53l1x_idf::VL53L1XIDF *sensor, uint32_t timeout_ms) {
  bool ready = false;
  uint32_t start = millis();
  while (!ready && (millis() - start) < timeout_ms) {
    sensor->check_data_ready(ready);
    if (!ready) {
      delay(5);
      App.feed_wdt();
    }
  }
  return ready;
}

VL53L1_Error VL53L1X::calibrate_offset_runtime(uint16_t target_distance_mm, uint8_t samples, int16_t &result_mm) {
  if (samples == 0) return ESP_ERR_INVALID_ARG;
  int32_t acc = 0;
  uint8_t ok = 0;
  // Warm-up measurement to wake the device and settle timing
  sensor_->start_ranging();
  wait_ready(sensor_.get(), 400);
  sensor_->clear_interrupt();
  sensor_->stop_ranging();

  for (uint8_t i = 0; i < samples; i++) {
    uint16_t written = 0;
    auto err = sensor_->calibrate_offset_once(target_distance_mm, written);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Offset calibration sample %u failed: %d", i, err);
      continue;
    }
    acc += static_cast<int16_t>(written);
    ok++;
    delay(5);
    App.feed_wdt();
  }
  if (ok == 0) return ESP_FAIL;
  result_mm = static_cast<int16_t>(acc / ok);
  this->offset = result_mm;
  ESP_LOGI(TAG, "Offset calibrated: %dmm (samples ok=%u)", result_mm, ok);
  return sensor_->set_offset_mm(result_mm);
}

VL53L1_Error VL53L1X::calibrate_xtalk_runtime(uint16_t target_distance_mm, uint8_t samples, uint16_t &result_cps) {
  if (samples == 0) return ESP_ERR_INVALID_ARG;
  uint32_t acc = 0;
  uint8_t ok = 0;
  // Use a slightly longer timing budget for better stability during calibration
  const RangingMode *saved_mode = this->ranging_mode;
  if (saved_mode != nullptr) {
    sensor_->set_timing_budget_us(std::max<uint32_t>(20000, saved_mode->timing_budget * 1000));
  }
  for (uint8_t i = 0; i < samples; i++) {
    auto err = sensor_->start_ranging();
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Xtalk start failed: %d", err);
      continue;
    }
    bool ready = wait_ready(sensor_.get(), 800);
    if (!ready) {
      sensor_->stop_ranging();
      ESP_LOGW(TAG, "Xtalk sample %u timed out - retrying once after reset", i);
      // Attempt one restart then retry wait once
      this->restart();
      err = sensor_->start_ranging();
      if (err != ESP_OK) continue;
      ready = wait_ready(sensor_.get(), 800);
      if (!ready) {
        sensor_->stop_ranging();
        ESP_LOGW(TAG, "Xtalk sample %u timed out after retry", i);
        continue;
      }
    }
    vl53l1x_idf::Measurement m;
    err = sensor_->read_measurement(m);
    sensor_->clear_interrupt();
    sensor_->stop_ranging();
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Xtalk sample %u read failed: %d", i, err);
      continue;
    }
    // Expect low signal in dark/coverglass; use signal rate to compute cps
    uint32_t cps = m.signal_rate_mcps * 1000u;  // mcps -> cps
    acc += cps;
    ok++;
    delay(5);
    App.feed_wdt();
  }
  if (ok == 0) return ESP_FAIL;
  result_cps = static_cast<uint16_t>(acc / ok);
  this->xtalk = result_cps;
  ESP_LOGI(TAG, "Xtalk calibrated: %ucps (samples ok=%u)", result_cps, ok);
  return sensor_->set_xtalk(result_cps);
}

void VL53L1X::calibrate_offset_service() {
  int distance_mm = 200;  // default target
  int16_t result = 0;
  auto err = calibrate_offset_runtime(distance_mm, 3, result);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Offset calibration service complete: %dmm", result);
  } else {
    ESP_LOGW(TAG, "Offset calibration service failed: %d", err);
  }
}

void VL53L1X::calibrate_xtalk_service() {
  int distance_mm = 600;  // default distance for xtalk target
  uint16_t result = 0;
  auto err = calibrate_xtalk_runtime(distance_mm, 5, result);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Xtalk calibration service complete: %ucps", result);
  } else {
    ESP_LOGW(TAG, "Xtalk calibration service failed: %d", err);
  }
}

void VL53L1X::calibrate_offset() {
  int16_t result = 0;
  auto err = calibrate_offset_runtime(200, 3, result);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Manual offset calibration complete: %dmm", result);
  } else {
    ESP_LOGW(TAG, "Manual offset calibration failed: %d", err);
  }
}

void VL53L1X::calibrate_xtalk() {
  uint16_t result = 0;
  auto err = calibrate_xtalk_runtime(600, 5, result);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Manual xtalk calibration complete: %ucps", result);
  } else {
    ESP_LOGW(TAG, "Manual xtalk calibration failed: %d", err);
  }
}

}  // namespace vl53l1x
}  // namespace esphome
