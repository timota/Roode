# Roode Project – ESPHome Driver Review (Analysis Only)

Scope: repository on branch `feat/driver-spike`; ESPHome + Roode designed originally for Arduino framework. Goal: polish for performance, reliability, maintainability under ESPHome (esp-idf target). No code changes in this document.

## Components
- VL53L1X driver (current Arduino-style ULD wrapper, partially migrated).
- Roode component (people counter logic).
- persisted_number (aux storage).

## Current State (observed)
- Build: `esphome compile esphome.yaml` currently fails due to ULD include paths and remaining Arduino-era logging (`roode::Roode::log_event`) in VL53L1X driver.
- Code split: VL53L1X driver is still Arduino-patterned (blocking, uses ULD C API), not integrated with ESPHome I2C abstractions; platform layer stubbed.
- Calibration/auto-cal: multiple attempts in prior spike; unstable at boot; timeouts and retries; timing not aligned with ULD recommendations.
- Logging: mixed Roode-specific logging in driver (tight coupling); noisy and causes unresolved symbols when Roode header absent.
- ULD sources: now copied into `components/vl53l1x/` but include paths need normalization; platform layer still Arduino-based.
- Config: `esphome.yaml` minimal test config (esp-idf, I2C pins, XSHUT/INT).

## Feature/Behavior Inventory (Arduino baseline)
- Init/boot: wait_for_boot, optional I2C address change, init via ULD.
- Ranging mode: distance mode, timing budget, intermeasurement (via RangingMode table).
- ROI: per-read ROI set, cached to avoid redundant writes.
- Data-ready: prefer INT pin if validated; fallback to polling; miss counters and retry to polling; periodic re-validation.
- Timeout handling: on timeout, power-cycle via XSHUT (if present), re-wait boot, increment recovery count.
- XSHUT sequencing: multi-sensor coordination (pull peers low during init/address change, restore after).
- Calibration apply: offset and xtalk static values applied at setup.
- Error tracking: consecutive failure counter, mark_failed.

## Gaps / Issues
- ULD platform layer still Arduino (Wire/Arduino.h); stubs exist but need real ESPHome I2C bindings or move to IDF/ESPHome-native driver (recommended).
- Roode logging hard-coupled in VL53L1X driver; breaks standalone build and adds noise.
- Auto-calibration logic unstable (timeouts on boot); timing windows and trigger ordering need redesign.
- Include path churn: flattened ULD sources but headers still reference ../platform paths; build fails.
- Duplicate driver implementations (old Arduino vs new spike); need a clean ESPHome-native version and to drop Arduino.
- No persistence of calibration values; recalibration always runs (and often fails).
- Testing configs: only minimal `esphome.yaml`; integration configs (peopleCounter*.yaml) not verified for esp-idf.

## Proposed Improvements (no implementation yet)
1) Choose one driver path: ESPHome-native IDF driver (preferred). Remove Arduino-era ULD platform or isolate it behind a clean interface.
2) Rebuild platform layer: implement I2C ops using `esphome::i2c::I2CDevice` (write/read multi, byte/word/dword); remove Wire/Arduino.h completely.
3) Decouple logging: remove Roode-specific logging from VL53L1X; use ESPHome logger only; expose hooks/counters via diagnostics if needed.
4) Normalize ULD includes: flatten includes to local headers or adjust include dirs so build works; prune unused ULD pieces if replaced by native code.
5) Auto-calibration redesign: single late on_boot trigger, generous ULD-aligned timeouts (2–3s per sample), optional warm-up, one retry max, non-blocking chunks; skip if stored cal exists.
6) Calibration persistence: store offset/xtalk in preferences; only recalibrate on request or missing data; add manual service/button.
7) XSHUT/INT handling: retain multi-sensor sequencing; ensure INT polarity/pull configuration is explicit and tested; fallback to polling robustly.
8) Error handling: keep mark_failed and recovery counters; ensure recoveries don’t spam logs; add diagnostic sensors if desired.
9) Build hygiene: ensure `esphome compile esphome.yaml` passes under esp-idf; remove unused Arduino framework configs; keep platformio options minimal.
10) Documentation: update README/plan with esp-idf-only stance, calibration behavior, INT/XSHUT expectations, and sample YAML.

## Next Steps (when work resumes)
- Fix build first: remove Roode log calls from VL53L1X; adjust ULD include paths; decide on platform layer approach.
- Implement ESPHome I2C platform bindings or port to IDF-native driver; then drop ULD Arduino remnants.
- Redesign auto-cal per proposal and add calibration persistence.
- Validate with `esphome compile` and on hardware; iterate logging/noise.
