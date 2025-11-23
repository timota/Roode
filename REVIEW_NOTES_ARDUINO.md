# Roode Project Review (Arduino-based ESPHome stack) – Analysis Only

Purpose: identify improvements for performance, reliability, and maintainability while keeping current Arduino-framework behaviour intact. No code changes made; this is a planning document.

## Components
- VL53L1X driver (Arduino-style, uses ULD; interrupt + XSHUT support; multi-sensor coordination).
- Roode component (people counter logic, zones/ROI, thresholds, calibration flows).
- persisted_number (storage helper).

## Observed strengths
- Multi-zone ROI handling already integrated with Roode logic.
- XSHUT + INT wiring supported for multi-sensor setups.
- Calibration (offset/xtalk) supported via configuration and applied at setup.
- Recovery logic (XSHUT pulse, retries) present.

## Issues / opportunities
1) Driver–Roode coupling: VL53L1X driver logs via `roode::Roode::log_event`, creating cross-component dependency and noisy logs.  
2) Blocking operations: synchronous ranging/timeout loops can block the main loop; large delays during auto-cal or timeout recovery.  
3) Interrupt handling robustness: INT validation/miss handling could be simplified; polarity/pull configuration should be explicit.  
4) Calibration stability: auto-cal timing windows and triggers are ad hoc; lack of calibration persistence (offset/xtalk) leads to repeated calibration.  
5) ULD platform layer: Arduino Wire-based platform mixed with ESPHome abstractions; could be clarified or wrapped cleanly.  
6) Error handling: `record_failure`/timeouts may trigger repeated resets; need clearer thresholds and backoff.  
7) Configuration clarity: YAML options for pins, timing, calibration not fully documented; legacy options may be unused.  
8) Multi-sensor sequencing: peer XSHUT coordination exists but is spread across code; could be encapsulated.  
9) Testing/build hygiene: ensure `esphome compile` passes for Arduino framework; avoid unused libraries and stale includes.  
10) Persisted calibration/thresholds: persisted_number exists but calibration persistence not clearly integrated.

## Proposed improvements (keep behaviour, polish)
- Decouple logging: remove direct Roode logging from the driver; use ESPHome logger and optional diagnostic sensors/counters.  
- Clarify platform layer: keep Arduino Wire code but isolate it behind a small adapter used by ULD; avoid mixing with ESPHome internals.  
- Non-blocking reads: bound per-iteration delays and feed WDT; consider short cooperative delays in long loops (timeouts, auto-cal).  
- INT handling: make polarity/pull explicit in config; validate once, fallback to polling with periodic re-validate; reduce log spam.  
- Calibration: add optional persistence for offset/xtalk; provide a manual service/button; run auto-cal only when no stored data; use consistent timing windows. Keep Roode zone recalibration separate (it adjusts entry/exit thresholds), but add a dedicated VL53L1X-level calibration trigger to avoid reusing `roode_platform->recalibration()` for sensor offset/xtalk.
  - Implemented: driver now persists offset/xtalk via ESPHome preferences and exposes `calibrate_and_store()` for manual triggers. Defaults: apply stored values first; auto-cal only when nothing stored or when explicitly triggered. (Sample button can be added as needed.)
- Recovery/backoff: tighten thresholds for XSHUT resets and consecutive failures; add cooldown to avoid rapid cycling.  
- Multi-sensor coordination: encapsulate XSHUT peer sequencing in helper functions; document ordering.  
- YAML/documentation: refresh sample configs (pins, timing budgets, calibration options); prune deprecated options.  
- Build hygiene: verify Arduino-framework `esphome compile` passes; remove unused ULD files/includes; ensure library deps declared once.  
- Metrics/diagnostics: expose optional diagnostic sensors (INT level, recovery counts) without coupling to Roode.

## Status updates
- Logging decoupled from VL53L1X driver (Roode log calls removed; ESPHome logger only).
- Build hygiene pass: Arduino `esphome compile esphome.yaml` now succeeds with current driver/library layout; remaining hygiene items (platform adapter cleanup, unused files) to revisit later.

## Next steps (when ready to implement)
1) Clean build under Arduino framework; normalize includes/lib deps; remove cross-component logging.  
2) Add calibration persistence and optional manual trigger; default to use stored values.  
3) Refine INT/poll fallback and recovery backoff to reduce timeouts.  
4) Document configuration options and recommended wiring/timing.  
5) Re-test multi-sensor flow (XSHUT sequencing, address changes).  

## Plan: Platform layer clarity & ULD cleanup (Arduino)

1. Platform/adapter audit
   - Locate all Arduino-specific I2C usage: search for `Wire` or raw ESP-IDF i2c calls inside `components/vl53l1x` and ULD platform files. Primary suspects: `VL53L1X_i2ccoms` / `vl53l1_platform` equivalents and any direct `Wire.beginTransmission`/`requestFrom` in wrapper code.
   - Identify existing abstraction: current driver inherits `i2c::I2CDevice` (ESPHome), but ULD platform layer may bypass it. Confirm whether the ULD platform files already provide a shim that could call ESPHome’s `I2CDevice` methods.
   - Audit result (done): Driver code uses only ESPHome I2CDevice and ULD APIs; no `Wire` or raw I2C calls in `components/`. External library `VL53L1X_ULD` (from PlatformIO) uses its own platform `VL53L1X_i2ccoms.cpp` (Wire-based). Calibration examples under `calibration/` still call `Wire.begin()` but are not built. No in-repo ULD sources are compiled, so pruning locally has no effect unless we vendor/override the library platform layer.

2. Target design
   - Define/confirm a minimal platform I2C interface used by ULD calls (read/write reg, burst read/write) implemented on top of ESPHome `i2c::I2CDevice` API. No direct `Wire` in the driver or platform files.
   - Driver code (`vl53l1x.cpp`) should only use the wrapped ULD APIs and ESPHome helpers; low-level I2C transactions live in the platform adapter only.
   - Platform-specific bits (e.g., delay, millis) continue to use ESPHome/Arduino abstractions already in place.

3. Migration steps
   - Update platform files (i2ccoms/platform) to call ESPHome `I2CDevice`/`i2c::I2CBus` instead of `Wire` (or ensure they already do, then drop any leftover Wire includes).
   - Remove/replace any direct `Wire.*` in driver or ULD wrappers; keep only adapter calls.
   - Adjust includes: drop unused Arduino headers where adapter is used.
   - Order: (a) fix adapter implementation; (b) remove direct Wire uses; (c) clean includes; (d) re-run build.

4. ULD cleanup
   - Inventory ULD files in repo (API/calibration/platform/types/error_codes). Mark which are referenced by the build (via IDE listing or compile logs).
   - Identify unused examples/sketches under `calibration/` or redundant platform variants; these can be isolated or removed from build paths (not necessarily deleted if still useful as docs).
   - Trim includes in `vl53l1x.h/.cpp` to only needed ULD headers.

5. Validation plan
   - Run `esphome compile esphome.yaml` (Arduino) to ensure build stays green after refactor.
   - Spot-check runtime on hardware if available (basic ranging) to confirm I2C access still works; otherwise rely on compile + log inspection for I2C transactions.
