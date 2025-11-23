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
- Calibration: add optional persistence for offset/xtalk; provide a manual service/button; run auto-cal only when no stored data; use consistent timing windows.
- Calibration: add optional persistence for offset/xtalk; provide a manual service/button; run auto-cal only when no stored data; use consistent timing windows. Keep Roode zone recalibration separate (it adjusts entry/exit thresholds), but add a dedicated VL53L1X-level calibration trigger to avoid reusing `roode_platform->recalibration()` for sensor offset/xtalk.
  - Implemented: driver now persists offset/xtalk via ESPHome preferences and exposes `calibrate_and_store()` for manual triggers. Sample button added in `esphome.yaml`. Defaults: apply stored values first; auto-cal only when nothing stored or when explicitly triggered.
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
