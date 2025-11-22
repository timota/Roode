# ESPHome ESP-IDF Migration Plan (feat/driver-spike)

We will execute these steps in order. After completing each step, update this file: mark it **DONE**, add a short note, and keep commits small and focused.

1. Review prior spike work and notes **DONE**  
   - `README_idf_gap.md` is no longer present; reviewed current VL53L1X spike code and espIdf.yaml.  
   - Current pain points: auto-cal start timing/timeout instability, on_boot trigger removed; build now succeeds.  
   - Example config uses esp-idf I2C bus with INT/XSHUT pins; Arduino support already unused here.

2. Freeze Arduino usage (stop adding new Arduino deps) **DONE**  
   - Audit: `components/roode/roode.{h,cpp}` still include `Arduino.h`; no new Arduino deps elsewhere.  
   - Rule: do not introduce new Arduino calls; removal of existing includes will be handled in later steps.

3. Replace Arduino-specific APIs with ESPHome abstractions  
   - Use `esphome::i2c::I2CDevice`, `millis()/delay()`, scheduler, GPIOPin, etc. Remove direct ESP-IDF and Arduino calls in driver/component code.  
   - **DONE (initial sweep)**: Removed `Arduino.h` from Roode component; no other Arduino includes found. Further replacements will continue in later steps if any surface.

4. Stabilize init & calibration flow under ESP-IDF  
   - Simplify initialization, warm-up, and auto-calibration; ensure single, late, reliable auto-cal per boot; align wait timings with ULD guidance.  
   - **IN PROGRESS**: multiple attempts to delay/start auto-cal; still seeing early warm-up failures/timeouts.  
   - Substeps (port features from current Arduino VL53L1X driver):
     4.1 Init & boot: wait_for_boot sequencing; apply desired I2C address; handle address change safely.  
     4.2 Ranging mode config: distance mode, timing budget, intermeasurement (per RangingMode table).  
     4.3 ROI handling: per-read ROI set (width/height/center) with caching to avoid redundant writes.  
     4.4 Data-ready flow: INT-based ready if validated; fallback to polling; miss counter and retry to polling; re-validate INT after cooldown.  
     4.5 Timeout handling: on measurement timeout, power-cycle via XSHUT (if available), re-wait boot, record recovery count.  
     4.6 Interrupt validation: validate pin at setup; disable INT if fails; schedule periodic re-validation.  
     4.7 XSHUT sequencing for multi-sensor: pull peers low before init/address change; restore after.  
     4.8 Calibration application (static): apply stored offset and xtalk values at setup.  
     4.9 Error tracking: record_failure/consecutive_failures; mark_failed on hard errors.  
     4.10 Logging parity: keep informative logs (setup, pins, timeouts, recoveries) without Roode-only calls.  
     4.11 Auto-calibration (new flow under IDF): single late on_boot trigger, ULD-like timing windows, one controlled retry.
   - Paused status: ULD sources are flattened into components/vl53l1x/, includes partially adjusted; build currently failing (missing platform includes resolved, remaining issues are ESPHome HAL replacements in ULD code and removal of Roode log calls in vl53l1x.cpp). Resume from here.

5. Clean build & runtime under `framework: esp-idf`  
   - Ensure `esphome compile espIdf.yaml` succeeds; resolve I2C conflicts; verify logs show stable start-up without calibration spam.

6. Feature parity check vs Arduino implementation  
   - Verify ROI control, timing budget, distance modes, INT/XSHUT handling, multi-sensor sequencing, calibration services/buttons.

7. Drop Arduino support  
   - Remove/ignore Arduino framework configs; document ESP-IDF-only stance.

8. Update docs/comments  
   - Document new structure, calibration behavior, and “no raw ESP-IDF API” rule; update example YAMLs as needed.

9. Final verification  
   - Run compile/test; confirm runtime logs; summarize remaining risks or follow-ups.
