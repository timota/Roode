# ESPHome ESP-IDF Migration Plan (feat/driver-spike)

We will execute these steps in order. After completing each step, update this file: mark it **DONE**, add a short note, and keep commits small and focused.

1. Review prior spike work and notes **DONE**  
   - `README_idf_gap.md` is no longer present; reviewed current VL53L1X spike code and espIdf.yaml.  
   - Current pain points: auto-cal start timing/timeout instability, on_boot trigger removed; build now succeeds.  
   - Example config uses esp-idf I2C bus with INT/XSHUT pins; Arduino support already unused here.

2. Freeze Arduino usage (stop adding new Arduino deps)  
   - Audit VL53L1X/Roode for Arduino APIs. Avoid new Arduino calls while migrating.

3. Replace Arduino-specific APIs with ESPHome abstractions  
   - Use `esphome::i2c::I2CDevice`, `millis()/delay()`, scheduler, GPIOPin, etc. Remove direct ESP-IDF and Arduino calls in driver/component code.

4. Stabilize init & calibration flow under ESP-IDF  
   - Simplify initialization, warm-up, and auto-calibration; ensure single, late, reliable auto-cal per boot; align wait timings with ULD guidance.

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
