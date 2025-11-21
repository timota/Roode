# VL53L1X ESP-IDF backend parity notes

## Implemented / matches Arduino ULD backend
- ROI per-measurement programming following UM2555.
- Timing budget & intermeasurement setters mapped to existing RangingMode values.
- Interrupt-ready path (GPIO change) with polling fallback.
- XSHUT power-cycle/restart with re-init for recovery and address bring-up.
- Offset and crosstalk register writes.

## Missing / reduced vs Arduino backend
- Offset/xTalk calibration flows: added single-shot offset calibrate helper; xtalk calibration still TODO (needs multi-sample routine per ULD).
- Range status diagnostics: sigma/signal thresholds now settable; publishing extra diagnostics to ESPHome sensors is still optional/TODO.
- Multi-sensor coordination: basic XSHUT sequencing restored; detailed retry/backoff metrics not yet ported.
- Timing budget: moved to ULD-style macro-period calculation; confirm against all distance modes in field.

## TODOs
- Add xtalk calibration routine (multi-sample) and wire to Roode maintenance path.
- Add optional ESPHome sensors to publish sigma/signal/range status for debugging.
- Backport richer recovery/backoff logging and interrupt validation thresholds if field data demands it.
