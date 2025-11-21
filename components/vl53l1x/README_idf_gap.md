# VL53L1X ESP-IDF backend parity notes

## Implemented / matches Arduino ULD backend
- ROI per-measurement programming following UM2555.
- Timing budget & intermeasurement setters mapped to existing RangingMode values.
- Interrupt-ready path (GPIO change) with polling fallback.
- XSHUT power-cycle/restart with re-init for recovery and address bring-up.
- Offset and crosstalk register writes.

## Missing / reduced vs Arduino backend
- BUS selection relies on ESPHome I2C bus `get_port()`; if unavailable, defaults to port 0. TODO: verify against ESPHome IDF I2C API and plumb explicitly.
- Offset/xTalk calibration flows (run-time calibration routines) are not ported; only stored-value setters exist. TODO: port calibration helpers from Arduino ULD or ST ULD.
- Range status filtering mirrors ULD basic mapping but does not expose sigma/signal thresholds or detailed diagnostics. TODO: add thresholds and status sensors if needed.
- Multi-sensor coord (address change sequencing across multiple XSHUT pins) simplified; restore detailed logging and retry counters as in legacy driver if required.
- Timing budget encoding currently approximates macro period; for production, port full ULD timing calc to match all modes.

## TODOs
- Confirm ESPHome I2C bus port retrieval and remove default-port fallback.
- Add calibration routines (offset/xtalk) callable from Roode or a maintenance path.
- Add sigma/signal threshold setters and status publish hooks for ESPHome sensors if desired.
- Backport richer recovery/backoff logic and interrupt validation from the legacy Arduino driver if field data demands it.
