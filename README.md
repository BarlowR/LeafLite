# leaflite

Minimal rewrite of [Leaf](https://github.com/DangerMonkeys/leaf) firmware for Leaf
3.2.3 hardware: IMU-fused baro vario (Mahony AHRS + upstream's flight-tested Kalman
and gravity gates, baro-only fallback), XCTracer-style piezo audio, GPS, IGC + raw
sensor logging, BLE to phone apps (LK8EX1 + NMEA passthrough), and a minimal LCD
showing 5s/20s windowed average climb. One external dependency (NimBLE-Arduino).

## Build & flash
```
pip install platformio
pio run -t upload && pio device monitor
```
If upload fails with "No serial data received", the device is off, asleep, or the
firmware crashed: hold **Boot** (pin through the smallest speaker hole) while
plugging in USB, upload, then reset (pin through the largest speaker slot) or replug.

## Controls
| Input | Action |
|---|---|
| UP / DOWN | Volume (chirp confirms) |
| CENTER click | Start/stop logging: IGC + raw sensor log (raw works without a GPS fix) |
| CENTER hold 3s | Power off (true off on battery via power latch; deep sleep on USB, CENTER wakes) |
| RIGHT hold | Battery page: icon, percent, pack voltage |
| LEFT hold 1s | USB drive mode: SD card mounts on the computer as a USB drive. Exit: CENTER, eject, or auto if no host connects in ~8s |

Automatic behavior: IGC logging starts on first GPS lock (a manual stop vetoes it
for the boot) and every shutdown path closes the logs; low battery (<3.20V
sustained, not charging/USB) powers off cleanly.

Signals: rising double-chirp = GPS lock. Top row of the vario page: antenna icon
(blinking = searching, solid = locked), steady dot = recording, battery icon =
charging.

## Tone profile
Beeps follow an XCTracer-style 12-point table {climb → frequency, cycle, duty%},
linearly interpolated, with climb/sink on/off thresholds (defaults: the published
profile from cloudbasemayhem.com/xctracer-tone-settings). Customize by putting
`tones.txt` (XCTRACER.TXT syntax) in the SD card root — a commented template
identical to the defaults is in `sd/tones.txt`. Unknown keys are ignored.

## Raw sensor logging & replay
Logging writes `*.raw.csv` next to the IGC (`/raw-NNN.csv` when no GPS date):
baro 20Hz, body-frame IMU 100Hz, GPS 1Hz; ~21MB/h. Replay it through the exact
firmware pipeline on the host to tune `ahrs.h` / `fusion.h` / `vario.h`:
```
cd test && g++ -std=c++17 -I../src rawlog_replay.cpp -o rawlog_replay
./rawlog_replay flight.raw.csv    # stats + rawlog_replay_out.csv for plotting
```
Host unit tests: `g++ -std=c++17 -I../src host_test.cpp && ./a.out` (same for
`display_test.cpp`).

Hardware verification status (what's confirmed on the bench vs. still assumed)
lives in [docs/bench-verification.md](docs/bench-verification.md).

## Known limitations (deliberate)
- No IGC G-record: fine for personal logs / XContest fun leagues, not FAI Cat 1.
- Battery percent is a linear map (3.25–4.08V, ~2s EMA): optimistic under load
  and while charging.
- Charge input fixed at 500mA; `power.h setInputMax()` selects ~1.35A. Never
  drive I1/I2 both HIGH (= charging off).
- No charging screen while powered off (the charger IC still charges).
- Mahony has no integral term — slow gyro bias is absorbed by the gravity
  tracker; revisit if you see false lift after big temperature swings.
