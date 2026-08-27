# leaflite

The [Leaf](https://github.com/DangerMonkeys/leaf) vario is a rad hardware platform for a paragliding variometer. This repository is a stripped down rewrite of software stack with a focused feature-set.

Features: 
* IGC logging
* Baro/GPS/IMU-fused state estimation (Mahony AHRS + Kalman)
* BLE to connect to phone flight instrument (I use [XCTrack](xctrack.org))
* XCTracer-style piezo audio tone configuration
* Display shows 5s/20s windowed average climb rates.

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
for the boot) and shutdown closes the logs (Manual power off or Low Voltage Cutoff).

Signals: rising double-chirp = GPS lock. Top row of the vario page: antenna icon
(blinking = searching, solid = locked), steady dot = recording, battery icon =
charging.

## Tone profile
Beeps follow an XCTracer-style 12-point table {climb → frequency, cycle, duty%},
linearly interpolated, with climb/sink on/off thresholds (defaults: the published
profile from cloudbasemayhem.com/xctracer-tone-settings). Customize by putting
`tones.txt` (XCTracer syntax) in the SD card root — a commented template
identical to the defaults is in `sd/tones.txt`. 

## Known limitations
- No IGC G-record: this device is fine for personal logs / XContest fun leagues, but is not passable for FAI Cat. 1.
- Battery percent is a linear map of voltage which is a crude estimation of charge state.
- Charge input fixed at 500mA and no charging screen while powered off.
- Mahony AHRS has no integral term — slow gyro bias is absorbed by the gravity
  tracker; revisit if you see false lift after big temperature swings.
