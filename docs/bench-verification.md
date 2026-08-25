# Bench verification status

Hardware assumptions that still need confirming on a real Leaf 3.2.3, in
priority order. Move items to "Verified" (with what was observed) as they pass.

## Open

1. **CHG_GOOD polarity** — flight-safety relevant: `power.charging()` gates the
   low-battery shutdown. With a working battery actually charging, the serial
   `charging` state must read true and the bolt icon must show; unplug and it
   must drop within a second. If inverted, flip the `== LOW` in `power.h`.
   Verify before flying.
2. **Volume pins** — the VOLA/VOLB truth table (2-bit level select) is a guess.
   Step the volume through 0–3 and note which combination gives which loudness.
3. **Display contrast** — `setContrast` default 238 (9-bit VOP). Adjust if the
   panel is washed out or too dark; the value that looks right goes in
   `st75256.h`.
4. **Vario feel** — the Kalman is not tuned to match upstream Leaf. Fly next to
   a known-good vario; tune offline via the raw-log replay
   (`test/rawlog_replay.cpp`, see README) before trusting it in lift.

## Verified

- **Compile & flash** — builds clean on the pinned pioarduino platform; uploads
  over TinyUSB CDC (Boot-button recovery works when firmware is down).
- **Buttons** — active-HIGH with pulldowns, matching upstream `buttons.cpp`
  (the original active-low guess caused an instant power-off boot loop).
- **Power latch** — GPIO 48 driven HIGH first thing in `setup()`; without it
  the device only runs on USB. Power-off = latch drop (battery) + deep sleep
  (USB).
- **Baro** — MS5611 at 0x77 initializes and samples at ~20Hz.
- **Display** — ST75256 flush addressing: row window 8..19 (u8g2's jlx19296
  handler sends 8+y_pos; the glass COMs start at RAM row 64) and column window
  0..191 (0..255 makes the address pointer wrap 64 bytes late per page and
  shear the frame). SPI mode 0 @ 4MHz. Rotation: `DISPLAY_ROT_CCW 1` is
  right-side-up.
- **SD** — mounts, IGC + raw logs write, USB drive mode exposes the card.
- **BLE** — advertises as `leaflite` (NimBLE 2.x needs `adv->setName()`; the
  GAP name alone is not in the advertisement).
- **Battery sense** — pin/divider verified against upstream (GPIO 1, x69/41).
  Bench unit's cell was disconnected/dead: the charger retry-cycles the sense
  line 4.2V→1.7V every ~6s, which is why the low-battery shutdown requires a
  stable (<150mV spread) 10s window before firing.
