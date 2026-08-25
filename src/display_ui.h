#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cmath>
#include <cstdint>
#include <cstring>
#endif

// Display content + averaging, kept free of hardware so it host-tests.
// The panel is 192x96 native but mounted portrait in the case (upstream drives
// it with U8G2_R1 = 90 deg software rotation), so we draw in a logical 96x192
// portrait space and map to native pages in FrameBuffer::set. If the image
// comes up upside down on your unit, flip DISPLAY_ROT_CCW.
// Layout (96x192 portrait): two panels stacked.
//   top:    5s  average climb, big 7-seg, one decimal, sign, "m/s" below
//   bottom: 20s average climb, same
// No font bitmaps: everything is rectangles (7-seg digits + rect-built "m/s").

// ---- windowed climb averages ----
// True windowed average: (alt_now - alt_N_seconds_ago) / N. Integrating
// altitude difference is noise-free compared to averaging instantaneous
// climb samples — the classic way vario averagers work.
class ClimbAverager {
 public:
  static const int HZ = 20, SPAN_S = 20, N = HZ * SPAN_S;

  void push(float altM) {
    buf_[head_] = altM;
    head_ = (head_ + 1) % N;
    if (count_ < N) count_++;
  }

  // Average climb (m/s) over the last `seconds`, using what's buffered so far.
  float avg(int seconds) const {
    int want = seconds * HZ;
    int have = count_ > want ? want : count_;
    if (have < HZ) return 0;  // <1s of data: not meaningful
    int newest = (head_ + N - 1) % N;
    int oldest = (head_ + N - have) % N;
    return (buf_[newest] - buf_[oldest]) / ((have - 1) / (float)HZ);
  }

 private:
  float buf_[N] = {};
  int head_ = 0, count_ = 0;
};

// ---- framebuffer + drawing ----
#ifndef DISPLAY_ROT_CCW
#define DISPLAY_ROT_CCW 1  // bench-verified orientation for 3.2.3 (0 was upside down)
#endif

class FrameBuffer {
 public:
  static const int W = 192, H = 96, PAGES = 12;  // native
  static const int LW = 96, LH = 192;            // logical (portrait)
  uint8_t px[PAGES * W];

  void clear() { memset(px, 0, sizeof(px)); }

  // Logical portrait (x: 0..95, y: 0..191) -> native panel coords
  void set(int lx, int ly) {
    if (lx < 0 || lx >= LW || ly < 0 || ly >= LH) return;
#if DISPLAY_ROT_CCW
    int x = W - 1 - ly, y = lx;
#else
    int x = ly, y = H - 1 - lx;
#endif
    px[(y / 8) * W + x] |= 1 << (y % 8);  // LSB = top row (panel data format 0x0C)
  }

  void fillRect(int x, int y, int w, int h) {
    for (int j = y; j < y + h; j++)
      for (int i = x; i < x + w; i++) set(i, j);
  }

  void vline(int x, int y, int h) { fillRect(x, y, 1, h); }
};

// ---- 7-segment digits ----
//  segment bits: 0=top 1=topR 2=botR 3=bottom 4=botL 5=topL 6=middle
static const uint8_t SEG7[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66,
                                 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

// Draw an arbitrary 7-seg mask in a w x h cell with segment thickness t.
inline void drawSegs(FrameBuffer& fb, uint8_t s, int x, int y, int w, int h, int t) {
  int mid = y + (h - t) / 2;
  if (s & 0x01) fb.fillRect(x + t, y, w - 2 * t, t);              // top
  if (s & 0x02) fb.fillRect(x + w - t, y + t, t, (h - 3 * t) / 2); // topR
  if (s & 0x04) fb.fillRect(x + w - t, mid + t, t, (h - 3 * t) / 2); // botR
  if (s & 0x08) fb.fillRect(x + t, y + h - t, w - 2 * t, t);      // bottom
  if (s & 0x10) fb.fillRect(x, mid + t, t, (h - 3 * t) / 2);      // botL
  if (s & 0x20) fb.fillRect(x, y + t, t, (h - 3 * t) / 2);        // topL
  if (s & 0x40) fb.fillRect(x + t, mid, w - 2 * t, t);            // middle
}

// Draw one digit in a w x h cell with segment thickness t.
inline void drawDigit(FrameBuffer& fb, int d, int x, int y, int w, int h, int t) {
  if (d < 0 || d > 9) return;
  drawSegs(fb, SEG7[d], x, y, w, h, t);
}

// "USb" splash for USB drive mode (7-seg letter masks: U=0x3E, S=5, b=0x7C).
inline void renderUsbPage(FrameBuffer& fb) {
  fb.clear();
  drawSegs(fb, 0x3E, 6, 74, 24, 44, 4);
  drawSegs(fb, 0x6D, 36, 74, 24, 44, 4);
  drawSegs(fb, 0x7C, 66, 74, 24, 44, 4);
}

// Value as [sign] D.D — clamped to +/-9.9 m/s (beyond that you have bigger
// concerns than the averager readout).
inline void drawClimb(FrameBuffer& fb, float v, int x, int y, int dw, int dh, int t) {
  if (v > 9.9f) v = 9.9f;
  if (v < -9.9f) v = -9.9f;
  int tenths = (int)lround(fabs(v) * 10.0);
  int whole = tenths / 10, frac = tenths % 10;
  int mid = y + (dh - t) / 2;
  // sign column
  if (v < -0.049f) fb.fillRect(x, mid, dw - 2, t);                 // minus
  else if (v > 0.049f) {                                           // plus
    fb.fillRect(x, mid, dw - 2, t);
    fb.fillRect(x + (dw - 2 - t) / 2, mid - (dw - 2 - t) / 2, t, dw - 2);
  }
  int cx = x + dw + 2;
  drawDigit(fb, whole, cx, y, dw, dh, t);
  fb.fillRect(cx + dw + 3, y + dh - t, t, t);  // decimal point
  drawDigit(fb, frac, cx + dw + 3 + t + 3, y, dw, dh, t);
}

// Battery icon at (x,y), 24x12: outline, terminal nub, fill by percent,
// lightning bolt overlay (drawn in "off" pixels would need XOR; instead the
// bolt sits left of the icon). Only rendered while charging.
inline void drawBatteryIcon(FrameBuffer& fb, int x, int y, int pct) {
  const int w = 22, h = 12;
  // outline
  fb.fillRect(x, y, w, 1); fb.fillRect(x, y + h - 1, w, 1);
  fb.fillRect(x, y, 1, h); fb.fillRect(x + w - 1, y, 1, h);
  fb.fillRect(x + w, y + 3, 2, h - 6);  // terminal nub
  // fill proportional to percent
  int fillW = (w - 4) * pct / 100;
  if (fillW > 0) fb.fillRect(x + 2, y + 2, fillW, h - 4);
  // bolt glyph to the left
  int bx = x - 8;
  fb.fillRect(bx + 3, y + 1, 2, 4);
  fb.fillRect(bx + 1, y + 4, 5, 2);
  fb.fillRect(bx + 2, y + 6, 2, 5);
}

// "m/s" unit label built from rectangles (no font). 's' is the 7-seg '5'.
inline void drawMsLabel(FrameBuffer& fb, int x, int y, int h, int t) {
  int w = h;  // roughly square cells
  // m: three legs + top bar
  fb.fillRect(x, y, t, h);
  fb.fillRect(x + (w - t) / 2, y, t, h);
  fb.fillRect(x + w - t, y, t, h);
  fb.fillRect(x, y, w, t);
  // slash: bottom-left -> top-right diagonal
  int sx = x + w + 3;
  for (int j = 0; j < h; j++)
    fb.fillRect(sx + (h - 1 - j) * (w - t) / (h - 1), y + j, t, 1);
  // s
  drawDigit(fb, 5, sx + w + 3, y, w, h, t);
}

// Battery page (shown only while RIGHT is held): big icon with
// fill level, percent in 7-seg, pack voltage below — the voltage readout is
// there to calibrate the crude linear percent map against reality.
inline void renderBatteryPage(FrameBuffer& fb, int pct, bool charging, int mv) {
  fb.clear();
  // icon: 64x32 outline with nub, fill proportional to percent
  const int x = 14, y = 28, w = 64, h = 32, t = 3;
  fb.fillRect(x, y, w, t); fb.fillRect(x, y + h - t, w, t);
  fb.fillRect(x, y, t, h); fb.fillRect(x + w - t, y, t, h);
  fb.fillRect(x + w, y + 9, 4, h - 18);  // terminal nub
  int fillW = (w - 2 * t - 4) * pct / 100;
  if (fillW > 0) fb.fillRect(x + t + 2, y + t + 2, fillW, h - 2 * t - 4);
  if (charging) {  // bolt centered above the icon
    fb.fillRect(46, 6, 4, 8);
    fb.fillRect(42, 12, 12, 4);
    fb.fillRect(46, 14, 4, 10);
  }
  // percent, right-aligned 7-seg, with a rect-built % sign
  const int dy = 84, dw = 20, dh = 40, dt = 4;
  drawDigit(fb, pct % 10, 52, dy, dw, dh, dt);
  if (pct >= 10) drawDigit(fb, (pct / 10) % 10, 28, dy, dw, dh, dt);
  if (pct >= 100) drawDigit(fb, 1, 4, dy, dw, dh, dt);
  const int px = 76, py = dy + 8;
  fb.fillRect(px, py, 5, 5);
  fb.fillRect(px + 9, py + 19, 5, 5);
  for (int j = 0; j < 24; j++) fb.fillRect(px + (23 - j) * 9 / 23, py + j, 3, 1);
  // pack voltage as X.XX plus a small V
  const int vy = 148, vw = 12, vh = 22, vt = 3;
  drawDigit(fb, (mv / 1000) % 10, 20, vy, vw, vh, vt);
  fb.fillRect(36, vy + vh - vt, vt, vt);  // decimal point
  drawDigit(fb, (mv / 100) % 10, 44, vy, vw, vh, vt);
  drawDigit(fb, (mv / 10) % 10, 60, vy, vw, vh, vt);
  for (int j = 0; j < vh; j++) {  // V: two converging arms
    fb.fillRect(76 + j * 4 / vh, vy + j, 2, 1);
    fb.fillRect(86 - j * 4 / vh, vy + j, 2, 1);
  }
}

// GPS icon states. Searching draws the armless antenna and the caller blinks
// it by alternating with NONE each display frame (2Hz refresh -> 1Hz blink);
// locked draws the full antenna with arms, solid.
enum : uint8_t { GPS_ICON_NONE = 0, GPS_ICON_SEARCHING = 1, GPS_ICON_LOCKED = 2 };

// Little antenna (mast + base, diverging arms when locked), 12x12.
inline void drawGpsIcon(FrameBuffer& fb, int x, int y, bool locked) {
  fb.fillRect(x + 5, y + 4, 2, 6);   // mast
  fb.fillRect(x + 2, y + 10, 8, 2);  // base
  if (!locked) return;
  for (int j = 0; j < 5; j++) {      // arms
    fb.fillRect(x + 4 - j, y + 4 - j, 2, 2);
    fb.fillRect(x + 6 + j, y + 4 - j, 2, 2);
  }
}

// Recording dot: filled ~10px circle (octagon from rects).
inline void drawRecIcon(FrameBuffer& fb, int x, int y) {
  fb.fillRect(x + 2, y, 6, 10);
  fb.fillRect(x, y + 2, 10, 6);
  fb.fillRect(x + 1, y + 1, 8, 8);
}

// Full page (96x192 portrait): two stacked panels, 5s avg on top, 20s below.
// Top row: GPS icon left (blinking antenna = searching, solid + arms =
// locked), steady REC dot beside it while a log file is open, battery icon
// right when charging.
inline void renderVarioPage(FrameBuffer& fb, float avg5, float avg20,
                            bool charging = false, int battPct = 0,
                            uint8_t gpsIcon = GPS_ICON_NONE, bool recording = false) {
  fb.clear();
  if (gpsIcon != GPS_ICON_NONE) drawGpsIcon(fb, 4, 4, gpsIcon == GPS_ICON_LOCKED);
  if (recording) drawRecIcon(fb, 22, 5);
  if (charging) drawBatteryIcon(fb, 66, 6, battPct);
  fb.fillRect(4, 95, 88, 1);                  // divider
  drawClimb(fb, avg5, 3, 20, 19, 56, 5);      // 5s average
  drawMsLabel(fb, 56, 80, 11, 2);
  drawClimb(fb, avg20, 3, 116, 19, 56, 5);    // 20s average
  drawMsLabel(fb, 56, 176, 11, 2);
}
