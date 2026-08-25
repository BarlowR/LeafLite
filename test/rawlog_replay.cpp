// Replay a leaflite raw sensor log (B/I/G rows from rawlog.h) through the
// EXACT firmware pipeline: Ahrs -> GravityTracker -> KalmanPA (fused) vs the
// baro-only Vario fallback. Tweak constants in ahrs.h/fusion.h/vario.h,
// rebuild, re-run — same math the device flies.
// Build: g++ -std=c++17 -I../src rawlog_replay.cpp -o rawlog_replay
// Run:   ./rawlog_replay flight.raw.csv   (writes rawlog_replay_out.csv)
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "ahrs.h"
#include "fusion.h"
#include "vario.h"

int main(int argc, char** argv) {
  if (argc < 2) { printf("usage: %s <flight.raw.csv>\n", argv[0]); return 1; }
  FILE* f = fopen(argv[1], "r");
  if (!f) { printf("cannot open %s\n", argv[1]); return 1; }

  Ahrs ahrs;
  GravityTracker grav;
  KalmanPA fused;
  Vario baroOnly;

  double lastImuT = -1, lastBaroT = -1;
  float awz = 1.0f, tot = 1.0f;
  long nI = 0, nB = 0, nG = 0, nBad = 0;
  double minAlt = 1e9, maxAlt = -1e9;
  std::vector<double> tv, fusedV, baroV, altV;

  char line[256];
  while (fgets(line, sizeof(line), f)) {
    if (line[0] == '#' || line[1] != ',') continue;
    if (line[0] == 'I') {
      unsigned long ms;
      float ax, ay, az, gx, gy, gz;
      if (sscanf(line + 2, "%lu,%f,%f,%f,%f,%f,%f", &ms, &ax, &ay, &az, &gx, &gy, &gz) != 7) {
        nBad++;
        continue;
      }
      double t = ms / 1000.0;
      float dt = lastImuT < 0 ? 0.01f : (float)(t - lastImuT);
      lastImuT = t;
      if (dt <= 0 || dt > 0.5f) continue;
      ahrs.update(ax, ay, az, gx, gy, gz, dt);
      awz = ahrs.verticalAccelG(ax, ay, az);
      tot = sqrtf(ax * ax + ay * ay + az * az);
      nI++;
    } else if (line[0] == 'B') {
      unsigned long ms;
      float pa, tc;
      if (sscanf(line + 2, "%lu,%f,%f", &ms, &pa, &tc) < 2) { nBad++; continue; }
      double t = ms / 1000.0;
      double dt = lastBaroT < 0 ? 0.05 : t - lastBaroT;
      lastBaroT = t;
      if (dt <= 0) continue;
      double alt = Vario::pressureToAltM(pa);
      if (!baroOnly.initialized()) baroOnly.reset((float)alt);
      baroOnly.update((float)alt, (float)dt);
      grav.setTotal(tot);
      fused.update(t, alt, grav.process(awz, dt) * 9.80665);
      minAlt = std::fmin(minAlt, alt);
      maxAlt = std::fmax(maxAlt, alt);
      tv.push_back(t);
      fusedV.push_back(fused.initialized() ? fused.velocity() : 0);
      baroV.push_back(baroOnly.climbCms() / 100.0);
      altV.push_back(alt);
      nB++;
    } else if (line[0] == 'G') {
      nG++;
    }
  }
  fclose(f);

  int n = (int)tv.size();
  if (n < 40) { printf("too little data (%d baro rows)\n", n); return 1; }
  printf("rows: %ld baro, %ld imu, %ld gps (%ld bad), %.1f min\n", nB, nI, nG, nBad,
         (tv.back() - tv.front()) / 60.0);
  printf("altitude range: %.1f .. %.1f m (span %.1f m)\n", minAlt, maxAlt, maxAlt - minAlt);
  printf("final gravity estimate: %.4f g\n", grav.gravity());

  // Lag: cross-correlate fused vs baro-only velocity over +/-2.5s shifts.
  // Positive best-shift means the fused estimate leads.
  int bestShift = 0;
  double bestCorr = -1e18;
  for (int shift = -50; shift <= 50; shift++) {
    double c = 0;
    int cnt = 0;
    for (int i = 0; i < n; i++) {
      int j = i + shift;
      if (j < 0 || j >= n) continue;
      c += fusedV[i] * baroV[j];
      cnt++;
    }
    c /= cnt;
    if (c > bestCorr) { bestCorr = c; bestShift = shift; }
  }
  double tick = (tv.back() - tv.front()) / (n - 1);
  printf("fused leads baro-only by ~%.0f ms (cross-correlation peak)\n", bestShift * tick * 1000);

  // Quiet-segment noise: velocity stddev over the first 2s (device at rest)
  double m1 = 0, m2 = 0, q1 = 0, q2 = 0;
  int qn = 0;
  for (int i = 0; i < n && tv[i] - tv.front() < 2.0; i++) { m1 += fusedV[i]; m2 += baroV[i]; qn++; }
  if (qn > 10) {
    m1 /= qn; m2 /= qn;
    for (int i = 0; i < qn; i++) {
      q1 += (fusedV[i] - m1) * (fusedV[i] - m1);
      q2 += (baroV[i] - m2) * (baroV[i] - m2);
    }
    printf("rest-noise stddev (first 2s): fused %.3f m/s, baro-only %.3f m/s\n",
           sqrt(q1 / qn), sqrt(q2 / qn));
  }

  FILE* out = fopen("rawlog_replay_out.csv", "w");
  fprintf(out, "t,alt,fused_v,baro_v\n");
  for (int i = 0; i < n; i++)
    fprintf(out, "%.3f,%.2f,%.3f,%.3f\n", tv[i], altV[i], fusedV[i], baroV[i]);
  fclose(out);
  printf("wrote rawlog_replay_out.csv (%d rows) for plotting\n", n);
  return 0;
}
