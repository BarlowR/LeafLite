// Replay upstream's recorded 50Hz baro+accel data through leaflite's two
// pipelines (baro-only Vario vs fused GravityTracker+KalmanPA) and compare.
// Build: g++ -std=c++17 -I../src replay.cpp -o replay && ./replay <csv>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>

#include "fusion.h"
#include "vario.h"

struct Sample { double t, pa, accelG; };

int main(int argc, char** argv) {
  FILE* f = fopen(argc > 1 ? argv[1] : "baro_accel_50hz_1.csv", "r");
  if (!f) { printf("no input file\n"); return 1; }
  char line[128];
  fgets(line, sizeof(line), f);  // header
  std::vector<Sample> data;
  while (fgets(line, sizeof(line), f)) {
    Sample s{};
    if (sscanf(line, "%lf,%lf,%lf", &s.t, &s.pa, &s.accelG) >= 2) data.push_back(s);
  }
  fclose(f);
  printf("samples: %zu, duration %.1fs\n", data.size(), data.back().t);

  Vario baroOnly;
  if (argc > 3) { baroOnly.measurementVariance = atof(argv[2]); baroOnly.accelVariance = atof(argv[3]); }
  KalmanPA fused;
  GravityTracker grav;

  double alt0 = Vario::pressureToAltM(data[0].pa);
  baroOnly.reset(alt0);
  double minAlt = 1e9, maxAlt = -1e9;
  double maxFusedV = -99, minFusedV = 99, maxBaroV = -99, minBaroV = 99;
  std::vector<double> tv, fusedV, baroV, altV;

  double lastT = data[0].t - 0.02;
  for (auto& s : data) {
    double dt = s.t - lastT;
    lastT = s.t;
    if (dt <= 0) continue;
    double altM = Vario::pressureToAltM(s.pa);
    minAlt = std::fmin(minAlt, altM); maxAlt = std::fmax(maxAlt, altM);

    baroOnly.update((float)altM, (float)dt);
    grav.setTotal(s.accelG);  // dataset has only vertical accel; assume quiet flight
    double a = grav.process(s.accelG, dt) * 9.80665;
    fused.update(s.t, altM, a);

    double fv = fused.initialized() ? fused.velocity() : 0;
    double bv = baroOnly.climbCms() / 100.0;
    tv.push_back(s.t); fusedV.push_back(fv); baroV.push_back(bv); altV.push_back(altM);
    maxFusedV = std::fmax(maxFusedV, fv); minFusedV = std::fmin(minFusedV, fv);
    maxBaroV = std::fmax(maxBaroV, bv); minBaroV = std::fmin(minBaroV, bv);
    if (std::isnan(fv) || std::isnan(bv)) { printf("NaN at t=%.2f\n", s.t); return 1; }
  }

  printf("altitude range: %.1f .. %.1f m (span %.1f m)\n", minAlt, maxAlt, maxAlt - minAlt);
  printf("baro-only velocity range: %+.2f .. %+.2f m/s\n", minBaroV, maxBaroV);
  printf("fused     velocity range: %+.2f .. %+.2f m/s\n", minFusedV, maxFusedV);
  printf("final gravity estimate: %.4f g\n", grav.gravity());

  // Lag: cross-correlate fused vs baro-only velocity over +/-1s shifts.
  // Positive best-shift means baro-only lags fused (fused leads).
  int n = (int)tv.size();
  int bestShift = 0; double bestCorr = -1e18;
  for (int shift = -50; shift <= 50; shift++) {
    double c = 0; int cnt = 0;
    for (int i = 0; i < n; i++) {
      int j = i + shift;
      if (j < 0 || j >= n) continue;
      c += fusedV[i] * baroV[j];
      cnt++;
    }
    c /= cnt;
    if (c > bestCorr) { bestCorr = c; bestShift = shift; }
  }
  printf("fused leads baro-only by ~%d ms (cross-correlation peak)\n", bestShift * 20);

  // Quiet-segment noise: velocity stddev over the first second (device at rest)
  double m1 = 0, m2 = 0, q1 = 0, q2 = 0; int qn = 0;
  for (int i = 0; i < n && tv[i] < 1.0; i++) { m1 += fusedV[i]; m2 += baroV[i]; qn++; }
  m1 /= qn; m2 /= qn;
  for (int i = 0; i < qn; i++) { q1 += (fusedV[i]-m1)*(fusedV[i]-m1); q2 += (baroV[i]-m2)*(baroV[i]-m2); }
  printf("rest-noise stddev: fused %.3f m/s, baro-only %.3f m/s\n", sqrt(q1/qn), sqrt(q2/qn));

  // Dump for plotting
  FILE* out = fopen("replay_out.csv", "w");
  fprintf(out, "t,alt,fused_v,baro_v,accel\n");
  for (int i = 0; i < n; i++)
    fprintf(out, "%.3f,%.2f,%.3f,%.3f,%.3f\n", tv[i], altV[i], fusedV[i], baroV[i], data[i+1].accelG);
  fclose(out);
  return 0;
}
