#include "uwb_processing.h"
#include <math.h>

// -------- GLOBALS --------
float anchor2_x = 2.0;    // will be set during calibration; but set to 2m jic
const float alpha = 0.3;  // 0.2 smoother, 0.4 more responsive; for EMA smoothing

// changed alpha from 0.4 to 0.2
// moved EMA from x and y to d1 and d2

// -------- FUNCTIONS --------
void sendAT(String cmd, HardwareSerial &uwb) {
  uwb.print(cmd);
  delay(100);
}
float median3(float a, float b, float c) {
  if ((a >= b && a <= c) || (a >= c && a <= b)) return a;
  if ((b >= a && b <= c) || (b >= c && b <= a)) return b;
  return c;
}
void configureTag(HardwareSerial &uwb) {
  Serial.println("Entering AT mode...");

  sendAT("+++", uwb);
  sendAT("AT+ROLE=1", uwb);
  sendAT("AT+RESPONDER_NUM=2", uwb);
  sendAT("AT+SRCADDR=0000", uwb);
  sendAT("AT+DSTADDR=11112222333344445555", uwb);
  sendAT("AT+INTV=50", uwb);
  sendAT("AT+RESET", uwb);
}
// -------- Noise-tolerant 2-anchor least-squares solver ----------
void computeXY_LS(float d1, float d2, float &x, float &y) {

  // ----- Approximate X using distance difference (stable) -----
  x = (d1 * d1 - d2 * d2 + anchor2_x * anchor2_x) / (2 * anchor2_x);

  // ----- Compute Y using least-squares -----
  float dx1 = x - 0;
  float dx2 = x - anchor2_x;

  float y_sq1 = d1 * d1 - dx1 * dx1;
  float y_sq2 = d2 * d2 - dx2 * dx2;

  // Clamp small negatives to zero (noise-tolerant)
  if (y_sq1 < 0) y_sq1 = 0;
  if (y_sq2 < 0) y_sq2 = 0;

  // Least-squares estimate of Y
  y = (sqrt(y_sq1) + sqrt(y_sq2)) / 2.0;
}
// -------- Parse distance from UWB output ----------
bool parseDistance(String line, String &srcAddr, float &dist) {
  line.trim();
  // Expecting format: P0,1111,57cm,14dB
  int firstComma = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);
  int cmIndex = line.indexOf("cm", secondComma + 1);

  if (firstComma >= 0 && secondComma > firstComma && cmIndex > secondComma) {
    srcAddr = line.substring(firstComma + 1, secondComma);
    String val = line.substring(secondComma + 1, cmIndex);
    val.trim();
    dist = val.toFloat() / 100.0;  // convert cm -> meters
    return true;
  }
  return false;
}
// -------- Calibration function ----------
void calibrateAnchors(float d1, float d2) {
  anchor2_x = d1 + d2;  // tag assumed between anchors
  #ifdef DEBUG
  Serial.print("Calibration done! Anchor distance = ");
  Serial.print(anchor2_x, 2);
  Serial.println(" m");
  #endif
}

bool collectCalibrationSample(float d1, float d2, float &offset_x, float &offset_y) {
  static float d1_samples[CALIBRATION_SAMPLE_SIZE];
  static float d2_samples[CALIBRATION_SAMPLE_SIZE];
  static int sample_count = 0;

  d1_samples[sample_count] = d1;
  d2_samples[sample_count] = d2;
  sample_count++;

  Serial.print("[CAL] Sample ");
  Serial.print(sample_count);
  Serial.print("/");
  Serial.println(CALIBRATION_SAMPLE_SIZE);

  if (sample_count < CALIBRATION_SAMPLE_SIZE) return false;

  // --- Simple mean over all samples ---
  float d1_sum = 0, d2_sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLE_SIZE; i++) {
    d1_sum += d1_samples[i];
    d2_sum += d2_samples[i];
  }

  float d1_cal = d1_sum / CALIBRATION_SAMPLE_SIZE;
  float d2_cal = d2_sum / CALIBRATION_SAMPLE_SIZE;

  // --- Set anchor separation and compute origin offset ---
  calibrateAnchors(d1_cal, d2_cal);

  float cal_x, cal_y;
  computeXY_LS(d1_cal, d2_cal, cal_x, cal_y);
  offset_x = cal_x;
  offset_y = cal_y;

  Serial.print("[CAL] Done. anchor2_x="); Serial.print(anchor2_x, 3);
  Serial.print("  offset=("); Serial.print(offset_x, 3);
  Serial.print(", "); Serial.print(offset_y, 3); Serial.println(")");

  sample_count = 0;
  return true;
}