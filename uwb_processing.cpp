#include "uwb_processing.h"
#include <math.h>

// -------- GLOBALS --------
float anchor2_x = 2.0;    // will be set during calibration; but set to 2m jic
const float alpha = 0.4;  // 0.2 smoother, 0.4 more responsive; for EMA smoothing

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
  sendAT("AT+INTV=200", uwb);
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