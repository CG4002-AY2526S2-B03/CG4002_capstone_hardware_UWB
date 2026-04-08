#include "Arduino.h"

extern float anchor2_x;
extern const float alpha;

#define CALIBRATION_SAMPLE_SIZE 20

// -------- FUNCTIONS --------
void configureTag(HardwareSerial &uwb);
bool parseDistance(String line, String &srcAddr, float &dist);
float median3(float a, float b, float c);
void computeXY_LS(float d1, float d2, float &x, float &y);

void calibrateAnchors(float d1, float d2);
bool collectCalibrationSample(float d1, float d2, float &offset_x, float &offset_y);
