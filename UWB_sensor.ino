#include <HardwareSerial.h>
#include <math.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "ESP32MQTTClient.h"

#define UWB_RX 35  // to IO5/RX on UWB sensor
#define UWB_TX 15  // to IO6/TX on UWB sensor

HardwareSerial uwb(2);

float anchor2_x = 2.0;  // will be set during calibration; but set to 2m jic
const float alpha = 0.4;  // 0.2 smoother, 0.4 more responsive; for EMA smoothing

const char *ssid = "BEL 7462";
const char *password = "9*9V7p68";
const char *mqtt_broker = "172.20.10.11";
const char *clientID = "esp32-player-client";
const std::string playerEspPublishTopic = "/playerPosition";
ESP32MQTTClient mqttClient;

typedef struct {
  float x;
  float y;
} Position;

QueueHandle_t positionQueue;

// -------- Helper Functions ----------
void sendAT(String cmd) {
  uwb.print(cmd);
  delay(100);
}
float median3(float a, float b, float c) {
  if ((a >= b && a <= c) || (a >= c && a <= b)) return a;
  if ((b >= a && b <= c) || (b >= c && b <= a)) return b;
  return c;
}
void configureTag() {
  Serial.println("Entering AT mode...");

  sendAT("+++");
  sendAT("AT+ROLE=1");
  sendAT("AT+RESPONDER_NUM=2");
  sendAT("AT+SRCADDR=0000");
  sendAT("AT+DSTADDR=11112222333344445555");
  sendAT("AT+INTV=20");
  sendAT("AT+RESET");
  Serial.println("Tag configured. Starting ranging...");
}
// -------- Noise-tolerant 2-anchor least-squares solver ----------
bool computeXY_LS(float d1, float d2, float &x, float &y) {

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

  return true;
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
  Serial.println("incorrect raw data format");
  return false;
}
// -------- Calibration function ----------
void calibrateAnchors(float d1, float d2) {
  anchor2_x = d1 + d2;  // tag assumed between anchors
  Serial.print("Calibration done! Anchor distance = ");
  Serial.print(anchor2_x, 2);
  Serial.println(" m");
}
std::string formatPayload(float x, float y) {

  JsonDocument doc;

  doc["clientID"] = clientID;

  doc["position"]["x"] = x;
  doc["position"]["y"] = y;

  std::string payload;
  serializeJson(doc, payload);

  return payload;

}
void wifiConnect() {
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting WiFi...");
    delay(500);
  }

  Serial.println("WiFi connected");
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nFireBeetle UWB Tag System Starting...");

  uwb.begin(921600, SERIAL_8N1, UWB_RX, UWB_TX);

  configureTag();

  wifiConnect();

  mqttClient.setMqttClientName(clientID);
  mqttClient.setURL(mqtt_broker, 1883, "", "");
  mqttClient.loopStart();

  positionQueue = xQueueCreate(1, sizeof(Position));
  xTaskCreatePinnedToCore(
    uwbTask,
    "UWB Task",
    6000,
    NULL,
    2,
    NULL,
    1
  );
  xTaskCreatePinnedToCore(
    wirelessTask,
    "Wireless Task",
    6000,
    NULL,
    1,
    NULL,
    0
  );
}

void onMqttEvent(esp_mqtt_event_handle_t event) {
  mqttClient.onEventCallback(event);
}

void wirelessTask(void *pvParameters) {
  Position pos;
  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnect();
    }
    if (mqttClient.isConnected()) {
      if (xQueueReceive(positionQueue, &pos, 0) == pdTRUE) {
        std::string payload = formatPayload(pos.x, pos.y);
        mqttClient.publish(playerEspPublishTopic, payload, 0, false);
        Serial.printf("Sent: %s\n", payload.c_str());
      }
    }    
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void uwbTask(void *pvParameters) {
  // ===== Median filter buffers =====
  float d1_hist[3] = { 0, 0, 0 };
  float d2_hist[3] = { 0, 0, 0 };
  int hist_index = 0;

  // ===== EMA filtered position =====
  float x_filtered = 0;
  float y_filtered = 0;
  bool first_position = true;

  float d1 = 0.0, d2 = 0.0;
  bool calibrate = true;  // set true to perform calibration

  while (1) {
    if (uwb.available()) {
      String line = uwb.readStringUntil('\n');
      line.trim();

      String src;
      float dist;
      if (parseDistance(line, src, dist)) {
        if (src == "1111") d1 = dist;
        else if (src == "2222") d2 = dist;

        // Perform calibration if requested
        if (calibrate && d1 > 0 && d2 > 0) {
          calibrateAnchors(d1, d2);
          calibrate = false;
        }

        // Update median buffers
        d1_hist[hist_index] = d1;
        d2_hist[hist_index] = d2;
        hist_index = (hist_index + 1) % 3;

        // Compute median distances
        float d1_med = median3(d1_hist[0], d1_hist[1], d1_hist[2]);
        float d2_med = median3(d2_hist[0], d2_hist[1], d2_hist[2]);

        // Compute position
        if (!calibrate) {
          float x, y;
          if (computeXY_LS(d1, d2, x, y)) {
            // ----- EMA smoothing
            if (first_position) {
              x_filtered = x;
              y_filtered = y;
              first_position = false;
            } else {
              x_filtered = alpha * x + (1 - alpha) * x_filtered;
              y_filtered = alpha * y + (1 - alpha) * y_filtered;
            }

            Serial.print(x);
            Serial.print(", ");
            Serial.println(y);
          }
        }
      }
    }
    // while (uwb.available()) {
    //   char c = uwb.read();
    //   Serial.write(c);
    // }
  }
}

void loop() {
}