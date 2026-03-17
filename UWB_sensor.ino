#include <HardwareSerial.h>
#include <math.h>
#include "Mqtt.h"
#include "config.h"
#include "uwb_processing.h"

#define UWB_RX 35  // to IO5/RX on UWB sensor
#define UWB_TX 15  // to IO6/TX on UWB sensor

HardwareSerial uwb(2);

// ----------- QUEUE HANDLES -----------
QueueHandle_t positionQueue;

void setup() {
  Serial.begin(115200);
  Serial.println("\nFireBeetle UWB Tag System Starting...");

  uwb.begin(921600, SERIAL_8N1, UWB_RX, UWB_TX);
  delay(50);
  configureTag(uwb);

  // Queues
  positionQueue = xQueueCreate(1, sizeof(Position));

  // ===== HANDLE MQTT =====
  wifiConnect();
  mqttClient.setMqttClientName(clientID);
  mqttClient.enableLastWillMessage("/will", "esp32-client-paddle went offline", false);

  String mqttBrokerURL = String(mqtt_broker);
  mqttClient.setURL(mqttBrokerURL.c_str(), 8883, "", "");
  mqttClient.setCaCert(caCert);
  mqttClient.setClientCert(clientCert);
  mqttClient.setKey(clientKey);
  mqttClient.loopStart();

  xTaskCreatePinnedToCore(
    mqttTask,
    "MQTT Task",
    6000,
    NULL,
    1,
    NULL,
    0);

  xTaskCreatePinnedToCore(
  uwbTask,
  "UWB Task",
  6000,
  NULL,
  2,
  NULL,
  1);
}

void mqttTask(void *pvParameters) {
  Position pos;
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnect();
    }
    if (xQueueReceive(positionQueue, &pos, 0) == pdTRUE) {
      if (mqttClient.isConnected()) {
        std::string payload = formatPayload(pos.x, pos.y);
        mqttClient.publish(playerEspPublishTopic, payload, 0, false);
        Serial.print("Position: ");
        Serial.print(pos.x);
        Serial.print(" , ");
        Serial.println(pos.y);
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
  Position pos;
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
          computeXY_LS(d1, d2, x, y);
          // ----- EMA smoothing
          if (first_position) {
            pos.x = x;
            pos.y = y;
            first_position = false;
          } else {
            pos.x = alpha * x + (1 - alpha) * pos.x;
            pos.y = alpha * y + (1 - alpha) * pos.y;
          }
          xQueueSend(positionQueue, &pos, 0);
        }
      }
    }
  }
}

void loop() {
}