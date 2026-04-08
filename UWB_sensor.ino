#include <HardwareSerial.h>
#include <math.h>
#include "Mqtt.h"
#include "config.h"
#include "uwb_processing.h"

#define UWB_RX 26  // to IO5/RX on UWB sensor
#define UWB_TX 27  // to IO6/TX on UWB sensor

#define DEBUG // enables print statements for debugging 

HardwareSerial uwb(2);

// --------- GLOBAL VARIABLES ----------
bool hasGameStarted = true;

// ----------- QUEUE HANDLES -----------
QueueHandle_t positionQueue;
QueueHandle_t calibrationQueue;

void setup() {
  Serial.begin(115200);
  Serial.println("\nFireBeetle UWB Tag System Starting...");

  uwb.begin(921600, SERIAL_8N1, UWB_RX, UWB_TX);
  delay(50);
  configureTag(uwb);

  // Queues
  positionQueue = xQueueCreate(1, sizeof(Position));
  calibrationQueue = xQueueCreate(1, sizeof(bool));

  // ===== HANDLE MQTT =====
  wifiConnect();
  mqttClient.setMqttClientName(clientID);
  mqttClient.enableLastWillMessage("/will", "esp32-player-client went offline", false);

  // String mqttBrokerURL = String(mqtt_broker);
  // mqttClient.setURL(mqttBrokerURL.c_str(), 1883, "", "");

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
      if (mqttClient.isConnected() && hasGameStarted) {
        std::string payload = formatPayload(pos.x, pos.y);
        mqttClient.publish(playerEspPublishTopic, payload, 0, false);
        
        #ifdef DEBUG
        Serial.print("[MQTT] Position: ");
        Serial.print(pos.x);
        Serial.print(" , ");
        Serial.println(pos.y);
        #endif
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

  float d1_ema = 0;
  float d2_ema = 0;

  float d1 = 0.0, d2 = 0.0;
  bool calibrate = true;  // set true to perform calibration

  float offset_x = 0.0f;
  float offset_y = 0.0f;

  while (1) {
    // Check if calibration requested
    bool calibrateRequest = false;
    if (xQueueReceive(calibrationQueue, &calibrateRequest , 0) == pdTRUE) {
      calibrate = true;
      Serial.println("calibrate received");
    }

    if (uwb.available()) {
      String line = uwb.readStringUntil('\n');
      line.trim();

      #ifdef DEBUG
      // Serial.println(line);
      #endif

      String src;
      float dist;
      if (parseDistance(line, src, dist)) {
        if (src == "1111") d1 = dist;
        else if (src == "2222") d2 = dist;

        // Perform calibration if requested
        if (calibrate && d1 > 0 && d2 > 0) {
          bool done = collectCalibrationSample(d1, d2, offset_x, offset_y);

          float cal_x, cal_y;
          computeXY_LS(d1, d2, cal_x, cal_y);  // compute position AT calibration point
          
          if (done) calibrate = false;
          // Skip position computation until calibration is complete
          continue;
        }

        // Compute position
        if (!calibrate) {
          // Update median buffers
          d1_hist[hist_index] = d1;
          d2_hist[hist_index] = d2;
          hist_index = (hist_index + 1) % 3;

          // Compute median distances
          float d1_med = median3(d1_hist[0], d1_hist[1], d1_hist[2]);
          float d2_med = median3(d2_hist[0], d2_hist[1], d2_hist[2]);

          // ----- EMA smoothing
          if (first_position) {
            d1_ema = d1_med;
            d2_ema = d2_med;
            first_position = false;
          } else {
            d1_ema = alpha * d1_med + (1 - alpha) * d1_ema;
            d2_ema = alpha * d2_med + (1 - alpha) * d2_ema;
          }

          computeXY_LS(d1_ema, d2_ema, pos.x, pos.y);

          pos.x -= offset_x;
          pos.y -= offset_y;
          xQueueSend(positionQueue, &pos, 0);

          #ifdef DEBUG
          // Serial.print("Position: ");
          // Serial.print(pos.x);
          // Serial.print(" , ");
          // Serial.println(pos.y);
          #endif
        }
      }
    }
  }
}

float applyEMA(float raw, float &ema_val) {
  ema_val = alpha * raw + (1 - alpha) * ema_val;
  return ema_val;
}

void loop() {
}