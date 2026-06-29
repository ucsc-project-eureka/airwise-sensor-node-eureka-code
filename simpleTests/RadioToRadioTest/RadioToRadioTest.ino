// Only runs on ESP32S3 Dev Module board for debug port access.
// onboard ESP32 sends message via wifi to another onboard esp32 every five seconds.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WAIT_TIME 5000
#define SECOND_TIME 1000


uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t sinkMAC[6];
bool sinkMACKnown = false;

// typedefs ---------------------------------------------------------------
enum messageType : uint8_t {
  DISCOVERY = 1,
  JOIN_REQUEST,
  TDMA_SCHEDULE,
  SENSOR_DATA,
  AGGREGATE_DATA
};

struct sensorDataPacket_t {
  uint8_t type;
  float temperature;
  float humidity;
  uint16_t soilMoisture;
  unsigned long timestamp;
};

// MAIN --------------------------------------------------------------
int lastTime = 0;
int currentTime = 0;
int messageCount = 0;
// construct generic data.
  sensorDataPacket_t myData = {
    .type = SENSOR_DATA,
    .temperature = 10,
    .humidity = 20,
    .soilMoisture = 300,
    .timestamp = 0
  };


void setup(){
  Serial.begin(9600);
  while(!Serial){;}
  Serial.println("Serial connected!");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  if(esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init failed!");
    return;
  }

  Serial.println("ESP32 radio serial ready");
}

void loop() {
  // ping an outside ESP32 with hellos. wait every five seconds.
  while (currentTime - lastTime < WAIT_TIME){
    currentTime = millis();
  }

  // execute sending task. broadcast for ease of reception.
  // send ptr to message.
  esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));


  

  // once they are equal, set to equal. then restart loop iteration.
  lastTime = currentTime;

}
