#include <esp_now.h>
#include <WiFi.h>

#define MAX_SENSOR_NODES 3
#define TDMA_SLOT_TIME 1000

unsigned long scheduledSlotTime = 0;

bool hasJoined = false;
bool clusterheadMACKnown = false;
bool scheduleReceived = false;

uint8_t clusterheadMAC[6];
uint8_t mySlotIndex = 255;

enum messageType : uint8_t {
  DISCOVERY = 1,
  JOIN_REQUEST,
  TDMA_SCHEDULE,
  SENSOR_DATA,
  AGGREGATE_DATA
};

struct discoveryPacket_t {
  uint8_t type;
  uint8_t hopCount;
  uint8_t roundCounter;
};

struct joinRequestPacket_t {
  uint8_t type;
};

struct tdmaSchedulePacket_t {
  uint8_t type;
  uint8_t macs[MAX_SENSOR_NODES][6];
};

struct sensorDataPacket_t {
  uint8_t type;
  float temperature;
  float humidity;
  uint16_t soilMoisture;
  unsigned long timestamp;
};

esp_now_peer_info_t peerInfo;

// Helpers -----------------------------------------------------------------

// in case to let other sensors know.
void sendJoinRequest() {
  if (hasJoined) return;
  joinRequestPacket_t joinPacket = { JOIN_REQUEST };
  esp_now_send(clusterheadMAC, (uint8_t *)&joinPacket, sizeof(joinPacket));
}

// in case of received discovery packet.
void handleDiscoveryPacket(const uint8_t *senderMAC, const discoveryPacket_t *packet) {
  if (packet->hopCount == 0) return;
  if (!clusterheadMACKnown) {
    memcpy(clusterheadMAC, senderMAC, 6);
    clusterheadMACKnown = true;

    // Register clusterhead as peer now that we have its MAC
    memcpy(peerInfo.peer_addr, clusterheadMAC, 6);
    peerInfo.channel = 2;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }
  if (!hasJoined) {
    sendJoinRequest();
  }
}

// in case of received schedule packet.
void handleSchedulePacket(const uint8_t *senderMAC, const tdmaSchedulePacket_t *packet) {
  scheduleReceived = true;
  uint8_t myMAC[6];
  WiFi.macAddress(myMAC);
  for (int i = 0; i < MAX_SENSOR_NODES; i++) {
    if (memcmp(packet->macs[i], myMAC, 6) == 0) {
      mySlotIndex = i;
      hasJoined = true;
      scheduledSlotTime = millis() + (mySlotIndex * TDMA_SLOT_TIME);
      return;
    }
  }
  hasJoined = false;
  mySlotIndex = 255;
}

// Reads data from Serial UART prints and parses into packet.
bool getDataFromCoproc(sensorDataPacket_t* dataPacket) {
  if (Serial.available()) {
    String header = Serial.readStringUntil('\n');
    header.trim();
    if (header == "SENSOR_DATA:") {
      dataPacket->type        = SENSOR_DATA;
      dataPacket->temperature  = Serial.readStringUntil('\n').toFloat();
      dataPacket->humidity     = Serial.readStringUntil('\n').toFloat();
      dataPacket->soilMoisture = Serial.readStringUntil('\n').toInt();
      dataPacket->timestamp    = Serial.readStringUntil('\n').toInt();
      return true;
    }
  }
  return false;
}

// ESP32 correct callback
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  const uint8_t *senderMac = recv_info->src_addr; // source MAC address.
  uint8_t packetType = incomingData[0];
  switch (packetType) {
    case DISCOVERY:
      if (len >= sizeof(discoveryPacket_t))
        handleDiscoveryPacket(senderMac, (const discoveryPacket_t *)incomingData);
      break;
    case TDMA_SCHEDULE:
      if (len >= sizeof(tdmaSchedulePacket_t))
        handleSchedulePacket(senderMac, (const tdmaSchedulePacket_t *)incomingData);
      break;
    default:
      break;
  }
}

// MAIN --------------------------------------------------------------------

void setup(){
  Serial.begin(9600);

  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (scheduleReceived && hasJoined && millis() >= scheduledSlotTime){
    // Trigger the corproc to send sensor data to ESP32.
    Serial.println("SENSOR_DATA");
    // Wait for coproc to respond.
    sensorDataPacket_t dataPacket;
    unsigned long timeout = millis() + 2000;
    while (!getDataFromCoproc(&dataPacket) && millis() < timeout);
    // Only send if we actually got data
    if (millis() < timeout) {
      esp_now_send(clusterheadMAC, (uint8_t *)&dataPacket, sizeof(dataPacket));
    }
    scheduleReceived = false;
  }
}