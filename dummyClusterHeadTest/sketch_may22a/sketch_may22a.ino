/*
NOTE: This should be compiled with the Heltec Wireless Shell (V3) in order to get responses from the esp32.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define MAX_SENSOR_NODES 3
#define TDMA_SLOT_TIME 1000
#define JOIN_REQUEST_TIMEOUT 1000
#define SENSOR_RESPONSE_TIMEOUT (TDMA_SLOT_TIME * MAX_SENSOR_NODES)

uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t sinkMAC[6];
bool sinkMACKnown = false;

const unsigned long packetInterval = 3000;  // Time in milliseconds
unsigned long lastPacketSentTime = 0;

// Default C++ enum values are type int
enum messageType : uint8_t {
  DISCOVERY = 1,
  JOIN_REQUEST,
  TDMA_SCHEDULE,
  SENSOR_DATA,
  AGGREGATE_DATA
};

// Packet structures
struct discoveryPacket_t {
  uint8_t type;      // Packet type identifier
  uint8_t hopCount;  // Hop count away from the sink
  uint8_t roundCounter;
};

struct joinRequestPacket_t {
  uint8_t type;
};

struct tdmaSchedulePacket_t {
  uint8_t type;
  uint8_t macs[MAX_SENSOR_NODES][6];  // List of sensor node MACs
};

struct sensorDataPacket_t {
  uint8_t type;
  float temperature;
  float humidity;
  uint16_t soilMoisture;
  unsigned long timestamp;
};

struct aggregateDataPacket_t {
  uint8_t type;
  uint8_t macs[MAX_SENSOR_NODES][6];
  float temperatures[MAX_SENSOR_NODES];
  float humidities[MAX_SENSOR_NODES];
  uint16_t soilMoistures[MAX_SENSOR_NODES];
  unsigned long timestamps[MAX_SENSOR_NODES];
  uint8_t readingsCount;
};

// Global variables
unsigned long discoverySentTime = 0;
unsigned long scheduleSentTime = 0;

bool waitingForJoinRequests = false;
bool waitingForSensorData = false;

uint8_t sensorNodeMACs[MAX_SENSOR_NODES][6];
uint8_t sensorNodeCount = 0;

sensorDataPacket_t sensorData[MAX_SENSOR_NODES];
bool sensorDataReceived[MAX_SENSOR_NODES] = { false };

void handleJoinRequest(const uint8_t *senderMAC) {

  // Check if MAC is already recorded
  for (uint8_t i = 0; i < sensorNodeCount; ++i) {
    Serial.println("Sensor node MAC address is already recorded");
    if (memcmp(sensorNodeMACs[i], senderMAC, 6) == 0) return;
  }

  // Store MAC address
  if (sensorNodeCount < MAX_SENSOR_NODES) {
    Serial.println("Storing sensor node MAC address");
    memcpy(sensorNodeMACs[sensorNodeCount++], senderMAC, 6);
  }
}

void handleSensorData(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(sensorDataPacket_t)) {
    Serial.println("Invalid SENSOR_DATA packet size");
    return;
  }

  for (uint8_t i = 0; i < sensorNodeCount; i++) {
    if (memcmp(mac, sensorNodeMACs[i], 6) == 0) {
      memcpy(&sensorData[i], incomingData, sizeof(sensorDataPacket_t));
      sensorDataReceived[i] = true;

      Serial.printf("Stored SENSOR_DATA from node %d\n", i);
      Serial.printf("  Temperature: %.2f °C\n", sensorData[i].temperature);
      Serial.printf("  Humidity: %.2f %%\n", sensorData[i].humidity);
      Serial.printf("  Soil Moisture: %u\n", sensorData[i].soilMoisture);
      return;
    }
  }

  Serial.println("SENSOR_DATA received from unknown MAC.");
}

void sendTDMASchedule() {
  tdmaSchedulePacket_t tdmaSchedulePacket;
  tdmaSchedulePacket.type = TDMA_SCHEDULE;

  Serial.println("Preparing TDMA Schedule:");

  for (int i = 0; i < sensorNodeCount; i++) {
    memcpy(tdmaSchedulePacket.macs[i], sensorNodeMACs[i], 6);

    // Print each MAC with slot index
    Serial.printf("  Slot %d -> MAC: ", i);
    for (int j = 0; j < 6; j++) {
      Serial.printf("%02X", tdmaSchedulePacket.macs[i][j]);
      if (j < 5) Serial.print(":");
    }
    Serial.println();
  }

  esp_now_send(broadcastAddress, (uint8_t *)&tdmaSchedulePacket, sizeof(tdmaSchedulePacket));
  Serial.println("TDMA Schedule sent to all nodes.");

  // Prepare to receive data
  scheduleSentTime = millis();
  waitingForSensorData = true;

  // Reset sensor data received flags
  for (int i = 0; i < MAX_SENSOR_NODES; i++) {
    sensorDataReceived[i] = false;
  }
}

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void OnDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  // Determine type of packet received
  // get the source address.
  const uint8_t* senderMac = recvInfo->src_addr;
  uint8_t packetType = incomingData[0];

  switch (packetType) {
    case JOIN_REQUEST:
      handleJoinRequest(senderMac);
      break;

    case SENSOR_DATA:
      handleSensorData(senderMac, incomingData, len);
      break;

    default:
      Serial.println("Unknown packet type received");
      break;
  }
}

int startTime;
int currentTime;
int sendTime;
void setup(){
  Serial.begin(115200);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  if(esp_now_init() != ESP_OK){
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
  Serial.println("\nClusterhead stand-in ready.");
  startTime = millis();
}
 
void loop(){
  currentTime = millis();
  // every five seconds, send a "give me data" ping.
  if((currentTime-sendTime)>=5000){
    discoveryPacket_t giveMeData = {
      .type = DISCOVERY,
      .hopCount = 1};
      sendTime = currentTime;
      esp_now_send(broadcastAddress,(uint8_t*)&giveMeData,sizeof(discoveryPacket_t));
      Serial.println("Sent GIVE DATA!");
      discoverySentTime = millis();
      waitingForJoinRequests = true;     
  }
  if (waitingForJoinRequests && millis() - discoverySentTime > JOIN_REQUEST_TIMEOUT) {
    Serial.println("Finished waiting for JOIN_REQUESTs.");
    waitingForJoinRequests = false;

    // Send TDMA schedule
    sendTDMASchedule();
  }
  if (waitingForSensorData && millis() - scheduleSentTime > SENSOR_RESPONSE_TIMEOUT) {
    Serial.println("Finished waiting for JOIN_REQUESTs.");
    waitingForSensorData = false;
    Serial.println("Data Recieved:");
    Serial.printf("  Temperature: %.2f °C\n", sensorData[0].temperature);
    Serial.printf("  Humidity: %.2f %%\n", sensorData[0].humidity);
    Serial.printf("  Soil Moisture: %u\n", sensorData[0].soilMoisture);
  }
}