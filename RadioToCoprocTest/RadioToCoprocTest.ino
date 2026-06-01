// Only runs on ESP32S3 Dev Module board for debug port access.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#define WAIT_TIME 5000
#define SECOND_TIME 1000

uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t sinkMAC[6];
bool sinkMACKnown = false;

// From Airwise's ESP32 UART connections.
#define ESP_PIN_TX 16
#define ESP_PIN_RX 15

// Pins/custom serial port for the ESP32
HardwareSerial COPROC_SERIAL(1);

// MAIN --------------------------------------------------------------

int lastTime = 0;
int currentTime = 0;
void setup() {
  Serial.begin(9600);
  while(!Serial){;}
  Serial.println("Serial connected!");

  COPROC_SERIAL.begin(9600,SERIAL_8N1,ESP_PIN_RX,ESP_PIN_TX);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  if(esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init failed!");
    return;
  }

  Serial.println("ESP32 radio serial ready");
}

void loop() {
  currentTime = millis();

  // send the pulse via serial.
  if((currentTime - lastTime)>=WAIT_TIME){
    COPROC_SERIAL.println("Hello Coproc from ESP32");
    Serial.println("Hello Coproc sent!\n");
    lastTime = currentTime;
  }
  // // Read whatever's sent back.
  // if(COPROC_SERIAL.available()){
  //   String input = COPROC_SERIAL.readStringUntil('\n');
  //   input.trim();
  //   Serial.println("Recieved message on ESP32Serial:");
  //   Serial.println(input);
  // }

}
