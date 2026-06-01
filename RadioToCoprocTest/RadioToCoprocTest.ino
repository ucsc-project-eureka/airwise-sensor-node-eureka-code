// Heltec wireless shell (V3) board.

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

// Pins/custom serial port for the ESP32 - Credit, Airwise team
HardwareSerial COPROC_SERIAL(1);

// MAIN --------------------------------------------------------------

int startTime;
int currentTime;
void setup() {
  Serial.begin(9600);
  COPROC_SERIAL.begin(9600,SERIAL_8N1,ESP_PIN_TX,ESP_PIN_TX);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);

  if(esp_now_init() != ESP_OK){
    return;
  }
  Serial.println("ESP32 radio serial ready");
  startTime = millis();
  currentTime = startTime;
}

void loop() {
  currentTime = millis();

  // send the pulse via serial.
  if((currentTime - startTime)%WAIT_TIME == 0){
    COPROC_SERIAL.println("Hello Coproc from ESP32");
    Serial.println("Hello Coproc sent!\n");
  }
  // Otherwise, add the loading dots every second.
  else if((currentTime-startTime)%SECOND_TIME == 0){
    Serial.println(".");
  }
  // // Read whatever's sent back.
  // if(COPROC_SERIAL.available()){
  //   String input = COPROC_SERIAL.readStringUntil('\n');
  //   input.trim();
  //   Serial.println("Recieved message on ESP32Serial:");
  //   Serial.println(input);
  // }

}
