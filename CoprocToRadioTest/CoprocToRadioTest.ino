// Run on Arduino Zero Native USB port board.
#include <Wire.h>
#include <SPI.h>
#include "wiring_private.h" // Necessary for pin peripheral multiplexing (from Sankie)

#define WAIT_TIME 5000
#define SECOND_TIME 1000

// Other setup pinouts ------------------------------------------------------------
// Use DEBUG_PORT for the Native Port
#define DEBUG_PORT SerialUSB

// From Airwise's ESP32 UART connections.
#define ESP_PIN_TX 8
#define ESP_PIN_RX 9

// Pins/custom serial port for the ESP32 - Credit, Airwise team
// Create a new Serial instance for SERCOM0 ----------------------------------------
Uart ESP32Serial(&sercom0, ESP_PIN_RX, ESP_PIN_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2);

// Helpers -------------------------------------------------------------------------

// MAIN ----------------------------------------------------------------------------
int currentTime = 0;
int lastTime = 0;

void setup() {
  // turn on the radio from the coproc
  PORT->Group[0].DIRSET.reg = PORT_PA17;
  PORT->Group[0].OUTCLR.reg = PORT_PA17;

  DEBUG_PORT.begin(115200);

  while (!DEBUG_PORT);
  DEBUG_PORT.println("Debug Serial initialized!");

  // Get UART connecting coproc and esp32 online.
  ESP32Serial.begin(9600); // UART, esp32->coproc and vice versa.

  // Assign the pins to the SERCOM peripheral (Peripheral C)
  pinPeripheral(ESP_PIN_RX, PIO_SERCOM);
  pinPeripheral(ESP_PIN_TX, PIO_SERCOM);
  Serial.println("Coproc listening on PA16(TX) and PA15(RX)...");
}

void loop() {
  currentTime = millis();
  // Send message every five seconds.
  // fire every interval of WAIT_TIME

  // if(currentTime - lastTime>=WAIT_TIME){
  //   ESP32Serial.println("Hello ESP32 from Coproc!\n");
  //   DEBUG_PORT.println("Hello ESP32 sent!\n");
  //   lastTime = currentTime; // reset time markers.
  // }
  // // Otherwise, add the loading dots every second.
  // else if((currentTime-startTime)%SECOND_TIME == 0){
  //   DEBUG_PORT.println(".");
  // }

  // Read whatever's sent back.
  if(ESP32Serial.available()){
    String input = ESP32Serial.readStringUntil('\n');
    input.trim();
    DEBUG_PORT.println("Recieved message on ESP32Serial:");
    DEBUG_PORT.println(input);
  }

  // Ensure PA17 remains LOW
  PORT->Group[0].OUTCLR.reg = PORT_PA17;
}
