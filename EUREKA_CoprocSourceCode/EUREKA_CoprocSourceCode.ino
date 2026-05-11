#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

// MODS
#include <Adafruit_seesaw.h>
Adafruit_seesaw ss;             // Soil sensor.
#include <Adafruit_INA3221.h>
#include "Adafruit_BME680.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h" // for M10S GPS interfacing.
#include "SdFat.h"
#include "wiring_private.h" // Necessary for pin peripheral multiplexing (from Sankie)
/*
          ^ Note: SoftwareSerial.h on arduino also allows this functionality 
for more expensive time/no simultaneous UART transmissions.*/ 

// SD Card Setup (From Sankie)
// ── Pin definitions (SAMD21) (SD) ──────────────────────────────
#define SD_MISO_PIN 22
#define SD_MOSI_PIN 23
#define SD_SCK_PIN  24
#define SD_CS_PIN   38

// ── Software SPI (SD) ──────────────────────────────────────────
// Note: in default Arduino library:
// set SPI_DRIVER_SELECT 2 in SdFatConfig.h to set as a software SPI.
SoftSpiDriver<SD_MISO_PIN, SD_MOSI_PIN, SD_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4), &softSpi)

// ── Globals ───────────────────────────────────────────────
SdCardFactory cardFactory;
SdCard* card = nullptr;
uint8_t sectorBuf[512];

// Other setup pinouts --------------------------------------
// Use DEBUG_PORT for the Native Port
#define DEBUG_PORT SerialUSB
SFE_UBLOX_GNSS myGNSS;
#define SOIL_I2C 0x36

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

// From Airwise's ESP32 UART connections.
#define ESP_PIN_TX 15
#define ESP_PIN_RX 16

// Reference values for sensor data processing.
#define SEALEVELPRESSURE_HPA (1013.25)

// Reference all appropriate fields.
#define wirePort Wire               // I2C Bus port name.
Adafruit_BME680 bme(&wirePort);     // I2C
Adafruit_INA3221 ina3221;

// Pins/custom serial port for the ESP32 - Credit, Airwise team
// Create a new Serial instance for SERCOM0 -------------------------------------
Uart ESP32Serial(&sercom0, ESP_PIN_RX, ESP_PIN_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2);

// Packet definitions - unclear if still useful because all coms are text/string prints.
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

// Helpers ----------------------------------------------------------------------

// Print functions - Sensors

// Read data from BME680. Prints all relevant data from the reading to DEBUG_PORT.
void serialPrintBMEData(void){
  DEBUG_PORT.println(F("BME680 test"));

  DEBUG_PORT.print("Temperature = ");
  DEBUG_PORT.print(bme.temperature);
  DEBUG_PORT.println(" *C");

  DEBUG_PORT.print("Pressure = ");
  DEBUG_PORT.print(bme.pressure / 100.0);
  DEBUG_PORT.println(" hPa");

  DEBUG_PORT.print("Humidity = ");
  DEBUG_PORT.print(bme.humidity);
  DEBUG_PORT.println(" %");

  DEBUG_PORT.print("Gas = ");
  DEBUG_PORT.print(bme.gas_resistance / 1000.0);
  DEBUG_PORT.println(" KOhms");

  DEBUG_PORT.print("Approx. Altitude = ");
  DEBUG_PORT.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  DEBUG_PORT.println(" m");

  DEBUG_PORT.println();
  return;
}

// Read data from INA3221. Print all relevant data from reading to Native USB.
void serialPrintINAData(void){
  DEBUG_PORT.println("\n--- Power Monitor Data ---");

  for (uint8_t i = 0; i < 3; i++) {
    float voltage_V = ina3221.getBusVoltage(i);
    float current_A = ina3221.getCurrentAmps(i);
    float current_mA = current_A * 1000.0;

    DEBUG_PORT.print("Channel ");
    DEBUG_PORT.print(i + 1); 
    DEBUG_PORT.print(": ");
    DEBUG_PORT.print(voltage_V, 3); 
    DEBUG_PORT.print(" V | ");
    DEBUG_PORT.print(current_mA, 2);
    DEBUG_PORT.println(" mA");
  }

  // Ensure PA17 remains LOW
  PORT->Group[0].OUTCLR.reg = PORT_PA17;
}

// Checking functions.

// Check RX on Arduino zero from the DEBUG_PORT, echo back what is recieved to a print check
void checkCoprocConnection(void){
  if (DEBUG_PORT.available()){
    DEBUG_PORT.println("\nCoproc Test: Coproc is reading from DEBUG_PORT input");
    String input = DEBUG_PORT.readStringUntil('\n');
    if (DEBUG_PORT.availableForWrite()){
      DEBUG_PORT.println("Coproc recieved: ");
      DEBUG_PORT.print(input);
    }
  }
  return;
}

// Check power monitoring.
void checkBme680(void){
  // Check initialization of every sensor.
  if (!bme.begin()) {
    DEBUG_PORT.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  else{
    // Set up oversampling and filter initialization for BME
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
    if (! bme.performReading()) {
      DEBUG_PORT.println("Failed to perform reading :(");
      return;}
    else{
      serialPrintBMEData();
    }
  }
  return;
}

void checkIna3221(void){
  if(!ina3221.begin()){
    DEBUG_PORT.println("Could not find ina chip, check wiring!");
    while (1) delay(10); // Wait 10 milliseconds to account for fluctuations
  }
  else{
    // 1. Set PA17 to LOW immediately via direct register access
    // Ensure the pin is an output
    PORT->Group[0].DIRSET.reg = PORT_PA17; 
    // Clear the pin (Set to LOW)
    PORT->Group[0].OUTCLR.reg = PORT_PA17;

    DEBUG_PORT.begin(115200);

    DEBUG_PORT.println("INA3221 port found and initialized!");
    serialPrintINAData();
  }
}

// Checking GPS sensor activation and communication
void checkM10S(void){
  DEBUG_PORT.println("\nM10S GPS Test:\n");
  if (myGNSS.begin() == false){
    DEBUG_PORT.println("u-blox GNSS module not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }


  //This will pipe all NMEA sentences to the serial port so we can see them
  DEBUG_PORT.println("u-blox GNSS module detected at default I2C address.");
  myGNSS.setI2COutput(COM_TYPE_NMEA);
  myGNSS.setNMEAOutputPort(DEBUG_PORT);
  return;
}

// Check SD access.
void checkSD(void){
  DEBUG_PORT.println("=== FAT32 Formatter ===");

  card = cardFactory.newCard(SD_CONFIG);
  if (!card || card->errorCode()) {
    DEBUG_PORT.println("Card init failed. Check wiring.");
    return;
  }
  DEBUG_PORT.println("Card detected.");

  uint32_t sectors = card->sectorCount();
  if (sectors == 0) {
    DEBUG_PORT.println("Could not read card sector count. Aborting.");
    return;
  }

  uint32_t sizeMB = sectors / 2048;
  DEBUG_PORT.print("Card size: ");
  DEBUG_PORT.print(sizeMB);
  DEBUG_PORT.println(" MB");

  if (sizeMB > 32768) {
    DEBUG_PORT.println("Card >32GB, aborting. This sketch is FAT32 only.");
    return;
  }

  DEBUG_PORT.println("Formatting as FAT32... (may take up to 60 seconds)");
  FatFormatter formatter;
  if (!formatter.format(card, sectorBuf, &DEBUG_PORT)) {
    DEBUG_PORT.println("Format FAILED.");
    return;
  }

  DEBUG_PORT.println("=== Format complete! Flash your main sketch now. ===");
  return;
}

void checkSoilSensor(void){
  DEBUG_PORT.println("seesaw Soil Sensor check:");
  if (!ss.begin(SOIL_I2C)) {
    DEBUG_PORT.println("ERROR! seesaw not found");
    // while(1) delay(1);
  } else {
    DEBUG_PORT.print("seesaw started! version: ");
    DEBUG_PORT.println(ss.getVersion(), HEX);
    float tempC = ss.getTemp();
    // Note: Capactative "touch" is the moisture level detected.
    uint16_t capread = ss.touchRead(0);
    
    DEBUG_PORT.print("Temperature: "); DEBUG_PORT.print(tempC); DEBUG_PORT.println("*C");
    DEBUG_PORT.print("Capacitive: "); DEBUG_PORT.println(capread);
  }
  return;
}

// Report device address over I2C.
void scanI2CWire(void){
  DEBUG_PORT.println("Scanning I2C bus...");
  for (byte address = 1; address < 127; address++) {
    wirePort.beginTransmission(address);
    byte error = wirePort.endTransmission();
    if (error == 0) {
      DEBUG_PORT.print("Device found at address 0x");
      if (address < 16) DEBUG_PORT.print("0");
      DEBUG_PORT.println(address, HEX);
    }
  }
  DEBUG_PORT.println("Done.");
  return;
}

// NOTE: No BMV080 module was installed on the board used for testing, hence no BMV080 interfacing included here.

// MAIN --------------------------------------------------------------------------

// Global variables: Mainly time keeping markers and flags.
int startTime;
int currentTime;
bool getDataFlag = false;

void setup(){
  DEBUG_PORT.begin(115200);

  while (!DEBUG_PORT);
  DEBUG_PORT.println("Debug Serial initialized!");

  // Get UART connecting coproc and esp32 online.
  ESP32Serial.begin(9600); // UART, esp32->coproc and vice versa.
  // Assign the pins to the SERCOM peripheral (Peripheral C)
  pinPeripheral(ESP_PIN_RX, PIO_SERCOM);
  pinPeripheral(ESP_PIN_TX, PIO_SERCOM);
  Serial.println("SAMD21 listening on PA16(TX) and PA15(RX)...");

  // Check functionality of SD card, format it as FAT32:
  // checkAndFormatSD();
  // Check initialization of every sensor relevant to EUREKA project.
  checkBme680();
  checkIna3221();
  checkM10S();
  checkSoilSensor(); // Note: I2C address not specified for soil sensor yet, this will be errored.

  startTime = millis();
  return;
}

void loop(){
  currentTime = millis() - startTime;
  if (ESP32Serial.available()){
    String input = ESP32Serial.readStringUntil('\n');
    input.trim();
    // NOTE: Assuming printed format for received "give data" message this way:
    if (input == "SENSOR_DATA"){
      getDataFlag = true;
      DEBUG_PORT.println("\nGET DATA command recieved!");
    }
  }
  if (getDataFlag){
    sensorDataPacket_t myData;
    myData.type = SENSOR_DATA;

    // NOTE: this assumes parsing on the other side will pick up string data sent in this format.
    DEBUG_PORT.println("\nSensor Data packet initialized!");
    ESP32Serial.println("SENSOR_DATA:");

    // get latest bme data.
    bme.performReading();
    myData.temperature = bme.temperature;
    myData.humidity = bme.humidity;
    // myData.soilMoisture = ss.touchRead(0); // Soil monitor not connected during testing, but if not hardcoded use this.
    // Soil moisture dummy value:
    myData.soilMoisture = 500;
    myData.timestamp = currentTime;

    // print check.
    DEBUG_PORT.println("Temperature = ");
    DEBUG_PORT.print(myData.temperature);
    DEBUG_PORT.println(" *C");
    
    ESP32Serial.println(myData.temperature);

    DEBUG_PORT.println("Humidity = ");
    DEBUG_PORT.print(myData.humidity);
    DEBUG_PORT.println(" %");
    
    ESP32Serial.println(myData.humidity);

    DEBUG_PORT.print("Soil Moisture = ");
    DEBUG_PORT.println(myData.soilMoisture);

    ESP32Serial.println(myData.soilMoisture);

    DEBUG_PORT.println("");
    DEBUG_PORT.print("Timestamp = ");
    DEBUG_PORT.print(myData.timestamp);
    DEBUG_PORT.println("");

    ESP32Serial.println(myData.timestamp);
    getDataFlag = false;
  }
}