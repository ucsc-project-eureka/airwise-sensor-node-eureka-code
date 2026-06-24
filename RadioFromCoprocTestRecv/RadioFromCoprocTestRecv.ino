// Radio recieves messages from the coproc and reprints what it got in its debug serial.

/*
 * - Mode: TEXTMSG
 * - Baud rate: 9600
 * - RX GPIO: 44
 * - TX GPIO: 43
 */

#define DEBUG_PORT Serial
#define ESP_PORT  Serial1

constexpr uint32_t DEBUG_BAUD = 115200;
constexpr uint32_t ESP_BAUD = 9600;

constexpr size_t RECEIVE_BUFFER_SIZE = 256;

char receiveBuffer[RECEIVE_BUFFER_SIZE];
size_t receiveLength = 0;

void setup() {

  DEBUG_PORT.begin(DEBUG_BAUD);
  uint32_t start = millis();
  while (!DEBUG_PORT && (millis() - start < 3000));

  DEBUG_PORT.println();
  DEBUG_PORT.print("Debug serial1 on, baud rate: ");
  DEBUG_PORT.println(DEBUG_BAUD);

  ESP_PORT.begin(ESP_BAUD,SERIAL_8N1, 44, 43);
  DEBUG_PORT.println();
  DEBUG_PORT.print("ESP UART on, baud rate: ");
  DEBUG_PORT.println(ESP_BAUD);}

void loop() {
  while (ESP_PORT.available()) {
    char c = ESP_PORT.read();

    // raw output
    // DEBUG_PORT.write(c);

    if (c == '\r') {
      continue;
    }

    //message complete, marked by newline
    if (c == '\n') {
      if (receiveLength > 0) {
        receiveBuffer[receiveLength] = '\0';

        DEBUG_PORT.print("Received from coproc: ");
        DEBUG_PORT.println(receiveBuffer);

        receiveLength = 0;
      }

      continue;
    }

    //add next char to buffer
    if (receiveLength < RECEIVE_BUFFER_SIZE - 1) {
      receiveBuffer[receiveLength++] = c;
    } else {
      DEBUG_PORT.println("BUFFER OVERFLOW, CLEARING");
      receiveLength = 0;
    }
  }
}