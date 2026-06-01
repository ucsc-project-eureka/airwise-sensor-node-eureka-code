#include <Arduino.h>

HardwareSerial Coproc(1);

struct Pair {
    int rx;
    int tx;
};

Pair candidates[] = {
    {1, 2},
    {2, 1},
    {3, 4},
    {4, 5},
    {5, 4},
    {6, 7},
    {7, 6},
    {15, 16},
    {16, 15},
    {17, 18},
    {18, 17},
    {43, 44},
    {44, 43}
};

int currentPair = 0;
unsigned long lastSwitch = 0;

void beginPair()
{
    Coproc.end();

    Coproc.begin(
        9600,
        SERIAL_8N1,
        candidates[currentPair].rx,
        candidates[currentPair].tx
    );

    Serial.println();
    Serial.println("================================");
    Serial.printf(
        "Testing RX=%d TX=%d\n",
        candidates[currentPair].rx,
        candidates[currentPair].tx
    );
    Serial.println("================================");
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("ESP32 UART Scanner");

    beginPair();
}

void loop()
{
    while (Coproc.available())
    {
        char c = Coproc.read();
        Serial.write(c);
    }

    if (millis() - lastSwitch > 5000)
    {
        lastSwitch = millis();

        currentPair++;

        if (currentPair >= sizeof(candidates)/sizeof(candidates[0]))
        {
            currentPair = 0;
        }

        beginPair();
    }
}