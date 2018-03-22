#include <SPI.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

typedef enum MessageType {
    ManualModeMsg = 0b00000000,
    RemoteModeMsg = 0b00000001,
    SteerMsg      = 0b00000100,
    AccelerateMsg = 0b00000110
} MessageType;


void setup() {
    Serial.begin(9600);
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
}

void loop() {
    delay(1000);
    Serial.println("Trying to read...");
    if (radio.available()) {
        int message;
        radio.read(&message, sizeof(message));
        Serial.println(message, BIN);

        // Mask out the header (first 8 bits)
        int header = message >> 8 & 0xFF;
        if (header == SteerMsg) {
            Serial.println("Read steering message!");
            int angle = message & 0xFF;
            Serial.println(angle,DEC);            
        } else if (header == AccelerateMsg) {
            Serial.println("Read accelerate message!");
            int acc = message & 0xFF;
            Serial.println(acc);
        }
    }
}
