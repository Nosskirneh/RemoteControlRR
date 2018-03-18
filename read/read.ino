#include <SPI.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

typedef enum MessageType {
    ManualMode = 0b00000000,
    RemoteMode = 0b00100000,
    SteerRight = 0b10000000,
    SteerLeft  = 0b10100000,
    Accelerate = 0b11000000,
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
    Serial.println("trying to read...");
    if (radio.available()) {
        byte text;
        radio.read(&text, sizeof(text));
        Serial.println(text);

        if (text == SteerRight) {
            Serial.println("Read right message!");
        } else if (text == SteerLeft) {
            Serial.println("Read left message!");
        } else if (text == Accelerate) {
            Serial.println("Read accelerate message!");
        }
    }
}
