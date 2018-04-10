/*
    RescueRunner:
    To prevent loss of message when trying to change mode, it will automatically
    switch mode on accelerate and steering messages. After 5 seconds of no
    messages received, it will timeout and fallback to manual mode.
*/


#include <SPI.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

// To be sent over radio
typedef enum MessageType {
    ManualMode = 0b00000000,
    RemoteMode = 0b00000001,
    Steer      = 0b00000100,
    Accelerate = 0b00000110
} MessageType;

int mode = ManualMode;

void setup() {
    Serial.begin(9600);
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
}

int nextRadioMessage() {
    int message;
    radio.read(&message, sizeof(message));
}

void readRadio() {
    Serial.println("Will read!");
    static unsigned long lastMessageReceivedTime = 0;
    if (radio.available()) {
        lastMessageReceivedTime = millis();
        int message = nextRadioMessage();
        Serial.println(message, BIN);

        // Mask out the header (first 8 bits)
        int header = message >> 8 & 0xFF;
        if (header == ManualMode || header == RemoteMode) {
            mode = header;
            Serial.print("Read changed mode to: ");
            Serial.println(mode);
        } else if (header == Steer) {
            mode = RemoteMode;
            Serial.println("Read steering message!");
            int angle = message & 0xFF;
            Serial.println(angle, DEC);
        } else if (header == Accelerate) {
            mode = RemoteMode;
            Serial.println("Read accelerate message!");
            int acc = message & 0xFF;
            Serial.println(acc);
        }
    } else if (mode != ManualMode && millis() - lastMessageReceivedTime > 5000) {
        Serial.println("No radio messages received for 5 seconds, falling back to manual mode!");
        mode = ManualMode;
    }
}

void loop() {
    delay(1000);
    readRadio();
}
