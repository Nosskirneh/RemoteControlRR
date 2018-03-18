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
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
}

// Replace this with joystick
void readSerial() {
    if (Serial.available()) {
        char c = toupper(Serial.read());
        byte message = 0;
        if (c == 'R') {
            Serial.println("Sending right message!");
            message = SteerRight;
        } else if (c == 'L') {
            Serial.println("Sending left message!");
            message = SteerLeft;
        } else if (c == 'A') {
            Serial.println("Sending accelerate message!");
            message = Accelerate;
        }

        Serial.println(message);
        radio.write(&message, sizeof(message));
    }
}

void loop() {
    delay(1000);
    Serial.println("Waiting for input...");

    readSerial();
}
