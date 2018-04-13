/*
    RescueRunner:
    To prevent loss of message when trying to change mode, it will automatically
    switch mode on accelerate and steering messages. After 5 seconds of no
    messages received, it will timeout and fallback to manual mode.
*/


#include <SPI.h>
#include <RF24.h>
#include <MCP41_Simple.h>
#include <RemoteControlRRLib.h>

#define POTENTIOMETER_CS 53

RF24 radio(RADIO_CE, RADIO_CSN);
const byte address[6] = "00001";

MCP41_Simple potentiometer;

int mode = ManualMode;

void setup() {
    Serial.begin(9600);
    // Radio
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();

    // Potentiometer
    potentiometer.begin(POTENTIOMETER_CS);
    potentiometer.setWiper(0);
}

int nextRadioMessage() {
    int message;
    radio.read(&message, sizeof(message));
    return message;
}

void updateAccelerationValue(int acc) {
    if (acc >= 0 && acc <= 100) {
        double wiper = percentageToStep(acc);
        potentiometer.setWiper(wiper);
    }
}

double percentageToStep(double percentage) {
    return percentage * 0.88 + 20;
}

void readRadio() {
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
            Serial.print("Read steering message: ");
            int angle = message & 0xFF;
            Serial.println(angle, DEC);
        } else if (header == Accelerate) {
            mode = RemoteMode;
            Serial.print("Read accelerate message: ");
            int acc = message & 0xFF;
            updateAccelerationValue(acc);
            Serial.println(acc);
        }
    } else if (mode != ManualMode && millis() - lastMessageReceivedTime > 5000) {
        Serial.println("No radio messages received for 5 seconds, falling back to manual mode!");
        mode = ManualMode;
        updateAccelerationValue(0);
    }
}

void loop() {
    delay(100);
    readRadio();
}
