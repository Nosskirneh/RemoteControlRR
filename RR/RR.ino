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

#define MOTOR_SPEED  9
#define MOTOR_CW     6
#define MOTOR_CCW    5


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
    Serial.print("Listening radio on ");
    Serial.print(RADIO_CE);
    Serial.print(", ");
    Serial.println(RADIO_CSN);

    // Potentiometer
    potentiometer.begin(POTENTIOMETER_CS);
    potentiometer.setWiper(0);

    // Motor
    pinMode(MOTOR_SPEED, OUTPUT);
    pinMode(MOTOR_CW, OUTPUT);
    pinMode(MOTOR_CCW, OUTPUT);
}

// Fetch the next radio message
int nextRadioMessage() {
    int message;
    radio.read(&message, sizeof(message));
    return message;
}

// Given an angle (0 - 180 degrees), either turn left, right or not at all
// To be replaced with a PID
void updateSteeringValue(int angle) {
    if (angle >= 0 && angle < 90) { // Right
        digitalWrite(MOTOR_CW, HIGH);
        digitalWrite(MOTOR_CCW, LOW);
        int speed = map(angle, 90, 0, 0, 255);
        analogWrite(MOTOR_SPEED, speed);
    } else if (angle > 90 && angle <= 180) { // Left
        digitalWrite(MOTOR_CW, LOW);
        digitalWrite(MOTOR_CCW, HIGH);
        int speed = map(angle, 90, 180, 0, 255);
        analogWrite(MOTOR_SPEED, speed);
    } else { // Center
        digitalWrite(MOTOR_CW, LOW);
        digitalWrite(MOTOR_CCW, LOW);
        analogWrite(MOTOR_SPEED, 0);
    }
}

// Send acceleration value to the digital potentiometer
void updateAccelerationValue(int acc) {
    if (acc >= 0 && acc <= 100) {
        double wiper = percentageToStep(acc);
        potentiometer.setWiper(wiper);
    }
}

// Help method that returns the value that the ECM of the Rescue Runner
double percentageToStep(double percentage) {
    return percentage * 0.88 + 20;
}

// Check for new messages and process each of them
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
            int angle = message & 0xFF;
            updateSteeringValue(angle);
            Serial.print("Read steering message: ");
            Serial.println(angle, DEC);
        } else if (header == Accelerate) {
            mode = RemoteMode;
            int acc = message & 0xFF;
            updateAccelerationValue(acc);
            Serial.print("Read accelerate message: ");
            Serial.println(acc);
        }
    } else if (mode != ManualMode && millis() - lastMessageReceivedTime > 5000) {
        Serial.println("No radio messages received for 5 seconds, falling back to manual mode!");
        mode = ManualMode;
        updateAccelerationValue(0);
        updateSteeringValue(90);
    }
}

void loop() {
    delay(100);
    readRadio();
}
