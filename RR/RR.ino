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

#define MOTOR_SPEED  45 // motor card pin 9
#define MOTOR_CW     44 // motor card pin 6
#define MOTOR_CCW    46 // motor card pin 7

#define RELAY1  24
#define RELAY2  25
#define RELAY3  26
#define RELAY4  27
#define RELAY5  28
#define RELAY6  29
#define RELAY7  30
#define RELAY8  31

#define PID_Kp 1
#define PID_Ki 0.5
#define PID_Kd 0
#define PID_Tf 0.04
#define PID_SAMPLE_TIME 0.1 // seconds

#define STEERING_SENSOR      A11
#define STEERING_SENSOR_LOW  630  // 666
#define STEERING_SENSOR_HIGH 40   // 29


RF24 radio(RADIO_CE, RADIO_CSN);
MCP41_Simple potentiometer;

int mode = ManualMode;
int steeringRef = -1;

void setup() {
    Serial.begin(9600);
    // Radio
    radio.begin();
    radio.openReadingPipe(0, RADIO_ADDRESS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.startListening();
    DEBUG_PRINT("Listening radio on ");
    DEBUG_PRINT(RADIO_CE);
    DEBUG_PRINT(", ");
    DEBUG_PRINTLN(RADIO_CSN);

    // Potentiometer
    potentiometer.begin(POTENTIOMETER_CS);
    potentiometer.setWiper(0);

    // Motor
    pinMode(MOTOR_SPEED, OUTPUT);
    pinMode(MOTOR_CW, OUTPUT);
    pinMode(MOTOR_CCW, OUTPUT);

    // Relays
    for (int i = RELAY1; i <= RELAY8; i++)
        pinMode(i, OUTPUT);
    turnRelaysOff();

    // Steering sensor
    pinMode(STEERING_SENSOR, INPUT);

    //runBenchmark();
}

// Fetch the next radio message
int nextRadioMessage() {
    int message;
    radio.read(&message, sizeof(message));
    return message;
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

// Enable RemoteMode if it isn't enabled already
void enableRemoteMode() {
    if (mode != RemoteMode) {
        mode = RemoteMode;
        turnRelaysOn();
    }
}

// Enable ManualMode
void enableManualMode() {
    mode = ManualMode;
    updateAccelerationValue(0);
    updateMotor(0);
    turnRelaysOff();
}

// Check for new messages and process each of them
void readRadio() {
    static unsigned long lastMessageReceivedTime = 0;
    if (radio.available()) {
        lastMessageReceivedTime = millis();
        int message = nextRadioMessage();
        DEBUG_PRINTLN(message);

        // Mask out the header (first 8 bits)
        int header = message >> 8 & 0xFF;
        if (header == ManualMode) {
            DEBUG_PRINT("Read changed mode to: ");
            DEBUG_PRINTLN(mode);
            enableManualMode();
        } else if (header == RemoteMode) {
            enableRemoteMode();
            DEBUG_PRINT("Read changed mode to: ");
            DEBUG_PRINTLN(mode);
        } else if (header == Steer) {
            enableRemoteMode();
            steeringRef = message & 0xFF;
            DEBUG_PRINT("Read steering message: ");
            DEBUG_PRINTLN(steeringRef);
        } else if (header == Accelerate) {
            enableRemoteMode();
            int acc = message & 0xFF;
            updateAccelerationValue(acc);
            DEBUG_PRINT("Read accelerate message: ");
            DEBUG_PRINTLN(acc);
        }
    } else if (mode != ManualMode && millis() - lastMessageReceivedTime > 5000) {
        DEBUG_PRINTLN("No radio messages received for 5 seconds, falling back to manual mode!");
        enableManualMode();
    }
}

// Turns Relay 1 to 6 on
void turnRelaysOff() {
    for (int i = RELAY1; i <= RELAY6; i++)
        digitalWrite(i, HIGH);
}

// Turns Relay 1 to 6 off
void turnRelaysOn() {
    for (int i = RELAY1; i <= RELAY6; i++)
        digitalWrite(i, LOW);
}

int getSteeringValue() {
    int value = analogRead(STEERING_SENSOR);
    return map(value, STEERING_SENSOR_LOW, STEERING_SENSOR_HIGH, 0, 255);
}

double calculatePID() {
    static double output = 0;
    static unsigned long lastTimestamp = 0;

    if (millis() - lastTimestamp > PID_SAMPLE_TIME * 1000) {
        lastTimestamp = millis();
        // Initial values
        static double P = 0.0;
        static double I = 0.0;
        static double D = 0.0;

        static int e_old = 0;

        // Calculate the error
        int e = steeringRef - getSteeringValue();

        D = PID_Tf / (PID_Tf + PID_SAMPLE_TIME) * D + PID_Kd / (PID_Tf + PID_SAMPLE_TIME) * (e - e_old);
        P = PID_Kp * e;
        double u = P + D + I; // Calculate control output
        // Calculate next I value
        I = I + PID_Ki * PID_SAMPLE_TIME * e;

        e_old = e;

        // Make sure the calculated output is within limit
        return output = constrain(u, -255.0, 255.0);
    }
    return output;
}

void updateMotor(double speed) {
    if (speed > 0) { // Right
        //DEBUG_PRINT("Right: ");
        digitalWrite(MOTOR_CW, HIGH);
        digitalWrite(MOTOR_CCW, LOW);
    } else if (speed < 0) { // Left
        //DEBUG_PRINT("Left: ");
        digitalWrite(MOTOR_CW, LOW);
        digitalWrite(MOTOR_CCW, HIGH);
    } else { // Center
        //DEBUG_PRINT("Center: ");
        digitalWrite(MOTOR_CW, LOW);
        digitalWrite(MOTOR_CCW, LOW);
    }
    //DEBUG_PRINTLN(abs(speed));
    analogWrite(MOTOR_SPEED, abs(speed));
}

void runBenchmark() {
    DEBUG_PRINTLN("Benchmark has begun...");
    steeringRef = 0;

    while (abs(steeringRef - getSteeringValue()) > 10) {
        DEBUG_PRINT("1: SteeringValue :");
        DEBUG_PRINT(getSteeringValue());
        DEBUG_PRINT(", ");
        DEBUG_PRINT("ref: ");
        DEBUG_PRINTLN(steeringRef);
        updateMotor(calculatePID());
    }

    delay(1000);

    // Motor is at end point
    steeringRef = 255;
    double timeStart = millis();

    while (abs(steeringRef - getSteeringValue()) > 10) {
        DEBUG_PRINT("2: SteeringValue :");
        DEBUG_PRINT(getSteeringValue());
        DEBUG_PRINT(", ");
        DEBUG_PRINT("ref: ");
        DEBUG_PRINTLN(steeringRef);
        updateMotor(calculatePID());
    }

    double totalTime = millis() - timeStart;
    DEBUG_PRINT("End to end took ");
    DEBUG_PRINTLN(totalTime / 1000);
    delay(1000);
}

void loop() {
    readRadio();

    if (mode == RemoteMode) {
        // Run PID here
        updateMotor(calculatePID());
    }
}
