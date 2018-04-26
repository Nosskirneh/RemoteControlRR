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
#include <SD.h>

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

#define SD_PIN 4
#define LOG_FILENAME "log"
#define LOG_FILETYPE ".txt"


RF24 radio(RADIO_CE, RADIO_CSN);
MCP41_Simple potentiometer;

char logName[30];
File logFile;

static int mode = ManualMode;
static int steeringRef = -1;
static int acceleration = -1;
static bool logEnabled = true;

static byte benchmarkMode = BenchmarkDone;


void setup() {
    Serial.begin(9600);
    // Radio
    radio.begin();
    radio.openWritingPipe(RADIO_ADDRESS[1]);
    radio.openReadingPipe(1, RADIO_ADDRESS[0]);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.startListening();
    sprintf(logMsg, "Listening radio on pins: %d, %d", RADIO_CE, RADIO_CSN);
    DEBUG_PRINTLN(logMsg);

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

    // Logging
    if (!SD.begin(SD_PIN)) {
        Serial.println("SD card initialization failed!");
    } else {
        Serial.println("SD card init succeeded");
        sprintf(logName, "%s%s", LOG_FILENAME, LOG_FILETYPE);
        logFile = SD.open(logName, FILE_WRITE);
        if (logFile) {
            Serial.println("\n*** Logging begin ***");
            logToFile("millis(), steeringRef, steering sensor, acceleration");
        }
    }
}

// Send acceleration value to the digital potentiometer
int updateAccelerationValue(int acc) {
    if (acc >= 0 && acc <= 100) {
        acceleration = acc;
        potentiometer.setWiper(percentageToStep(acc));
    }
}

// Help method that returns the value that the ECM
// of the Rescue Runner wants to accelerate
double percentageToStep(double percentage) {
    return percentage * 0.88 + 20;
}

// Enable RemoteMode if it isn't enabled already
void enableRemoteMode() {
    if (mode != RemoteMode) {
        mode = RemoteMode;
        benchmarkMode = BenchmarkDone;
        sendACK(mode);

        turnRelaysOn();
    }
}

// Enable ManualMode if it isn't enabled already
void enableManualMode() {
    if (mode != ManualMode) {
        mode = ManualMode;
        benchmarkMode = BenchmarkDone;
        sendACK(mode);

        updateAccelerationValue(0);
        updateMotor(0);
        turnRelaysOff();
        resetPIDValues();
    }
}

void handleBenchmark(byte data) {
    sendACK(Benchmark);

    if (data == BenchmarkRight || data == BenchmarkLeft) {
        // First set speed to 0
        updateAccelerationValue(0);
        delay(1000);
        // Run benchmark
        benchmarkMode = BenchmarkInitiated;
        runBenchmark(data);
    } else {
        // Cancel
        benchmarkMode = BenchmarkDone;
    }
}

// Check for new messages and process each of them
void readRadio() {
    static unsigned long lastMessageReceivedTime = 0;
    if (radio.available()) {
        lastMessageReceivedTime = millis();
        int message = nextRadioMessage();

        // Mask out the header (first 8 bits)
        int header = message >> 8 & 0xFF;
        if (header == ManualMode) {
            enableManualMode();
            sprintf(logMsg, "Read change mode to: %d", mode);
            DEBUG_PRINTLN(logMsg);
        } else if (header == RemoteMode) {
            enableRemoteMode();
            sprintf(logMsg, "Read change mode to: %d", mode);
            DEBUG_PRINTLN(logMsg);
        } else if (header == Steer && benchmarkMode == BenchmarkDone) {
            enableRemoteMode();
            steeringRef = message & 0xFF;
            sprintf(logMsg, "Read steering message: %d", steeringRef);
            DEBUG_PRINTLN(logMsg);
        } else if (header == Accelerate && benchmarkMode == BenchmarkDone) {
            enableRemoteMode();
            int acc = message & 0xFF;
            updateAccelerationValue(acc);
            sprintf(logMsg, "Read accelerate message: %d", acc);
            DEBUG_PRINTLN(logMsg);
        } else if (header == SetLogging) {
            logEnabled = message & 0xFF;
            sendACK(header);
        } else if (header == NewLog) {
            int logNumber = message & 0xFF;
            sprintf(logName, "%s%d%s", LOG_FILENAME, logNumber, LOG_FILETYPE);
            Serial.println("\n*** Logging begin ***");
            logToFile("millis(), steeringRef, steering sensor, acceleration");
            sendACK(header);
        } else if (header == Benchmark) {
            handleBenchmark(message & 0xFF);
        }
    } else if (benchmarkMode != BenchmarkInitiated && mode != ManualMode &&
               millis() - lastMessageReceivedTime > 5000) {
        DEBUG_PRINTLN("No radio messages received for 5 seconds, falling back to manual mode!");
        enableManualMode();
    }
}

// Send radio message that the last message with header `header` was received.
void sendACK(int header) {
    int message = combine(ACK, header);
    sendRadioMessage(message);
}

void logToFile(const char *str) {
    logFile = SD.open(logName, FILE_WRITE);
    logFile.println(str);
    logFile.close();
}

void logToFile(double value) {
    logFile = SD.open(logName, FILE_WRITE);
    logFile.print(value);
    logFile.close();
}

// Turns Relay 1 to 6 off
void turnRelaysOff() {
    for (int i = RELAY1; i <= RELAY6; i++)
        digitalWrite(i, HIGH);
}

// Turns Relay 1 to 6 on
void turnRelaysOn() {
    for (int i = RELAY1; i <= RELAY6; i++)
        digitalWrite(i, LOW);
}

int getSteeringValue() {
    int value = analogRead(STEERING_SENSOR);
    return map(value, STEERING_SENSOR_LOW, STEERING_SENSOR_HIGH, 0, 255);
}

// Initial values
// Declared in the global scope to be able to reset them when switching to manual mode
static double P = 0.0;
static double I = 0.0;
static double D = 0.0;
static int e_old = 0;

double calculatePID() {
    static double output = 0;
    static unsigned long lastTimestamp = 0;

    if (millis() - lastTimestamp > PID_SAMPLE_TIME * 1000) {
        lastTimestamp = millis();

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

void resetPIDValues() {
    P = 0.0;
    I = 0.0;
    D = 0.0;
    e_old = 0;
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

void runBenchmark(boolean toRight) {
    sprintf(logMsg, "Benchmark (toRight: %d) has begun!", toRight);
    DEBUG_PRINTLN(logMsg);
    logToFile(logMsg);
    steeringRef = toRight ? 255 : 0;

    while (abs(steeringRef - getSteeringValue()) > 10) {
        sprintf(logMsg, "1: SteeringValue: %d, ref: %d", getSteeringValue(), steeringRef);
        DEBUG_PRINTLN(logMsg);
        updateMotor(calculatePID());

        // Should cancel?
        readRadio();
        if (benchmarkMode == BenchmarkDone) return;
    }

    delay(1000);

    // Motor is at end point
    steeringRef = toRight ? 0 : 255;
    double timeStart = millis();

    while (abs(steeringRef - getSteeringValue()) > 10) {
        sprintf(logMsg, "2: SteeringValue: %d, ref: %d", getSteeringValue(), steeringRef);
        DEBUG_PRINTLN(logMsg);
        updateMotor(calculatePID());

        // Should cancel?
        readRadio();
        if (benchmarkMode == BenchmarkDone) return;
    }

    double totalTime = millis() - timeStart;
    sprintf(logMsg, "End to end took: %d", totalTime / 1000);
    DEBUG_PRINTLN(logMsg);
    logToFile(logMsg);
    delay(1000);
}

void loop() {
    readRadio();

    if (mode == RemoteMode) {
        // Run PID here
        updateMotor(calculatePID());

        // Log
        if (logEnabled) {
            sprintf(logMsg, "%lu,\t%d,\t%d,\t%d", millis(), steeringRef, getSteeringValue(), acceleration);
            logToFile(logMsg);
            Serial.println(logMsg);   
        }
    }
}
