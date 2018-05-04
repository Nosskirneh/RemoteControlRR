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

unsigned int PID_Kp = 10;
unsigned int PID_Ki = 5;
unsigned int PID_Kd = 0;
#define PID_Tf 0.04
#define PID_SAMPLE_TIME 0.1 // seconds

#define STEERING_SENSOR      A11
#define STEERING_SENSOR_LOW  655 // max 670
#define STEERING_SENSOR_HIGH 10  // min 0

RF24 radio(RADIO_CE, RADIO_CSN);
MCP41_Simple potentiometer;

static int mode = ManualMode;
static int steeringRef = -1;
static int acceleration = -1;
static bool logEnabled = true;


void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);

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
    return (100 - percentage) * 0.88 + 20;
}

// Enable RemoteMode if it isn't enabled already
void enableRemoteMode() {
    if (mode != RemoteMode) {
        mode = RemoteMode;
        sendACK(mode);

        turnRelaysOn();
    }
}

// Enable ManualMode if it isn't enabled already
void enableManualMode() {
    if (mode != ManualMode) {
        mode = ManualMode;
        sendACK(mode);

        updateAccelerationValue(0);
        updateMotor(0);
        turnRelaysOff();
        resetPIDValues();
    }
}

void handleBenchmark(byte data) {
    sendACK(Benchmark);

    // First set speed to 0
    updateAccelerationValue(0);
    DEBUG_PRINTLN("Waiting for RescueRunner to slow down (1 s)...");
    pauseBenchmarkForTime(1000);

    // Run benchmark
    if (data == BenchmarkRightToLeft)
        runBenchmark(255, 0);
    else if  (data == BenchmarkLeftToRight)
        runBenchmark(0, 255);
    else if (data == BenchmarkRightToMid)
        runBenchmark(255, 128);
    else if (data == BenchmarkLeftToMid)
        runBenchmark(0, 128);
}

void pauseBenchmarkForTime(unsigned int delay) {
    unsigned long pauseStartTime = millis();
    while (millis() - pauseStartTime < delay) {
        updateMotor(calculatePID());
        log();
    }
}

void runBenchmark(byte startValue, byte endValue) {
    memset(logMsg, 0, sizeof(logMsg));
    sprintf(logMsg, "Benchmark has begun (%d to %d)", startValue, endValue);
    DEBUG_PRINTLN(logMsg);
    sendToLog();
    steeringRef = startValue;

    memset(logMsg, 0, sizeof(logMsg));
    while (abs(steeringRef - getSteeringValue()) > 5) {
        sprintf(logMsg, "1: SteeringValue: %d, ref: %d", getSteeringValue(), steeringRef);
        DEBUG_PRINTLN(logMsg);
        log();
        updateMotor(calculatePID());

        // Should cancel?
        if (shouldStopBenchmark()) return;
    }

    pauseBenchmarkForTime(500);

    // Motor is at end point
    steeringRef = endValue;
    memset(logMsg, 0, sizeof(logMsg));
    unsigned long timeStart = millis();

    bool hasReachedGoal = false;
    unsigned long timeSinceGoal = 0;
    while (timeSinceGoal == 0 || millis() - timeSinceGoal > 1000) {
        sprintf(logMsg, "2: SteeringValue: %d, ref: %d", getSteeringValue(), steeringRef);
        DEBUG_PRINTLN(logMsg);
        log();

        updateMotor(calculatePID());

        if (abs(steeringRef - getSteeringValue()) < 5) { // Reached goal?
            if (!hasReachedGoal) { // Do not overwrite the saved time on any following goal run
                hasReachedGoal = true;
                timeSinceGoal = millis(); // Save time
            }
        } else {
            hasReachedGoal = false;
            timeSinceGoal = 0;
        }

        // Should cancel?
        if (shouldStopBenchmark()) return;
    }

    memset(logMsg, 0, sizeof(logMsg));
    sprintf(logMsg, "End to end took: %ld ms", (timeSinceGoal - timeStart));
    DEBUG_PRINTLN(logMsg);
    sendToLog();
}

// Smaller version of readRadio to be called in the benchmark method
bool shouldStopBenchmark() {
    if (!radio.available())
        return false;

    int message = nextRadioMessage();
    return (message >> 8 & 0xFF == Benchmark);
}

// Check for new messages and process each of them
void readRadio() {
    static unsigned long lastMessageReceivedTime = 0;
    if (radio.available()) {
        lastMessageReceivedTime = millis();
        int message = nextRadioMessage();

        // Mask out the header (first 8 bits)
        int header = message >> 8 & 0xFF;
        int data = message & 0xFF;
        if (header == SetMode) {
            if (data == ManualMode)
                enableManualMode();
            else if (data == RemoteMode)
                enableRemoteMode();
            memset(logMsg, 0, sizeof(logMsg));
            sprintf(logMsg, "Read change mode to: %d", mode);
            DEBUG_PRINTLN(logMsg);
        } else if (header == Steer) {
            enableRemoteMode();
            steeringRef = data;
            memset(logMsg, 0, sizeof(logMsg));
            sprintf(logMsg, "Read steering message: %d", steeringRef);
            DEBUG_PRINTLN(logMsg);
        } else if (header == Accelerate) {
            enableRemoteMode();
            updateAccelerationValue(data);
            memset(logMsg, 0, sizeof(logMsg));
            sprintf(logMsg, "Read accelerate message: %d", data);
            DEBUG_PRINTLN(logMsg);
        } else if (header == SetLogging) {
            logEnabled = data;
            sendACK(header);
        } else if (header == NewLog) {
            memset(logMsg, 0, sizeof(logMsg));
            sprintf(logMsg, "NEW_LOG %d", data);
            sendToLog();
            Serial.println("\n*** Logging begin ***");
            memset(logMsg, 0, sizeof(logMsg));
            sprintf(logMsg, "millis(), steeringRef, steering sensor, acceleration");
            sendToLog();
            sendACK(header);
        } else if (header == Benchmark) {
            handleBenchmark(message & 0xFF);
        } else if (header == PIDP) {
            PID_Kp = data;
            memset(logMsg, 0, sizeof(logMsg));
            sprintf(logMsg, "Read new P message: %d", PID_Kp);
            DEBUG_PRINTLN(logMsg);
            sendACK(header);
        } else if (header == PIDI) {
            PID_Ki = data;
            memset(logMsg, 0, sizeof(logMsg));
            sprintf(logMsg, "Read new I message: %d", PID_Ki);
            DEBUG_PRINTLN(logMsg);
            sendACK(header);
        } else if (header == PIDD) {
            PID_Kd = data;
            memset(logMsg, 0, sizeof(logMsg));
            sprintf(logMsg, "Read new D message: %d", PID_Kd);
            DEBUG_PRINTLN(logMsg);
            sendACK(header);
        }
    } else if (mode != ManualMode && millis() - lastMessageReceivedTime > 5000) {
        DEBUG_PRINTLN("No radio messages received for 5 seconds, falling back to manual mode!");
        enableManualMode();
    }
}

void sendToLog() {
    while (!tryToLog()) {
        DEBUG_PRINTLN("Serial1 busy... trying again!");
    }
}

bool tryToLog() {
    static unsigned long lastSentMessage = 0;

    if (millis() - lastSentMessage > 30) {
        lastSentMessage = millis();
        Serial1.write(logMsg, 60);
        Serial.println(logMsg);
        return true;
    }
    return false;
}

// Send radio message that the last message with header `header` was received.
void sendACK(int header) {
    int message = combine(ACK, header);
    sendRadioMessage(message);
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

        // Turn the ints into floats
        double Kp = PID_Kp / 10.0f;
        double Ki = PID_Ki / 10.0f;
        double Kd = PID_Kd / 10.0f;

        D = PID_Tf / (PID_Tf + PID_SAMPLE_TIME) * D + Kd / (PID_Tf + PID_SAMPLE_TIME) * (e - e_old);
        P = Kp * e;
        double u = P + D + I; // Calculate control output
        // Calculate next I value
        I = I + Ki * PID_SAMPLE_TIME * e;

        e_old = e;

        I = constrain(I, -255.0 - P - D, 255 - P - D);
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

void log() {
    if (logEnabled) {
        memset(logMsg, 0, sizeof(logMsg));
        sprintf(logMsg, "%lu,\t%d,\t%d,\t%d", millis(), steeringRef, getSteeringValue(), acceleration);
        tryToLog();
    }
}

void loop() {
    readRadio();

    if (mode == RemoteMode) {
        // Run PID here
        updateMotor(calculatePID());

        // Log
        log();
    }
}
