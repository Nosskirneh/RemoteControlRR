/*
    Leaderboat:
    The PS2-controller's left joystick is used to accelerate and the right for
    steering. Hold the SELECT button to change mode between manual and remote
    control.
*/


#include <SPI.h>
#include <RF24.h>
#include <PS2X_lib.h>
#include <RemoteControlRRLib.h>

// The PS2 joystick and button values varies between 0 and 255.
#define PS2_MIN            0
#define PS2_MAX            255
#define JOYSTICK_CENTER    128
#define JOYSTICK_THRESHOLD 28

RF24 radio(RADIO_CE, RADIO_CSN);
PS2X ps2x;

int mode = ManualMode;
static byte benchmarkMode = BenchmarkDone;

#define PS2_CLK 24
#define PS2_CMD 23
#define PS2_ATT 22
#define PS2_DAT 25


void setup() {
    Serial.begin(115200);

    // Setup radio
    radio.begin();
    radio.openWritingPipe(RADIO_ADDRESS[0]);
    radio.openReadingPipe(1, RADIO_ADDRESS[1]);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.stopListening();
    sprintf(logMsg, "Sending radio on pins: %d, %d", RADIO_CE, RADIO_CSN);
    DEBUG_PRINTLN(logMsg);

    // Setup PS2 controller
    int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, true, true);

    while (error != 0) {
        Serial.println("Something is wrong with the PS2 controller! Trying again...");
        delay(200);
        error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, true, true);
    }

    // This isn't done by the library for some reason
    ps2x.enableRumble();
    ps2x.enablePressures();

    DEBUG_PRINTLN("Everything went OK!");
    sendVibratePulses(1, 300);
}


/* Vibrate */
static unsigned long vibrateDeltaTime;
static unsigned long vibrateStartTime;
static unsigned long vibrateEndTime;
static byte vibrate;
static unsigned int pulses;

void updateVibrateState() {
    if (pulses == 0)
        return;

    if (vibrate != 0 && millis() - vibrateStartTime > vibrateDeltaTime) {
        // Enough time has passed, stop vibrating
        vibrateEndTime = millis();
        pulses--;
        vibrate = 0;
    } else if (vibrate == 0 && millis() - vibrateEndTime > vibrateDeltaTime) {
        vibrateStartTime = millis();
        vibrate = PS2_MAX;
    }
}

void sendVibratePulses(unsigned int p, unsigned int t) {
    pulses = p;
    vibrateDeltaTime = t;
    vibrateStartTime = millis();
    DEBUG_PRINTLN("Vibrate!");
    ps2x.read_gamepad(false, PS2_MAX); // Vibrate
}
// ---

void checkSerialMessage() {
    static String input = "";

    if (Serial.available()) {
        input = Serial.readStringUntil(';');
        Serial.println(input);
        processSerialMessage(input);
        input = "";
    }
}

// Help method to receive string separated by a character
String getStringAtIndexSeparatedByChar(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }

    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void processSerialMessage(String input) {
    String header = getStringAtIndexSeparatedByChar(input, ' ', 0);
    int data = getStringAtIndexSeparatedByChar(input, ' ', 1).toInt();
    if (header.equals("NEW_LOG")) {
        DEBUG_PRINTLN("Read newlog message");
        int message = combine(NewLog, data);
        sendRadioMessage(message);
    } else if (header.equals("PID_P")) {
        DEBUG_PRINTLN("Read new PID P message");
        int message = combine(PIDP, data);
        sendRadioMessage(message);
    } else if (header.equals("PID_I")) {
        DEBUG_PRINTLN("Read new PID I message");
        int message = combine(PIDI, data);
        sendRadioMessage(message);
    } else if (header.equals("PID_D")) {
        DEBUG_PRINTLN("Read new PID D message");
        int message = combine(PIDD, data);
        sendRadioMessage(message);
    }
}

#define NR_BUTTONS      5
#define SELECT_BUTTON   0
#define R2_BUTTON       1
#define L2_BUTTON       2
#define CIRCLE_BUTTON   3
#define TRIANGLE_BUTTON 4

static unsigned int buttons[NR_BUTTONS] = {PSB_SELECT, PSB_R2, PSB_L2, PSB_CIRCLE, PSB_TRIANGLE};
static bool hasSavedStartTime[NR_BUTTONS];
static unsigned long startTime[NR_BUTTONS];

// Generic check button held state method to prevent redundancy
void checkButton(PS2X ps2x, unsigned int button, unsigned int delay, void *function()) {
    if (ps2x.Button(buttons[button])) {
        // Store the timestamp of the first button event
        if (!hasSavedStartTime[button]) {
            hasSavedStartTime[button] = true;
            startTime[button] = millis();
        }

        // Has button been pressed for <delay> time?
        if (millis() - startTime[button] > delay) {
            sprintf(logMsg, "Button %d held for at least %d millis", button, delay);
            DEBUG_PRINTLN(logMsg);

            function();

            hasSavedStartTime[button] = false;
        }
    } else if (startTime[button] != 0) {
        // If we go one loop iteration and the button is
        // not held after the first button event, cancel
        hasSavedStartTime[button] = false;
    }
}

void sendLoggingEnabled() {
    int message = combine(SetLogging, true);
    sendRadioMessage(message);
}

void sendLoggingDisabled() {
    int message = combine(SetLogging, false);
    sendRadioMessage(message);
}

void sendChangeOfMode() {
    // Switch mode
    if (mode == ManualMode)
        mode = RemoteMode;
    else
        mode = ManualMode;
    sprintf(logMsg, "Sending mode change to: %d", mode);
    DEBUG_PRINTLN(logMsg);

    sendRadioMessage(combine(mode, 0));
}

void sendRightBenchmark() {
    sendBenchmarkEvent(BenchmarkRight);
}

void sendLeftBenchmark() {
    sendBenchmarkEvent(BenchmarkLeft);
}

void sendBenchmarkEvent(byte direction) {
    int message;
    if (benchmarkMode != BenchmarkDone) {
        // Cancel the benchmark
        DEBUG_PRINTLN("Will cancel benchmark");
        message = combine(Benchmark, BenchmarkCancel);
        benchmarkMode = BenchmarkDone;
    } else {
        // Start it
        DEBUG_PRINTLN("Will start benchmark");
        message = combine(Benchmark, direction);
        benchmarkMode = BenchmarkSent;
    }
    sendRadioMessage(message);
}

void readController() {
    ps2x.read_gamepad(false, vibrate);

    // Check button states
    checkButton(ps2x, SELECT_BUTTON, 400, &sendChangeOfMode);
    checkButton(ps2x, R2_BUTTON, 1000, &sendRightBenchmark);
    checkButton(ps2x, L2_BUTTON, 1000, &sendLeftBenchmark);
    checkButton(ps2x, CIRCLE_BUTTON, 1000, &sendLoggingDisabled);
    checkButton(ps2x, TRIANGLE_BUTTON, 1000, &sendLoggingEnabled);

    // Joysticks
    /* Always send both acceleration and steering messages, since just
    sending when the joysticks are not in the middle will result in it being
    impossible to send a 0 % acceleration or steer straight forward message.
    */
    if (mode == RemoteMode && benchmarkMode == BenchmarkDone) {
        static bool lastSentMessage = false;

        int message;

        // Create message
        if (lastSentMessage != true) {
            // Accelerate
            // Map the left joystick's Y-coordinate to a percentage
            byte LY = ps2x.Analog(PSS_LY);
            byte percentage;
            if (LY < JOYSTICK_CENTER - JOYSTICK_THRESHOLD)
                percentage = map(LY, JOYSTICK_CENTER - JOYSTICK_THRESHOLD, PS2_MIN, 0, 100);
            else
                percentage = 0;
            message = combine(Accelerate, percentage);
        } else {
            // Steering
            byte RX = ps2x.Analog(PSS_RX);
            // Prevent small deviations from center
            if (RX < JOYSTICK_CENTER - JOYSTICK_THRESHOLD) // Right
                RX = map(RX, PS2_MIN, JOYSTICK_CENTER - JOYSTICK_THRESHOLD, PS2_MIN, JOYSTICK_CENTER);
            else if (RX > JOYSTICK_CENTER + JOYSTICK_THRESHOLD) // Left
                RX = map(RX, JOYSTICK_CENTER + JOYSTICK_THRESHOLD, PS2_MAX, JOYSTICK_CENTER, PS2_MAX);
            else // Center
                RX = JOYSTICK_CENTER;
            message = combine(Steer, RX);
        }

        // Send it
        if (trySendRadioMessage(message)) {
            if ((message >> 8 & 0xFF) == Accelerate)
                DEBUG_PRINT("Will send percentage: ");
            else
                DEBUG_PRINT("Will send steering angle: ");
            DEBUG_PRINTLN(message & 0xFF);
            lastSentMessage = !lastSentMessage;   
        }
    }
}

void processACK(byte data) {
    if (data == ManualMode || data == RemoteMode) {
        sprintf(logMsg, "Changing mode to: %d", mode);
        DEBUG_PRINTLN(logMsg);
        sendVibratePulses(mode, 300);
        benchmarkMode = BenchmarkDone;
    } else if (data == NewLog || data == SetLogging) {
        sendVibratePulses(1, 1000);
    } else if (data == Benchmark) {
        if (benchmarkMode == BenchmarkSent) // Benchmark started
            benchmarkMode = BenchmarkInitiated;
        else // Second ACK: Benchmark was canceled/completed
            benchmarkMode = BenchmarkDone;
        sendVibratePulses(1, 1000);
    }
}

// Check for new ACK messages
void readRadio() {
    if (radio.available()) {
        int message = nextRadioMessage();

        // Mask out the header (first 8 bits)
        int header = message >> 8 & 0xFF;
        if (header == ACK) {
            DEBUG_PRINTLN("Read ACK!");
            processACK(message & 0xFF);
        }
    }
}

void loop() {
    // Read the data from PS2 controller
    readController();

    // Check serial input for new log message
    checkSerialMessage();

    // Check if any ACK messages have been received
    readRadio();

    // Update vibrate state of PS2 controller
    updateVibrateState();
}
