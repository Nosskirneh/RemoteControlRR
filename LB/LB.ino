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

#define PS2_CLK 24
#define PS2_CMD 23
#define PS2_ATT 22
#define PS2_DAT 25


void setup() {
    Serial.begin(9600);

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
    sendVibratePulse(300);
}

void sendVibratePulse(unsigned int time) {
    DEBUG_PRINTLN("Vibrate!");
    ps2x.read_gamepad(false, PS2_MAX); // Vibrate
    delay(time);
    ps2x.read_gamepad(false, 0);       // Stop vibrate
}

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
    if (header.equals("NEW_LOG")) {
        DEBUG_PRINTLN("Read newlog message");
        int data = getStringAtIndexSeparatedByChar(input, ' ', 1).toInt();
        int message = combine(NewLog, data);
        sendRadioMessage(message);
    }
}

#define NR_BUTTONS      4
#define SELECT_BUTTON   0
#define R2_BUTTON       1
#define CIRCLE_BUTTON   2
#define TRIANGLE_BUTTON 3

static unsigned int buttons[NR_BUTTONS] = {PSB_SELECT, PSB_R2, PSB_CIRCLE, PSB_TRIANGLE};
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
    sprintf(logMsg, "Changing mode to: %d", mode);
    DEBUG_PRINTLN(logMsg);

    sendRadioMessage(combine(mode, 0));
}

void sendRunBenchmark() {
    int message = combine(RunBenchmark, true);
    sendRadioMessage(message);
}

void readController() {
    ps2x.read_gamepad();

    // Check button states
    checkButton(ps2x, SELECT_BUTTON, 200, &sendChangeOfMode);
    checkButton(ps2x, R2_BUTTON, 1000, &sendRunBenchmark);
    checkButton(ps2x, CIRCLE_BUTTON, 1000, &sendLoggingDisabled);
    checkButton(ps2x, TRIANGLE_BUTTON, 1000, &sendLoggingEnabled);

    // Joysticks
    /* Always send both acceleration and steering messages, since just
    sending when the joysticks are not in the middle will result in it being
    impossible to send a 0 % acceleration or steer straight forward message.
    */
    if (mode == RemoteMode) {
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
            if ((message >> 8 & 0xFF) == Accelerate) {
                DEBUG_PRINT("Will send percentage: ");
            } else {
                DEBUG_PRINT("Will send steering angle: ");
            }
            DEBUG_PRINTLN(message & 0xFF);
            lastSentMessage = !lastSentMessage;   
        }
    }
}

void processACK(byte data) {
    if (data == ManualMode || data == RemoteMode) {
        // Vibrate and delay!
        for (int i = 0; i < mode; i++) {
            if (i != 0)
                delay(500);
            sendVibratePulse(300);
        }
    } else if (data == NewLog || data == SetLogging || data == RunBenchmark) {
        sendVibratePulse(1000);
    }
}

// Check for new ACK messages
void readRadio() {
    if (radio.available()) {
        int message = nextRadioMessage();
        DEBUG_PRINTLN(message);

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
}
