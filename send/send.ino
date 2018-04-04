#include <SPI.h>
#include <RF24.h>
#include <PS2X_lib.h>


// The PS2 joystick and button values varies between 0 and 255.
#define PS2_MIN            0
#define PS2_MAX            255
#define JOYSTICK_CENTER    128
#define JOYSTICK_THRESHOLD 28

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

PS2X ps2x;

// To be sent over radio
typedef enum MessageType {
    ManualMode = 0b00000000,
    RemoteMode = 0b00000001,
    Steer      = 0b00000100,
    Accelerate = 0b00000110
} MessageType;

int mode = ManualMode;

#define PS2_CLK 22
#define PS2_CMD 23
#define PS2_ATT 24
#define PS2_DAT 25
#define pressures false
#define rumble    false


void setup() {
    Serial.begin(9600);

    // Setup radio
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();

    // Setup PS2 controller
    int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, pressures, rumble);

    while (error != 0) {
        Serial.println("Something is wrong with the PS2 controller! Do not hold any buttons on start. Trying again...");
        delay(200);
        error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, pressures, rumble);
    }

    Serial.println("Everything went OK!");
    ps2x.enableRumble();
    ps2x.enablePressures();
}

bool isJoystickUpwards(byte value) {
    // Middle is about 128 (+- 28)
    return value < (JOYSTICK_CENTER - JOYSTICK_THRESHOLD);
}

int combine(byte b1, byte b2) {
    // Put the content on the first two bytes of an int
    int combined = b1 << 8 | b2;
    return combined;
}

bool sendRadioMessage(int message) {
    static unsigned long lastTimestamp = 0;
    // RF24 somehow crashes Arduino when sending too fast.
    // Limit it to every 0.05 s. Do not block first call.
    if (lastTimestamp == 0 || millis() - lastTimestamp > 50) {
        //Serial.print("sending message: ");
        //Serial.println(message, BIN);
        radio.write(&message, sizeof(message));
        lastTimestamp = millis();
        return true;
    }
    return false;
}

void sendVibratePulse() {
    Serial.println("Vibrate!");
    ps2x.read_gamepad(false, PS2_MAX); // Vibrate
    delay(300);
    ps2x.read_gamepad(false, 0);   // Stop vibrate
}

void checkSelectButton(PS2X ps2x) {
    static bool shouldSaveStartTime = true;
    static unsigned long firstButtonTime;
    if (ps2x.Button(PSB_SELECT)) {
        // Store the timestamp of the first button event
        if (shouldSaveStartTime) {
            shouldSaveStartTime = false;
            firstButtonTime = millis();
        }

        // Has button been pressed for 0.2 seconds?
        if (millis() - firstButtonTime > 200) {
            Serial.println("Button held for at least 0.2 s");

            // Switch mode
            if (mode == ManualMode) {
                mode = RemoteMode;
            } else {
                mode = ManualMode;
            }
            Serial.print("Changing mode to: ");
            Serial.println(mode);

            // Change of mode is far more important to send than
            // an accelerate/steering message (the latter are
            // repeating itself whilst change of mode is not).
            while (!sendRadioMessage(mode)) {
                Serial.println("Radio chip busy... trying again!");
            }

            // Vibrate and delay!
            for (int i = 0; i < mode + 1; i++) {
                if (i != 0)
                    delay(500);
                sendVibratePulse();
            }
            shouldSaveStartTime = true;
        }
    } else if (firstButtonTime != 0) {
        // If we go one loop iteration and the button is
        // not held after the first button event, cancel
        shouldSaveStartTime = true;
    }
}

void readController() {
    ps2x.read_gamepad();

    // Send change of mode?
    checkSelectButton(ps2x);

    if (mode == RemoteMode) {
        // Send accelerate?
        byte LY = ps2x.Analog(PSS_LY);
        if (isJoystickUpwards(LY)) {
            // Map the left joystick's Y-coordinate to a percentage before sending the data
            byte percentage = map(LY, JOYSTICK_CENTER - JOYSTICK_THRESHOLD, PS2_MIN, 0, 100);
            Serial.print("Will send percentage: ");
            Serial.println(percentage);
            int message = combine(Accelerate, percentage);
            sendRadioMessage(message);
        }

        // Map joystick coordinates to degrees
        // Always send steering (if it's center it's 90 degrees)
        byte RX = ps2x.Analog(PSS_RX);
        byte angle;
        if (RX < JOYSTICK_CENTER - JOYSTICK_THRESHOLD) // Right
            angle = map(RX, PS2_MIN, JOYSTICK_CENTER - JOYSTICK_THRESHOLD, 180, 90);
        else if (RX > JOYSTICK_CENTER + JOYSTICK_THRESHOLD) // Left
            angle = map(RX, JOYSTICK_CENTER + JOYSTICK_THRESHOLD, PS2_MAX, 90, 0);
        else // Center
            angle = 90;
        Serial.print("Will send steering angle: ");
        Serial.println(angle);
        int message = combine(Steer, angle);
        sendRadioMessage(message);
    }
}

void loop() {
    delay(40);
    readController();
}
