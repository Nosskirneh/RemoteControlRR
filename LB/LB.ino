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
    radio.openWritingPipe(RADIO_ADDRESS);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
    Serial.print("Sending radio on ");
    Serial.print(RADIO_CE);
    Serial.print(", ");
    Serial.println(RADIO_CSN);

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

    Serial.println("Everything went OK!");
    sendVibratePulse();
}

int combine(byte b1, byte b2) {
    // Put the content on the first two bytes of an int
    int combined = b1 << 8 | b2;
    return combined;
}

bool sendRadioMessage(int message) {
    static unsigned long lastTimestamp = 0;
    // RF24 somehow crashes Arduino when sending too fast.
    // Limit it to every 0.1 s. Do not block first call.
    if (lastTimestamp == 0 || millis() - lastTimestamp > 40) {
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
    ps2x.read_gamepad(false, 0);       // Stop vibrate
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
        /* Always send both a acceleration and a steering message, since just
        sending when the joysticks are not in the middle will result in it being
        impossible to send a 0 % acceleration or steer straight forward message.
        */

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
        if (sendRadioMessage(message)) {
            if ((message >> 8 & 0xFF) == Accelerate) {
                Serial.print("Will send percentage: ");
            } else {
                Serial.print("Will send steering angle: ");
            }
            Serial.println(message & 0xFF);
            lastSentMessage = !lastSentMessage;   
        }
    }
}

void loop() {
    readController();
}
