#include <SPI.h>
#include <RF24.h>
#include <PS2X_lib.h>

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

    if (error != 0) {
        Serial.println("Something is wrong with the PS2 controller");
    } else {
        Serial.println("Everything went OK!");
        ps2x.enableRumble();
        ps2x.enablePressures();
    }
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

void sendRadioMessage(int message) {
    Serial.print("sending message: ");
    Serial.println(message, BIN);
    radio.write(&message, sizeof(message));
}

void sendVibratePulse() {
    ps2x.read_gamepad(false, 255); // Vibrate
    delay(300);
    ps2x.read_gamepad(false, 0); // Vibrate
}

void readController() {
    static int mode = ManualMode;
    ps2x.read_gamepad();

    // Send change of mode?
    if (ps2x.Button(PSB_SELECT)) {
        // Switch mode
        if (mode == ManualMode) {
            mode = RemoteMode;
        } else {
            mode = ManualMode;
        }

        sendRadioMessage(mode);

        // Vibrate and delay!
        // Replace the delays with millis() to allow user inputs in between?
        for (int i = 0; i < mode + 1; i++) {
            delay(500);
            sendVibratePulse();
        }
    }

    if (mode == RemoteMode) {
        // Send accelerate?
        byte LY = ps2x.Analog(PSS_LY);
        if (isJoystickUpwards(LY)) {
            // The Y value of the joysticks on the PS2 controller
            // has 255 as its maximum value downwards and 0 upwards.
            // We need to map it to a percentage before sending the data
            byte percentage = map(LY, JOYSTICK_CENTER - JOYSTICK_THRESHOLD, 0, 0, 100);
            int message = combine(Accelerate, percentage);
            sendRadioMessage(message);
        }

        // Always send steering (if it's center it's 90 degrees)
        byte RX = ps2x.Analog(PSS_RX);
        Serial.println(ps2x.Analog(PSS_RX));
        byte angle;
        if (RX < JOYSTICK_CENTER - JOYSTICK_THRESHOLD) // Right
            angle = map(RX, 0, JOYSTICK_CENTER - JOYSTICK_THRESHOLD, 180, 90);
        else if (RX > JOYSTICK_CENTER + JOYSTICK_THRESHOLD) // Left
            angle = map(RX, JOYSTICK_CENTER + JOYSTICK_THRESHOLD, 255, 90, 0);
        else // Center
            angle = 90;
        Serial.println(angle);
        int message = combine(Steer, angle);
        sendRadioMessage(message);
    }

}

void loop() {
    delay(1000);

    readController();
}
