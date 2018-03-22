#include <SPI.h>
#include <RF24.h>
#include <PS2X_lib.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

PS2X ps2x;

typedef enum MessageType {
    ManualModeMsg = 0b00000000,
    RemoteModeMsg = 0b00000001,
    SteerMsg      = 0b00000100,
    AccelerateMsg = 0b00000110
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
        Serial.println('Something is wrong with the PS2 controller');
    } else {
      ps2x.enableRumble();
      ps2x.enablePressures();
    }
}

bool isJoystickUpwards(byte value) {
    // Middle is about 128 (+- 28)
    return value < (128 - 28);
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

void readController() {
    ps2x.read_gamepad();

    // Send accelerate?
    byte LY = ps2x.Analog(PSS_LY);
    if (isJoystickUpwards(LY)) {
        // The Y value of the joysticks on the PS2 controller
        // has 255 as its maximum value downwards and 0 upwards.
        // We need to map it to a percentage before sending the data
        byte percentage = map(LY, 128 - 28, 0, 0, 100);
        int message = combine(AccelerateMsg, percentage);
        sendRadioMessage(message);
    }

    // Always send steering (if it's center it's 90 degrees)
    byte RX = ps2x.Analog(PSS_RX);
    Serial.println(ps2x.Analog(PSS_RX));
    byte angle;
    if (RX < 128 - 28) // Right
        angle = map(RX, 0, 128 - 28, 180, 90);
    else if (RX > 128 + 28) // Left
        angle = map(RX, 128 + 28, 255, 90, 0);
    else // Center
        angle = 90;
    Serial.println(angle);
    int message = combine(SteerMsg, angle);
    sendRadioMessage(message);
}

void loop() {
    delay(1000);

    readController();
}
