#include "RemoteControlRRLib.h"

int combine(byte b1, byte b2) {
    // Put the content on the first two bytes of an int
    int combined = b1 << 8 | b2;
    return combined;
}

// Fetch the next radio message
int nextRadioMessage() {
    int message;
    radio.read(&message, sizeof(message));
    return message;
}

// Change of mode and other messages are far more important to send than an
// accelerate/steering message (the latter are repeating itself whilst change of
// mode is not).
void sendRadioMessage(int message) {
    while (!trySendRadioMessage(message))
        DEBUG_PRINTLN("Radio chip busy... trying again!");
}

bool trySendRadioMessage(int message) {
    static unsigned long lastTimestamp = 0;
    // RF24 somehow crashes Arduino when sending too fast.
    // Limit it to every 0.04 s. Do not block first call.
    if (lastTimestamp == 0 || millis() - lastTimestamp > 40) {
        radio.stopListening();
        radio.write(&message, sizeof(message));
        radio.startListening();
        lastTimestamp = millis();
        return true;
    }
    return false;
}
