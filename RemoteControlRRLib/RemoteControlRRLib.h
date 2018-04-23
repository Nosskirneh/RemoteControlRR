#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINT(x) Serial.print(x)
#else
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINT(x)
#endif

#define RADIO_CE  7
#define RADIO_CSN 8

// To be sent over radio
typedef enum MessageType {
    ManualMode = 0b00000000,
    RemoteMode = 0b00000001,
    Steer      = 0b00000100,
    Accelerate = 0b00000110
} MessageType;

const byte RADIO_ADDRESS[6] = "00001";
