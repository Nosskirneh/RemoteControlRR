#include <RF24.h>

extern RF24 radio;

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
    ManualMode = 0b00000001,
    RemoteMode = 0b00000010,
    Steer      = 0b00001000,
    Accelerate = 0b00001001,
    SetLogging = 0b10000000,
    NewLog     = 0b10000001,
    Benchmark  = 0b10000010,
    ACK        = 0b10000011
} MessageType;

// To be sent after the Benchmark header
typedef enum BenchmarkData {
    BenchmarkLeft   = 0b00000000,
    BenchmarkRight  = 0b00000001,
    BenchmarkCancel = 0b11110000
} BenchmarkData;

// To be used locally
typedef enum BenchmarkMode {
    BenchmarkSent = 0,
    BenchmarkInitiated = 1,
    BenchmarkDone = 2
} BenchmarkMode;

const byte RADIO_ADDRESS[2][6] = {"00001", "00002"};
const char logMsg[60];

// Put the content on the first two bytes of an int
int combine(byte b1, byte b2);

// Fetch the next radio message
int nextRadioMessage();

// Change of mode and other messages are far more important to send than an
// accelerate/steering message (the latter are repeating itself whilst change of
// mode is not).
void sendRadioMessage(int message);
bool trySendRadioMessage(int message);
