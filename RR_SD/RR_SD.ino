#include <SD.h>

#define SD_PIN 4
#define LOG_FILENAME "log"
#define LOG_FILETYPE ".txt"

char logName[30];
File logFile;
const char logMsg[100];


void setup() {
    Serial.begin(9600);
    Serial1.begin(230400);

    // Logging
    if (!SD.begin(SD_PIN)) {
        Serial.println("SD card initialization failed!");
    } else {
        Serial.println("SD card init succeeded");
        sprintf(logName, "%s%s", LOG_FILENAME, LOG_FILETYPE);
        logFile = SD.open(logName, FILE_WRITE);
        if (logFile) {
            Serial.println("\n*** Logging begin ***");
            logToFile("millis(), steeringRef, steering sensor, acceleration");
        }
    }
}

void logToFile(String str) {
    logFile = SD.open(logName, FILE_WRITE);
    logFile.println(str);
    logFile.close();
}

void loop() {
    static bool nextMsgIsLogName = false;
    static String input = "";

    if (!Serial1.available())
        return;

    input = Serial1.readStringUntil('\0');
    Serial.print("reading input: ");
    Serial.println(input);
    if (input.equals("NEW_LOG")) {
        // Is next message new log name?
        Serial.println("received new log msg");
        nextMsgIsLogName = true;
    } else if (nextMsgIsLogName) {
        // Save log name
        sprintf(logName, "%s%lu%s", LOG_FILENAME, input.toInt(), LOG_FILETYPE);
        Serial.println(logName);
        nextMsgIsLogName = false;
    } else {
        // Write to log
        logToFile(input);
        Serial.println(input);
    }
}
