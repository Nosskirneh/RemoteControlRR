#include <SD.h>

#define SD_PIN 4
#define LOG_FILENAME "log"
#define LOG_FILETYPE ".txt"

char logName[30];
File logFile;


void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);

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
    if (!Serial1.available())
        return;

    String input = Serial.readStringUntil('\0');
    if (input.length() == 0)
        return;

    if (input.startsWith("NEW_LOG")) {
        Serial.println("received new log msg");
        sprintf(logName, "%s%lu%s", LOG_FILENAME, input.substring(8).toInt(), LOG_FILETYPE);
        Serial.println(logName);
    } else {
        // Write to log
        logToFile(input);
        Serial.println(input);
    }
}
