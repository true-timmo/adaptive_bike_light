#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

#ifndef MotionSensor_h
#include "MotionSensor.h"
#endif

class MotionLogger {
    private:
        static constexpr char* kLogPath = "/log.csv";
        const uint32_t LOG_DURATION_MS = 20UL * 60UL * 1000UL;
        const uint32_t SAMPLE_INTERVAL_MS = 1000;
        const uint16_t FLUSH_EVERY_N_SAMPLES = 20;

    public:
        File logf;
        bool isLogging = false;
        uint32_t nextSampleMs = 0;
        uint32_t endTimeMs = 0;
        uint32_t sampleCount = 0;

        bool begin() {
            if (!LittleFS.begin(true)) { // true: format on fail
                Serial.println(F("FEHLER: LittleFS Mount fehlgeschlagen."));
                return false;
            }

            return true;
        }

        void startNewLog() {
            if (LittleFS.exists(kLogPath)) {
                LittleFS.remove(kLogPath); // frisch beginnen; alternativ: append
            }
            logf = LittleFS.open(kLogPath, FILE_WRITE);
            if (!logf) {
                Serial.println(F("FEHLER: /log.csv konnte nicht geöffnet werden!"));
                return;
            }
            // CSV-Header
            logf.println(F("t_ms,x,y,z"));
            logf.flush();

            isLogging = true;
            sampleCount = 0;
            nextSampleMs = millis();
            endTimeMs = millis() + LOG_DURATION_MS;

            Serial.println(F("Logging gestartet..."));
        };

        void stopLog() {
            if (isLogging) {
                isLogging = false;
                if (logf) {
                logf.flush();
                logf.close();
                }
                Serial.println(F("Logging beendet."));
            }
        }

        void dumpLogToSerial() {
            if (!LittleFS.exists(kLogPath)) {
                Serial.println(F("Keine Logdatei vorhanden."));
                return;
            }
            File f = LittleFS.open(kLogPath, FILE_READ);
            if (!f) {
                Serial.println(F("FEHLER: Konnte Logdatei nicht öffnen."));
                return;
            }
            Serial.println(F("---- BEGIN LOG (/log.csv) ----"));
            while (f.available()) {
                Serial.write(f.read());
            }
            f.close();
            Serial.println(F("---- END LOG ----"));
        }

        void handleSerialMenu() {
        if (!Serial.available()) return;
            char cmd = (char)Serial.read();
            switch (cmd) {
                case 'd': dumpLogToSerial(); break;
                case 'c': stopLog(); if (LittleFS.exists(kLogPath)) LittleFS.remove(kLogPath);
                            Serial.println(F("Logdatei gelöscht.")); break;
                case 'r': stopLog(); startNewLog(); break;
                default:  Serial.println(F("Befehle: d=Dump, c=Clear, s=Status, r=Restart")); break;
            }
        }

        void logAccel(Accel accel) {
            uint32_t now = millis();

            if (logf) {
                logf.print(now);  logf.print(',');
                logf.print(accel.x, 3); logf.print(',');
                logf.print(accel.y, 3); logf.print(',');
                logf.println(accel.z, 3);

                sampleCount++;
                if (sampleCount % FLUSH_EVERY_N_SAMPLES == 0) {
                    logf.flush();
                }
            }
        };
};