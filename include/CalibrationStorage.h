#pragma once
#include <Arduino.h>
#include <EEPROM.h>

struct CalibBlob {
    uint32_t magic = 0xC0FFEE21;
    uint16_t version = 1;
    float rollDegOffset = 0.0f;
    uint16_t crc = 0;
};

class CalibrationStorage {
    private:
        static constexpr int EEPROM_ADDR = 0;
        bool initialized = false;
        size_t eepromSize = 0;

        static uint16_t crc16_acc(const uint8_t* d, size_t n) {
            uint16_t c = 0;
            for (size_t i=0;i<n;i++) c = (uint16_t)(c + d[i]);
            return c;
        };

        uint16_t crcCalib(const CalibBlob& b) {
            uint16_t c = 0;
            c = crc16_acc((const uint8_t*)&b.magic,   sizeof b.magic);
            c = crc16_acc((const uint8_t*)&b.version, sizeof b.version) + c;
            c = crc16_acc((const uint8_t*)&b.rollDegOffset, sizeof b.rollDegOffset) + c;
            return c;
        }

    public:
        CalibrationStorage() = default;

        bool begin(size_t memorySize) {
            eepromSize = memorySize;
            initialized = EEPROM.begin(memorySize);

            return initialized;
        }

        CalibBlob loadCalibration() {
            CalibBlob blob{};
            EEPROM.get(EEPROM_ADDR, blob);

            bool ok = (blob.magic == 0xC0FFEE21) && (blob.version == 1);
            if (ok) {
                uint16_t expectedCrc = crcCalib(blob);
                ok = (expectedCrc == blob.crc);
            }

            if (ok) {
                Serial.printf("EEPROM: Offset geladen = %.2f°\n", blob.rollDegOffset);
            } else {
                Serial.println("EEPROM: keine gültige Kalibrierung gefunden (Offset=0).");
            }

            return blob;
        };

        bool saveCalibration(float offsetDeg) {
            CalibBlob blob{};
            blob.rollDegOffset = offsetDeg;
            blob.crc = crcCalib(blob);
            EEPROM.put(EEPROM_ADDR, blob);
            if (!EEPROM.commit()) {
                Serial.println("EEPROM: commit FAILED");
                return false;
            }
            Serial.printf("EEPROM: Offset gespeichert = %.2f°\n", offsetDeg);
            return true;
        };
};