#pragma once
#include <Arduino.h>
#include <EEPROM.h>

struct ConfigBlob {
    uint32_t magic = 0xC0FFEE21;
    uint8_t  version = 6;
    float    rollDegOffset = 0.0f;
    float    yawBias = 0.0f;
    bool     logging = false;
    bool     servo = false;
    int8_t   gearOffset = 0;
    float    gearRatio = 5.5f;
    uint16_t crc = 0;
};


class ConfigurationStorage {
    private:
        static constexpr int EEPROM_ADDR = 0;
        static constexpr uint8_t BLOB_VERSION = 6;

        Stream *logger;
        bool initialized = false;
        size_t eepromSize = 0;

        static uint16_t crc16_acc(const uint8_t* d, size_t n) {
            uint16_t c = 0;
            for (size_t i=0;i<n;i++) c = (uint16_t)(c + d[i]);
            return c;
        };

        uint16_t crcConfig(const ConfigBlob& b) {
            uint16_t c = 0;
            c = crc16_acc((const uint8_t*)&b.magic,   sizeof b.magic);
            c = crc16_acc((const uint8_t*)&b.version, sizeof b.version) + c;
            c = crc16_acc((const uint8_t*)&b.rollDegOffset, sizeof b.rollDegOffset) + c;
            c = crc16_acc((const uint8_t*)&b.yawBias, sizeof b.yawBias) + c;
            c = crc16_acc((const uint8_t*)&b.logging, sizeof b.logging) + c;
            c = crc16_acc((const uint8_t*)&b.servo, sizeof b.servo) + c;
            c = crc16_acc((const uint8_t*)&b.gearOffset, sizeof b.gearOffset) + c;
            c = crc16_acc((const uint8_t*)&b.gearRatio, sizeof b.gearRatio) + c;

            return c;
        }

    public:
        ConfigurationStorage(Stream* l) : logger(l) {};

        bool begin(size_t memorySize) {
            eepromSize = memorySize;
            initialized = EEPROM.begin(memorySize);

            return initialized;
        }

        ConfigBlob load() {
            logger->println("EEPROM: load calibration");
            ConfigBlob blob{};
            EEPROM.get(0, blob);
            bool ok = (blob.magic == 0xC0FFEE21) && (blob.version == BLOB_VERSION);
            if (ok) {
                uint16_t expectedCrc = crcConfig(blob);
                ok = (expectedCrc == blob.crc);
            }
            if (!ok) {
                logger->println("EEPROM: invalid blob, using defaults");
                blob = ConfigBlob{};
            }
            return blob;
        };

        bool save(ConfigBlob& blob) {
            logger->println("EEPROM: save calibration");
            blob.crc = crcConfig(blob);
            EEPROM.put(EEPROM_ADDR, blob);
            if (!EEPROM.commit()) {
                logger->println("EEPROM: commit FAILED");
                return false;
            }
            logger->printf("EEPROM: saved! magic=%08X ver=%u offset=%.2f bias=%.3f logging=%d servo=%d gearRatio=%.3f gearOffset=%d crc=%u size=%u\n",
                        blob.magic, blob.version, blob.rollDegOffset, blob.yawBias,
                        (int)blob.logging, (int)blob.servo, blob.gearRatio, blob.gearOffset, blob.crc, (unsigned)sizeof(blob));
            return true;
        };
};