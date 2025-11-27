#pragma once
#include <Arduino.h>
#include <EEPROM.h>

struct ConfigBlob {
    uint32_t deviceID = 0xC0FFEE42;
    uint8_t  version = 2;
    float    yawBias = 0.0f;
    bool     logging = false;
    bool     bluetooth = true;
    bool     servo = false;
    int8_t   gearOffset = 0;
    float    gearRatio = 5.5f;
    bool     curveBoost = false;
    uint16_t crc = 0;
};


class ConfigurationStorage {
    private:
        static constexpr int EEPROM_ADDR = 0;
        static constexpr uint32_t DEVICE_ID = 0xC0FFEE42;
        static constexpr uint8_t BLOB_VERSION = 2;

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
            c = crc16_acc((const uint8_t*)&b.deviceID,   sizeof b.deviceID);
            c = crc16_acc((const uint8_t*)&b.version, sizeof b.version) + c;
            c = crc16_acc((const uint8_t*)&b.yawBias, sizeof b.yawBias) + c;
            c = crc16_acc((const uint8_t*)&b.logging, sizeof b.logging) + c;
            c = crc16_acc((const uint8_t*)&b.bluetooth, sizeof b.bluetooth) + c;
            c = crc16_acc((const uint8_t*)&b.servo, sizeof b.servo) + c;
            c = crc16_acc((const uint8_t*)&b.gearOffset, sizeof b.gearOffset) + c;
            c = crc16_acc((const uint8_t*)&b.gearRatio, sizeof b.gearRatio) + c;
            c = crc16_acc((const uint8_t*)&b.curveBoost, sizeof b.curveBoost) + c;

            return c;
        }

    public:
        ConfigurationStorage(Stream* l) : logger(l) {};

        bool begin(size_t memorySize) {
            eepromSize = memorySize;
            initialized = EEPROM.begin(memorySize);

            return initialized;
        }

        void dump(ConfigBlob& blob) {
            logger->printf("EEPROM:\n  ver=%u logging=%d servo=%d\n  gearRatio=%.3f gearOffset=%d curveBoost=%d\n  size=%u\n",
                        blob.version, (int)blob.logging, (int)blob.servo, 
                        blob.gearRatio, blob.gearOffset, (int)blob.curveBoost,
                        (unsigned)sizeof(blob));
        }

        ConfigBlob load() {
            logger->println("EEPROM: load config");
            ConfigBlob blob{};
            EEPROM.get(0, blob);
            bool ok = (blob.deviceID == DEVICE_ID) && (blob.version == BLOB_VERSION);
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
            logger->println("EEPROM: save config");
            blob.crc = crcConfig(blob);
            EEPROM.put(EEPROM_ADDR, blob);
            if (!EEPROM.commit()) {
                logger->println("EEPROM: commit FAILED");
                return false;
            }
            dump(blob);
            
            return true;
        };
};