#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include <cstddef>

struct CalibBlob {
    float xOffset = 0.0f;
    float yOffset = 0.0f;
    float zOffset = 0.0f;
    float rollBias = 0.0f;
    float yawBias = 0.0f;

    CalibBlob() {}
    CalibBlob(float _roll, float _yaw, float _x, float _y, float _z) : rollBias(_roll), yawBias(_yaw), xOffset(_x), yOffset(_y), zOffset(_z) {}
};

struct ConfigBlob {
    uint32_t   deviceID = 0xC0FFEE42;
    uint8_t    version = 3;
    CalibBlob  calibData = CalibBlob();
    bool       logging = false;
    bool       bluetooth = true;
    bool       servo = false;
    int8_t     gearOffset = 0;
    float      gearRatio = 5.5f;
    bool       curveBoost = false;
    uint16_t   crc = 0;
};

class ConfigurationStorage {
    private:
        static constexpr int EEPROM_ADDR = 0;
        static constexpr uint32_t DEVICE_ID = 0xC0FFEE42;
        static constexpr uint8_t BLOB_VERSION = 3;

        Stream *logger;
        bool initialized = false;
        size_t eepromSize = 0;

        uint16_t crcConfig(const ConfigBlob& b) {
            const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&b);
            const size_t len = offsetof(ConfigBlob, crc);
            
            uint16_t c = 0;
            for (size_t i=0;i<len;i++) c = (uint16_t)(c + ptr[i]);

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
            logger->printf("EEPROM: v%u | size: %u bytes\n", blob.version, (unsigned)sizeof(blob));
            logger->printf("gearOffset: %d | gearRatio: %.3f\n", blob.gearOffset, blob.gearRatio);
            logger->printf("yaw: %.2f | roll: %.2f | x: %.2f | y: %.2f | z: %.2f\n", 
                blob.calibData.yawBias, blob.calibData.rollBias, 
                blob.calibData.xOffset, blob.calibData.yOffset, blob.calibData.zOffset);
            logger->printf("logging: %d | servo: %d | curveBoost: %d\n", (int)blob.logging, (int)blob.servo, (int)blob.curveBoost);
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
            
            return true;
        };
};