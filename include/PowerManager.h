#pragma once
#include <Arduino.h>

class PowerManager {
    private:
        static constexpr float VOLTAGE_DIVIDER = 1.805;
        static constexpr uint8_t PWR_OFF_PERCENT = 10;
        static constexpr uint8_t PWR_ON_PERCENT = 20;
        
        static inline constexpr float VOLTS[]  = {4.20, 4.10, 4.00, 3.90, 3.80, 3.75, 3.70, 3.65, 3.60, 3.50, 3.40};
        static inline constexpr float PERCENT[] = {100,  90,   80,   70,   60,   50,   40,   30,   20,   10,    0 };

        uint8_t battPin;
        uint8_t vusbPin;
        uint8_t pwrPin;
        bool pwrEnabled = true;
        int pwrStatus = HIGH;

        float rawToV(int raw) {
            return raw * (3.3 / 4095.0) * VOLTAGE_DIVIDER;
        }

        uint8_t resolveBatteryStatus(float v_batt) {
            if (v_batt >= VOLTS[0]) return 100;
            if (v_batt <= VOLTS[10]) return 0;

            for (int i = 0; i < 10; i++) {
                if (v_batt >= VOLTS[i+1]) {
                    float v1 = VOLTS[i];
                    float v2 = VOLTS[i+1];
                    float p1 = PERCENT[i];
                    float p2 = PERCENT[i+1];

                    return (uint8_t)(p1 + (p2 - p1) * (v_batt - v1) / (v2 - v1));
                }
            }

            return 0;
        }
    
    public:
        PowerManager(uint8_t batt_pin, uint8_t vusb_pin, uint8_t pwr_pin)
            : battPin(batt_pin), vusbPin(vusb_pin), pwrPin(pwr_pin) {
                analogReadResolution(12);  // 0–4095
                analogSetAttenuation(ADC_11db); // bis ca. 3.3V
                pinMode(pwrPin, OUTPUT);
            };

        float readVBattery() {
            return rawToV(analogRead(battPin));
        }

        uint8_t readBatteryPercent() {
            return resolveBatteryStatus(readVBattery());
        }

        float readVUSB() {
            return rawToV(analogRead(vusbPin));
        }

        bool isPowerEnabled() {
            if (!pwrEnabled) {
                pwrStatus = LOW;
            } else {
                bool usbPlugged = resolveBatteryStatus(readVUSB()) > 0;
                uint8_t batteryLimit = (pwrStatus == LOW) ? PWR_ON_PERCENT : PWR_OFF_PERCENT;
                bool battEmpty = resolveBatteryStatus(readVBattery()) < batteryLimit;
                
                pwrStatus = (!usbPlugged && !battEmpty) ? HIGH : LOW;
            }
            digitalWrite(pwrPin, pwrStatus);

            return pwrStatus == HIGH;
        }

        bool enablePower(bool state) {
            pwrEnabled = state;

            return isPowerEnabled();
        }
};