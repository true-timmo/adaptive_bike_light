#pragma once
#include <Arduino.h>
#include <math.h>
#include "Button.h"
#include "RideController.h"
#include "esp_sleep.h"

class PowerManager {
    private:
        static constexpr float VOLTAGE_DIVIDER = 1.805;
        static constexpr uint8_t PWR_OFF_PERCENT = 5;
        static constexpr uint8_t PWR_ON_PERCENT = 20;
        static constexpr uint32_t IDLE_DELAY_MS = 1000;
        static constexpr uint32_t SLEEP_DELAY_MS = 600000;

        
        static inline constexpr float VOLTS[]  = {4.15, 4.05, 3.95, 3.85, 3.75, 3.65, 3.55, 3.45, 3.35, 3.30};
        static inline constexpr float PERCENT[] = {100,   90,   80,   65,   50,   35,   20,   10,    5,    0};

        uint8_t battPin;
        uint8_t vusbPin;
        uint8_t pwrPin;
        uint8_t btnPin;

        float vBattRaw = NAN;
        float vUsbRaw = NAN;
        
        Button* button;
        RideController* ride;
        bool sleepPending = false;
        bool pwrEnabled = true;
        int pwrStatus = HIGH;

        float rawToV(float raw) {
            return raw * (3.3 / 4095.0) * VOLTAGE_DIVIDER;
        }

        uint8_t resolveBatteryStatus(float v_batt) {
            if (v_batt >= VOLTS[0]) return 100;
            if (v_batt <= VOLTS[9]) return 0;

            for (int i = 0; i < 9; i++) {
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

        bool isUsbPlugged() {
            return resolveBatteryStatus(readVUSB()) > 0;
        }

        bool isSleepPending() {
            if (sleepPending) return true;

            if (!isUsbPlugged() && ride->getLastServoMoveMs() > SLEEP_DELAY_MS) {
                setSleepPending();
            }

            return sleepPending;
        }
    
    public:
        PowerManager(uint8_t batt_pin, uint8_t vusb_pin, uint8_t pwr_pin, Button* b, RideController* r)
            : battPin(batt_pin), vusbPin(vusb_pin), pwrPin(pwr_pin), button(b), ride(r) {
                analogReadResolution(12);  // 0–4095
                analogSetAttenuation(ADC_11db); // bis ca. 3.3V
                pinMode(pwrPin, INPUT);
            };

        float readVBattery() {
            int raw = analogRead(battPin);
            vBattRaw = (!isfinite(vBattRaw)) 
                ? raw 
                : vBattRaw * 0.85f + raw * 0.15f;

            return rawToV(vBattRaw);
        }

        uint8_t readBatteryPercent() {
            return resolveBatteryStatus(readVBattery());
        }

        float readVUSB() {
            int raw = analogRead(vusbPin);
             vUsbRaw = (!isfinite(vUsbRaw)) 
                ? raw 
                : vUsbRaw * 0.85f + raw * 0.15f;

            return rawToV(vUsbRaw);
        }

        bool isPowerEnabled() {
            if (!pwrEnabled) {
                pwrStatus = LOW;
            } else {
                bool isIdle = ride->getLastServoMoveMs() > IDLE_DELAY_MS;
                uint8_t batteryLimit = (pwrStatus == HIGH) ? PWR_ON_PERCENT : PWR_OFF_PERCENT;
                bool battEmpty = resolveBatteryStatus(readVBattery()) < batteryLimit;
                
                pwrStatus = (!isIdle && !isUsbPlugged() && !battEmpty) ? HIGH : LOW;
            }

            if (pwrStatus == HIGH) {
                pinMode(pwrPin, OUTPUT);
                digitalWrite(pwrPin, LOW);
            } else {
                pinMode(pwrPin, INPUT);
            }
            

            return pwrStatus == HIGH;
        }

        bool enablePower(bool state) {
            pwrEnabled = state;

            return isPowerEnabled();
        }
    
        void setSleepPending() {
            sleepPending = true;
            ride->turnNeutral();
        }

        bool goSleep() {
            if (!isSleepPending()) return false;

            ride->hibernate();
            delay(20);
            enablePower(false);
            sleepPending = false;

            const esp_deepsleep_gpio_wake_up_mode_t wakeLevel =
                button->getActiveLevel() ? ESP_GPIO_WAKEUP_GPIO_HIGH : ESP_GPIO_WAKEUP_GPIO_LOW;

            esp_deep_sleep_enable_gpio_wakeup(BIT(button->getPin()), wakeLevel);
            esp_deep_sleep_start();

            return true;
        }
};