#pragma once
#include <Arduino.h>

#ifndef ESP32_Servo_h
#include <ESP32Servo.h>
#endif

#ifndef MotionSensor_h
#include "MotionSensor.h"
#endif

/*
 * Servo reagiert auf Neigewinkel (Grad): servo = neutral + GAIN * lean_deg
 * Vorzeichen bestimmt, ob „in die Kurve“ gelenkt wird.
 */
struct SERVO {
  static constexpr int PIN             = 21;
  static constexpr int PWM_MIN         = 1000;
  static constexpr int PWM_MAX         = 2000;
  static constexpr float MAX_SPEED_DPS = 360.0f;
  static constexpr float NEUTRAL_DEG   = 90.0f; 
  static constexpr float MIN_DEG       = 30.0f;
  static constexpr float MAX_DEG       = 150.0f;
  static constexpr float GAIN          = -3.5f;
};

enum class RideState { STRAIGHT, CURVE };

class RideController {
    private:
        static constexpr uint32_t CALIB_TIME_MS = 2000;
        static constexpr bool AUTO_RECENTER_ENABLE = true;
        static constexpr float AUTO_RECENTER_RATE_DPS = 0.05f;
        static constexpr float ROLL_SIGN = -1.0f;

        static constexpr float LEAN_ENTER_DEG = 4.0f;   // ab diesem gefilterten Rollwinkel: "Kurve"
        static constexpr float LEAN_EXIT_DEG  = 2.0f;   // darunter zurück zu "Gerade"
        static constexpr uint32_t ENTER_HOLD_MS = 180;  // Mindestdauer für Eintritt
        static constexpr uint32_t EXIT_HOLD_MS  = 400;  // Mindestdauer für Austritt

        // --- „Schwanken“ glätten ---
        static constexpr float LPF_TAU_S = 0.25f;       // größer = stärker geglättet
        static constexpr float OUTPUT_DEADBAND_DEG = 0.8f;

        MotionSensor *sensor;
        Servo *servo;
        RideState state = RideState::STRAIGHT;

        float rollDegOffset       = 0.0f;
        float rollDegFiltered     = 0.0f; // gefilterter Rollwinkel nach Offset
        float currentServoAngle   = SERVO::NEUTRAL_DEG;

        uint32_t lastTimestamp    = 0;
        float lastToCurrent       = 0.0f;
        uint32_t currentTimestamp = 0;
        uint32_t stateTimerStart  = 0;

        static inline float clampf(float v, float lo, float hi) {
            return v < lo ? lo : (v > hi ? hi : v);
        };

    public:
        RideController(MotionSensor* s, Servo* v) : sensor(s), servo(v) {}

        void init(float offset) {
            rollDegOffset = offset;
            servo->setPeriodHertz(50);
            servo->attach(SERVO::PIN, SERVO::PWM_MIN, SERVO::PWM_MAX);
            servo->write(SERVO::NEUTRAL_DEG);
        }

        void setTiming() {
            currentTimestamp = millis();
            lastToCurrent = (currentTimestamp - lastTimestamp) / 1000.0f;
            if (lastToCurrent <= 0.0f || lastToCurrent > 0.5f) {
                lastToCurrent = 0.02f;
            }
            lastTimestamp = currentTimestamp;
        }

        static inline float computeRollDegFromAccel(Accel accel) {
            if (!isfinite(accel.x) || !isfinite(accel.y)) return NAN;

            float rollRad = atan2f(accel.x, accel.y);
            float rollDeg = rollRad * 180.0f / PI;
            return ROLL_SIGN * rollDeg;
        };

        float runCalibration(uint32_t duration_ms = 2000) {
            servo->write(SERVO::NEUTRAL_DEG);

            Serial.println("Kalibriere... bitte Fahrrad/Mechanik aufrecht halten.");
            uint32_t t0 = millis();
            uint32_t n  = 0;
            double sum = 0.0;

            Accel accel = Accel();
            while (millis() - t0 < duration_ms) {
                if (sensor->readAccel(&accel)) {
                    float roll = computeRollDegFromAccel(accel);
                    if (isfinite(roll)) { sum += roll; n++; }
                }
                delay(5);
            }
            rollDegOffset = (n > 0) ? (float)(sum / (double)n) : 0.0f;
            Serial.printf("Kalibrierung fertig. Neuer Offset = %.2f°\n", rollDegOffset);
            return rollDegOffset;
        };

        void turnNeutral() {
            float step = SERVO::MAX_SPEED_DPS * lastToCurrent;
            float target = SERVO::NEUTRAL_DEG;
            float delta = clampf(target - currentServoAngle, -step, +step);
            currentServoAngle = clampf(currentServoAngle + delta, SERVO::MIN_DEG, SERVO::MAX_DEG);
            servo->write(currentServoAngle);
        };

        void handleCurve(float rollDeg) {
            rollDeg -= rollDegOffset;

            // 1) Low-Pass-Filter
            float alpha = lastToCurrent / (LPF_TAU_S + lastToCurrent);
            rollDegFiltered += alpha * (rollDeg - rollDegFiltered);

            // === Zustandserkennung mit Hysterese + Haltezeit ===
            float absLean = fabsf(rollDegFiltered);
            switch (state) {
                case RideState::STRAIGHT:
                if (absLean >= LEAN_ENTER_DEG) {
                    if (currentTimestamp - stateTimerStart >= ENTER_HOLD_MS) {
                    state = RideState::CURVE;
                    stateTimerStart = currentTimestamp;
                    }
                } else {
                    stateTimerStart = currentTimestamp;
                }
                break;

                case RideState::CURVE:
                if (absLean <= LEAN_EXIT_DEG) {
                    if (currentTimestamp - stateTimerStart >= EXIT_HOLD_MS) {
                    state = RideState::STRAIGHT;
                    stateTimerStart = currentTimestamp;
                    }
                } else {
                    stateTimerStart = currentTimestamp;
                }
                break;
            }

            // // === Ziel-Servowinkel bestimmen ===
            float targetDeg;
            if (state == RideState::STRAIGHT) {
                targetDeg = SERVO::NEUTRAL_DEG;

                // langsames, driftfreies Nachzentrieren (optional)
                if (AUTO_RECENTER_ENABLE) {
                    float rec = AUTO_RECENTER_RATE_DPS * lastToCurrent;
                    float err = rollDeg;
                    if (fabsf(err) > 0.1f) {
                        rollDegOffset += (err > 0 ? +rec : -rec);
                    }
                }

            } else {
                // Kurvenfahrt aktiv: Servo proportional zum gefilterten Neigewinkel
                float cmd = SERVO::NEUTRAL_DEG + SERVO::GAIN * rollDegFiltered;

                if (fabsf(cmd - SERVO::NEUTRAL_DEG) < OUTPUT_DEADBAND_DEG) {
                cmd = SERVO::NEUTRAL_DEG;
                }
                targetDeg = clampf(cmd, SERVO::MIN_DEG, SERVO::MAX_DEG);
            }

            // === Slew-Rate-Limiter ===
            float maxStep = SERVO::MAX_SPEED_DPS * lastToCurrent;
            float delta   = clampf(targetDeg - currentServoAngle, -maxStep, +maxStep);
            currentServoAngle = clampf(currentServoAngle + delta, SERVO::MIN_DEG, SERVO::MAX_DEG);

            servo->write(currentServoAngle);
        };
};