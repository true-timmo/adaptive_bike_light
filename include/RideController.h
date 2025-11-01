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
  static constexpr int PIN             = 10;
  static constexpr int PWM_MIN         = 1000;
  static constexpr int PWM_MAX         = 2000;
  static constexpr float MAX_SPEED_DPS = 360.0f;
  static constexpr float NEUTRAL_DEG   = 90.0f;
  static constexpr float GEAR_OFFSET   = -7.0f;
  static constexpr float MIN_DEG       = 30.0f;
  static constexpr float MAX_DEG       = 150.0f;
  static constexpr float GAIN          = -5.5f;
};

enum class RideState { STRAIGHT, CURVE };

class RideController {
    private:
        static constexpr uint32_t CALIB_TIME_MS = 2000;
        static constexpr bool AUTO_RECENTER_ENABLE = true;
        static constexpr float AUTO_RECENTER_RATE_DPS = 0.05f;

        // Gyro-Assist Einstellungen
        static constexpr float YAW_NORM   = 80.0f;   // °/s für volle Yaw-Gewichtung
        static constexpr float ROLL_NORM  = 10.0f;   // °  für „Roll ist schon groß“
        static constexpr float K_YAW      = 0.06f;   // Basiseinfluss der Yaw-Rate (Feintuning)

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
        float yawBias             = 0.0f;
        float rollDegFiltered     = 0.0f; // gefilterter Rollwinkel nach Offset
        float currentServoAngle   = SERVO::NEUTRAL_DEG;

        uint32_t lastTimestamp    = 0;
        float lastToCurrent       = 0.0f;
        uint32_t currentTimestamp = 0;
        uint32_t stateTimerStart  = 0;

        static inline float clampf(float v, float lo, float hi) {
            return v < lo ? lo : (v > hi ? hi : v);
        };

        float neutralAngle() const {
            return SERVO::NEUTRAL_DEG + SERVO::GEAR_OFFSET; // + SERVO::GAIN * (-rollDegOffset);
        }

        float minAngle() {
            return SERVO::MIN_DEG + SERVO::GEAR_OFFSET;

        };

        float maxAngle() {
            return SERVO::MAX_DEG + SERVO::GEAR_OFFSET;
        }

    public:
        RideController(MotionSensor* s, Servo* v) : sensor(s), servo(v) {}

        void init(MotionData calibrationData) {
            rollDegOffset = calibrationData.roll;
            yawBias = calibrationData.yaw;
            
            servo->setPeriodHertz(50);
            servo->attach(SERVO::PIN, SERVO::PWM_MIN, SERVO::PWM_MAX);
            servo->write(minAngle());
            delay(500);
            servo->write(maxAngle());
            delay(500);
            servo->write(neutralAngle());
        }

        void setTiming() {
            currentTimestamp = millis();
            lastToCurrent = (currentTimestamp - lastTimestamp) / 1000.0f;
            if (lastToCurrent <= 0.0f || lastToCurrent > 0.5f) {
                lastToCurrent = 0.02f;
            }
            lastTimestamp = currentTimestamp;
        }

        MotionData runCalibration(uint32_t duration_ms = 2000) {
            Serial.println("Kalibriere... bitte Fahrrad/Mechanik aufrecht halten.");
            rollDegOffset = sensor->calibrateRollAngle();
            Serial.println(F("Kalibriere Gyro-Bias... Bitte nicht bewegen."));
            yawBias = sensor->calibrateGyroBias();

            Serial.printf("Kalibrierung fertig. Neuer Offset = %.2f°\n", rollDegOffset);
            Serial.printf("Gyro-Z-Bias: %.3f °/s\n", yawBias);
            servo->write(neutralAngle());

            return MotionData(rollDegOffset, yawBias);
        };

        void turnNeutral() {
            float step = SERVO::MAX_SPEED_DPS * lastToCurrent;
            float target = neutralAngle();
            float delta = clampf(target - currentServoAngle, -step, +step);
            currentServoAngle = clampf(currentServoAngle + delta, minAngle(), maxAngle());

            servo->write(currentServoAngle);
        }

        void handleCurve(MotionData motionData) {
            float rollDeg = motionData.roll - rollDegOffset;  // kalibrierter Roll
            float yawRate = motionData.yaw;                   // °/s (Bias bereits im Sensor korrigiert)

            // --- 1) Adaptive Glättung: je kleiner Yaw, desto stärkeres LPF (Gerade ruhiger) ---
            float yawMag   = fabsf(yawRate);
            float yawFrac  = fminf(yawMag / YAW_NORM, 1.0f);                   // 0..1
            float dynTau   = LPF_TAU_S * (1.0f + 0.6f * (1.0f - yawFrac));     // 0.25..0.4 s
            float alpha    = lastToCurrent / (dynTau + lastToCurrent);
            rollDegFiltered += alpha * (rollDeg - rollDegFiltered);

            // --- 2) Zustandserkennung (Hysterese) – Eintritt erleichtern, wenn Yaw groß ---
            float absLean = fabsf(rollDegFiltered);
            float leanEnterDyn = LEAN_ENTER_DEG - 1.5f * yawFrac;              // bis ~1.5° leichter
            if (leanEnterDyn < 1.0f) leanEnterDyn = 1.0f;

            switch (state) {
            case RideState::STRAIGHT:
                if (absLean >= leanEnterDyn || yawMag >= 20.0f) {
                    if (currentTimestamp - stateTimerStart >= ENTER_HOLD_MS) {
                        state = RideState::CURVE;
                        stateTimerStart = currentTimestamp;
                    }
                } else {
                    stateTimerStart = currentTimestamp;
                }
                break;

            case RideState::CURVE:
                if ((absLean <= LEAN_EXIT_DEG && yawMag < 10.0f)) {
                    if (currentTimestamp - stateTimerStart >= EXIT_HOLD_MS) {
                        state = RideState::STRAIGHT;
                        stateTimerStart = currentTimestamp;
                    }
                } else {
                    stateTimerStart = currentTimestamp;
                }
                break;
            }

            // --- 3) Zielwinkel: Roll dominiert bei großer Schräglage, Yaw hilft v. a. bei kleinem Roll ---
            float targetDeg;
            if (state == RideState::STRAIGHT) {
                targetDeg = neutralAngle();

                if (AUTO_RECENTER_ENABLE) {
                    float rec = AUTO_RECENTER_RATE_DPS * lastToCurrent;
                    float err = rollDeg;
                    if (fabsf(err) > 0.1f) rollDegOffset += (err > 0 ? +rec : -rec);
                }
            } else {
                // Yaw-Gewichtung wird klein, wenn Roll schon groß ist:
                float rollFrac = fminf(fabsf(rollDegFiltered) / ROLL_NORM, 1.0f); // 0..1
                float yawWeight = (1.0f - rollFrac) * yawFrac;                    // groß: kleiner Roll + große Yaw
                float blended = rollDegFiltered + (K_YAW * yawWeight) * yawRate;

                float cmd = neutralAngle() + SERVO::GAIN * blended;
                if (fabsf(cmd - neutralAngle()) < OUTPUT_DEADBAND_DEG)
                    cmd = neutralAngle();

                targetDeg = clampf(cmd, minAngle(), maxAngle());
            }

            // --- 4) Slew-Rate-Limiter + optional „nur schreiben, wenn’s sich lohnt“ ---
            float maxStep = SERVO::MAX_SPEED_DPS * lastToCurrent;
            float delta   = clampf(targetDeg - currentServoAngle, -maxStep, +maxStep);
            float next    = clampf(currentServoAngle + delta, minAngle(), maxAngle());

            if (fabsf(next - currentServoAngle) > 0.2f) { // vermeidet Sirren
                currentServoAngle = next;

                Serial.printf("CurrentAngle: %f°", currentServoAngle);
                servo->write(currentServoAngle);
            }
        }
};