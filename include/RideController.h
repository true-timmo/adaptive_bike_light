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
  static constexpr int PIN                  = 10;
  static constexpr int PWM_MIN              = 1000;
  static constexpr int PWM_MAX              = 2000;
  static constexpr float MAX_SPEED_DPS      = 360.0f;
  static constexpr float NEUTRAL_DEG        = 90.0f;
  static constexpr float GEAR_OFFSET        = -7.0f;
  static constexpr float MIN_DEG            = 30.0f;
  static constexpr float MAX_DEG            = 150.0f;
  static constexpr float GAIN               = -6.0f;
  static constexpr float WRITE_DEADBAND_DEG = 0.2f;
};

enum class RideState { STRAIGHT, CURVE };

class RideController {
    private:
        static constexpr bool AUTO_RECENTER_ENABLE    = true;
        static constexpr float AUTO_RECENTER_RATE_DPS = 0.05f;

        static constexpr float    G_MPS2          = 9.80665f; // m/s^2
        static constexpr float    SHOCK_THR_Z     = 3.0f;
        static constexpr uint32_t SHOCK_HOLD_MS   = 100;

        // --- Snap / Richtungswechsel-Booster ---
        static constexpr float    YAW_SNAP_THR_DEGS = 80.0f;   // ab dieser Yaw-Rate (°/s) gilt es als „kräftig“
        static constexpr uint32_t SNAP_HOLD_MS      = 150;     // so lange Boost aktiv (ms)
        static constexpr float    K_YAW_SNAP        = 0.10f;   // extra Yaw-Gewichtung im Snap-Fenster
        static constexpr float    SNAP_SPEED_MULT   = 2.0f;    // Slew-Rate-Multiplikator im Snap-Fenster

        // Gyro-Assist Einstellungen
        static constexpr float YAW_NORM   = 80.0f;   // °/s für volle Yaw-Gewichtung
        static constexpr float ROLL_NORM  = 10.0f;   // °  für „Roll ist schon groß“
        static constexpr float K_YAW      = 0.07f;   // Basiseinfluss der Yaw-Rate (Feintuning)

        static constexpr float LEAN_ENTER_DEG   = 3.0f;  // ab diesem gefilterten Rollwinkel: "Kurve"
        static constexpr float LEAN_EXIT_DEG    = 2.0f;  // darunter zurück zu "Gerade"
        static constexpr uint32_t ENTER_HOLD_MS = 140;   // Mindestdauer für Eintritt
        static constexpr uint32_t EXIT_HOLD_MS  = 400;   // Mindestdauer für Austritt

        // --- „Schwanken“ glätten ---
        static constexpr float LPF_TAU_S           = 0.25f;  // größer = stärker geglättet
        static constexpr float OUTPUT_DEADBAND_DEG = 0.8f;

        MotionSensor *sensor;
        Servo *servo;
        RideState state = RideState::STRAIGHT;

        float rollDegOffset       = 0.0f;
        float rollDegFiltered     = 0.0f;
        float yawBias             = 0.0f;
        float prevYawRate         = 0.0f;
        float currentServoAngle   = SERVO::NEUTRAL_DEG;

        uint32_t stateTimerStart  = 0;
        uint32_t currentTimestamp = 0;
        uint32_t lastTimestamp    = 0;
        float lastToCurrent       = 0.0f;
        uint32_t snapHoldUntil    = 0;
        uint32_t shockHoldUntil   = 0;

        void writeServoAngle(float target, float multiplier = 1.0f) {
            const float step   = (SERVO::MAX_SPEED_DPS * multiplier) * lastToCurrent;
            const float delta  = clampf(target - currentServoAngle, -step, +step);
            const float next   = clampf(currentServoAngle + delta, minAngle(), maxAngle());

            // Slew-Rate-Limiter + optional „nur schreiben, wenn’s sich lohnt“
            if (fabsf(next - currentServoAngle) > SERVO::WRITE_DEADBAND_DEG) {
                currentServoAngle = next;
                servo->write(currentServoAngle);
            }
        };

        bool snapBoostActive(float yawRate) {
            if ((yawRate * prevYawRate) < 0.0f && (fmaxf(fabsf(yawRate), fabsf(prevYawRate)) >= YAW_SNAP_THR_DEGS)) {
                snapHoldUntil = currentTimestamp + SNAP_HOLD_MS;
            }
            prevYawRate = yawRate;

            return currentTimestamp < snapHoldUntil;
        };

        bool shockDetected(Accel *accel) {
            if (!accel->valid) return false;

            static float zAvg = G_MPS2;
            zAvg += 0.2f * (accel->z - zAvg);
            float devZ = fabsf(zAvg - G_MPS2);

            if (devZ > SHOCK_THR_Z  && currentTimestamp > shockHoldUntil) {
                shockHoldUntil = currentTimestamp + SHOCK_HOLD_MS;
            }

            if (currentTimestamp < shockHoldUntil) {
                //writeServoAngle(neutralAngle(), 0.5f);
                return true;
            }

            return false;
        };

        static inline float clampf(float v, float lo, float hi) {
            return v < lo ? lo : (v > hi ? hi : v);
        };

        float neutralAngle() const {
            return SERVO::NEUTRAL_DEG + SERVO::GEAR_OFFSET;
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

        MotionData runCalibration() {
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
            writeServoAngle(neutralAngle());
        }

        void handleCurve(MotionData motionData) {
            if (shockDetected(&motionData.accel)) return;

            float rollDeg = motionData.roll - rollDegOffset;  // kalibrierter Roll
            bool snapBoost = snapBoostActive(motionData.yaw);
            float yawRate = motionData.yaw * (snapBoost ? (1.0f + K_YAW_SNAP) : 1.0f);
            float stepMultiplier = snapBoost ? SNAP_SPEED_MULT : 1.0f;

            // --- 1) Adaptive Glättung: je kleiner Yaw, desto stärkeres LPF (Gerade ruhiger) ---
            float yawMag   = fabsf(motionData.yaw);
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

            writeServoAngle(targetDeg, stepMultiplier);
        }
};