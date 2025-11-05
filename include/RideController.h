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
        static constexpr float    SHOCK_THR_Z     = 6.0f;
        static constexpr uint32_t SHOCK_HOLD_MS   = 100;

        // Snap / S-Kurve
        static constexpr float    YAW_SNAP_THR_DEGS     = 11.0f;  // Peak je Vorzeichenlobe
        static constexpr float    YAW_SNAP_ENERGY_DEG   = 8.0f;   // ∫|yaw| dt in Grad
        static constexpr float    YAW_SNAP_JERK_THR     = 3500.0f;// °/s² (deutlich höher)
        static constexpr uint32_t SNAP_MIN_PHASE_MS     = 70;     // mind. Dauer des vorherigen Lobes
        static constexpr uint32_t SNAP_REFRACTORY_MS    = 300;    // Sperrzeit nach Snap
        static constexpr uint32_t SNAP_HOLD_MS          = 150;    // Boost-Dauer
        static constexpr float    K_YAW_SNAP            = 0.10f;
        static constexpr float    SNAP_SPEED_MULT       = 2.0f;
        static constexpr float    YAW_EPS               = 2.5f;   // Deadzone um 0°/s

        // Gyro-Assist Einstellungen
        static constexpr float YAW_NORM   = 80.0f;   // °/s für volle Yaw-Gewichtung
        static constexpr float ROLL_NORM  = 10.0f;   // °  für „Roll ist schon groß“
        static constexpr float K_YAW      = 0.07f;   // Basiseinfluss der Yaw-Rate (Feintuning)

        static constexpr float LEAN_ENTER_DEG   = 3.0f;  // ab diesem gefilterten Rollwinkel: "Kurve"
        static constexpr float LEAN_EXIT_DEG    = 2.0f;  // darunter zurück zu "Gerade"
        static constexpr uint32_t ENTER_HOLD_MS = 140;   // Mindestdauer für Eintritt
        static constexpr uint32_t EXIT_HOLD_MS  = 400;   // Mindestdauer für Austritt

        // --- „Schwanken“ glätten ---
        static constexpr float LPF_TAU_S           = 0.10f;  // größer = stärker geglättet
        static constexpr float OUTPUT_DEADBAND_DEG = 0.8f;

        Stream *logger;
        MotionSensor *sensor;
        Servo *servo;
        RideState state = RideState::STRAIGHT;

        float rollDegOffset       = 0.0f;
        float rollDegFiltered     = 0.0f;
        float yawBias             = 0.0f;
        float currentServoAngle   = SERVO::NEUTRAL_DEG;

        float prevYawRate      = 0.0f;
        int      prevYawSign   = 0;
        float    peakYawMag    = 0.0f;     // max(|yaw|) im aktuellen Lobe
        float    yawEnergyDeg  = 0.0f;     // ∫|yaw| dt im aktuellen Lobe
        uint32_t phaseStartMs  = 0;        // Startzeit des aktuellen Lobes
        uint32_t lastSnapAt    = 0;        // für Refractory


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
            // Refractory: kurz nach Snap nicht erneut triggern
            if (currentTimestamp - lastSnapAt < SNAP_REFRACTORY_MS) {
                prevYawRate = yawRate;
                return (currentTimestamp < snapHoldUntil);
            }

            // Signum mit Deadzone
            int sign = 0;
            if (yawRate >  YAW_EPS) sign = +1;
            if (yawRate < -YAW_EPS) sign = -1;

            // dt stabilisieren (Jerk-Noise vermeiden)
            float dt = lastToCurrent;
            if (dt <= 0.0f) dt = 0.01f;
            if (dt < 0.002f) dt = 0.002f; // Schutz: unrealistisch kleine dt clampen

            // Innerhalb der aktuellen Vorzeichenphase: Peak & Energie akkumulieren
            if (sign != 0 && sign == prevYawSign) {
                float mag = fabsf(yawRate);
                if (mag > peakYawMag) peakYawMag = mag;
                yawEnergyDeg += mag * dt;  // °/s * s = °
            }

            // Richtungswechsel?
            if (sign != 0 && prevYawSign != 0 && (sign != prevYawSign)) {
                uint32_t phaseDur = currentTimestamp - phaseStartMs;
                float jerk = fabsf((yawRate - prevYawRate) / dt);

                bool enoughDuration = (phaseDur >= SNAP_MIN_PHASE_MS);
                bool enoughEnergy   = (yawEnergyDeg >= YAW_SNAP_ENERGY_DEG);
                bool strongPeak     = (peakYawMag >= YAW_SNAP_THR_DEGS);
                bool strongJerk     = (jerk       >= YAW_SNAP_JERK_THR);

                if (enoughDuration && enoughEnergy && (strongPeak || strongJerk)) {
                    snapHoldUntil = currentTimestamp + SNAP_HOLD_MS;
                    lastSnapAt    = currentTimestamp;
                    logger->printf("%u: SNAP peak=%.1f E=%.1f dur=%ums jerk=%.0f\n",
                                   currentTimestamp, peakYawMag, yawEnergyDeg, phaseDur, jerk);
                }

                // neue Phase initialisieren
                peakYawMag    = fabsf(yawRate);
                yawEnergyDeg  = peakYawMag * dt;
                phaseStartMs  = currentTimestamp;
            }

            // Phasenwechsel initialisieren (Start einer Lobe)
            if (sign != prevYawSign) {
                prevYawSign = sign;
                phaseStartMs = currentTimestamp;
                if (sign == 0) { peakYawMag = 0.0f; yawEnergyDeg = 0.0f; }
                else {
                    float mag = fabsf(yawRate);
                    peakYawMag   = mag;
                    yawEnergyDeg = mag * dt;
                }
            }

            prevYawRate = yawRate;
            return (currentTimestamp < snapHoldUntil);
        }

        bool shockDetected(Accel *accel) {
            if (!accel->valid) return false;

            static float zAvg = G_MPS2;
            zAvg += 0.2f * (accel->z - zAvg);
            float devZ = fabsf(zAvg - G_MPS2);

            if (devZ > SHOCK_THR_Z  && currentTimestamp > shockHoldUntil) {
                shockHoldUntil = currentTimestamp + SHOCK_HOLD_MS;
                logger->printf("%d: Shock detected! Force = %f\n", currentTimestamp, devZ);
            }

            if (currentTimestamp < shockHoldUntil) {
                turnNeutral();
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
        RideController(MotionSensor* s, Servo* v, Stream* l) : sensor(s), servo(v), logger(l) {}

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
            logger->println("Calibrating roll angle...");
            rollDegOffset = sensor->calibrateRollAngle();
            logger->println(F("Calibrating Gyro-Bias..."));
            yawBias = sensor->calibrateGyroBias();

            logger->printf("Calibration done. Roll-Offset = %.2f°, Gyro-Z-Bias: %.3f °/s\n", rollDegOffset, yawBias);
            servo->write(neutralAngle());

            return MotionData(rollDegOffset, yawBias);
        };

        void turnNeutral() {
            writeServoAngle(neutralAngle());
        }

        void handleCurve(MotionData motionData) {
            if (shockDetected(&motionData.accel)) return;

            float rollDeg = motionData.roll - rollDegOffset;
            bool snapBoost = snapBoostActive(motionData.yaw);
            float yawRate = motionData.yaw * (snapBoost ? (1.0f + K_YAW_SNAP) : 1.0f);
            float stepMultiplier = snapBoost ? SNAP_SPEED_MULT : 1.0f;

            // Adaptive Glättung: je kleiner Yaw, desto stärkeres LPF (Gerade ruhiger) ---
            float yawMag   = fabsf(motionData.yaw);
            float yawFrac  = fminf(yawMag / YAW_NORM, 1.0f);                   // 0..1
            float dynTau   = LPF_TAU_S * (1.0f + 0.6f * (1.0f - yawFrac));     // 0.25..0.4 s
            float alpha    = lastToCurrent / (dynTau + lastToCurrent);
            rollDegFiltered += alpha * (rollDeg - rollDegFiltered);

            // Zustandserkennung (Hysterese) – Eintritt erleichtern, wenn Yaw groß ---
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

            // Zielwinkel: Roll dominiert bei großer Schräglage, Yaw hilft v. a. bei kleinem Roll ---
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