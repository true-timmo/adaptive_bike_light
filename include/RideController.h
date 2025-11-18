#pragma once
#include <Arduino.h>

#ifndef ESP32_Servo_h
#include <ESP32Servo.h>
#endif

#ifndef MotionSensor_h
#include "MotionSensor.h"
#endif

#ifndef _MotionFilter_h
#include "MotionFilter.h"
#endif

#ifndef CurveDetector_h
#include "CurveDetector.h"
#endif

struct SERVO {
  static constexpr int PIN                  = 20;
  static constexpr int PWM_MIN              = 500;
  static constexpr int PWM_MAX              = 2000;
  static constexpr float MAX_SPEED_DPS      = 240.0f;
  static constexpr float NEUTRAL_DEG        = 90.0f;
  static constexpr float MECHANICAL_OFFSET  = -0.0f;
  static constexpr float MIN_DEG            = 20.0f;
  static constexpr float MAX_DEG            = 160.0f;
  static constexpr float GAIN               = -6.0f;
  static constexpr float WRITE_DEADBAND_DEG = 1.0f;
};

enum class RideState { STRAIGHT, CURVE };

class RideController {
    private:
        // Gyro-Assist Einstellungen
        static constexpr float YAW_NORM   = 40.0f;   // °/s für volle Yaw-Gewichtung
        static constexpr float ROLL_NORM  = 10.0f;   // °  für „Roll ist schon groß“
        static constexpr float K_YAW      = 0.07f;   // Basiseinfluss der Yaw-Rate (Feintuning)

        static constexpr float LEAN_ENTER_DEG   = 2.0f;  // ab diesem gefilterten Rollwinkel: "Kurve"
        static constexpr float LEAN_EXIT_DEG    = 1.0f;  // darunter zurück zu "Gerade"
        static constexpr uint32_t ENTER_HOLD_MS = 140;   // Mindestdauer für Eintritt
        static constexpr uint32_t EXIT_HOLD_MS  = 400;   // Mindestdauer für Austritt
        static constexpr float YAW_ENTER_MIN_DPS    = 1.5f;   // Yaw, die „echte“ Lenkung andeutet
        static constexpr float YAW_ENTER_STRONG_DPS = 15.0f;  // starke Lenkbewegung => Kurve auch ohne viel Lean

        static constexpr float LPF_TAU_S = 0.12f;
        static constexpr float CURVE_BOOST_FACTOR = 0.5f;

        Stream *logger;
        MotionSensor *sensor;
        Servo *servo;
        RideState state = RideState::STRAIGHT;
        MotionFilter filter;
        CurveDetector detector;

        float rollDegFiltered     = 0.0f;
        bool servoEnabled         = false;
        bool loggingEnabled       = false;
        bool curveBoostEnabled    = false;
        float currentServoAngle   = SERVO::NEUTRAL_DEG;

        uint32_t stateTimerStart  = 0;
        uint32_t currentTimestamp = 0;
        uint32_t lastTimestamp    = 0;
        float lastToCurrent       = 0.0f;
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

        void logEverything(float gyroYaw, float gyroRoll, float accRollDeg, float accRollFiltered, float yawFrac, RideState rideState, float multiplier, float servoPos) {
            if (!loggingEnabled) return;

            static uint32_t lastLogMs = currentTimestamp;

            if (currentTimestamp - lastLogMs >= 20) {
                lastLogMs = currentTimestamp;
                logger->printf("|%+.2f|%+.2f|%+.2f|%+.2f|%+.2f|%d|%+.1f|%+.1f\n",
                gyroYaw, gyroRoll, accRollDeg, accRollFiltered, yawFrac, rideState, multiplier, servoPos);
            }
        };

        static inline float clampf(float v, float lo, float hi) {
            return v < lo ? lo : (v > hi ? hi : v);
        };

        float neutralAngle() const {
            return SERVO::NEUTRAL_DEG + SERVO::MECHANICAL_OFFSET;
        }

        float minAngle() {
            return SERVO::MIN_DEG + SERVO::MECHANICAL_OFFSET;

        };

        float maxAngle() {
            return SERVO::MAX_DEG + SERVO::MECHANICAL_OFFSET;
        }

    public:
        RideController(MotionSensor* s, Servo* v, Stream* l) : 
            sensor(s),
            servo(v),
            logger(l),
            filter(l, currentTimestamp, lastToCurrent),
            detector(l, currentTimestamp, currentServoAngle, neutralAngle())
        {};

        bool toggleCurveBoostState() {
            curveBoostEnabled = !curveBoostEnabled;

            return curveBoostEnabled;
        }

        void setLoggingState(bool state) {
            if (state != loggingEnabled) {
                loggingEnabled = state;
            }
        }

        void setServoState(bool state) {
            if (state != servoEnabled) {
                if (state == true) {
                    servo->setPeriodHertz(50);
                    servo->attach(SERVO::PIN, SERVO::PWM_MIN, SERVO::PWM_MAX);                    
                    logger->printf("Servo attached. PIN:%d\n", SERVO::PIN);
                } else {
                    servo->detach();
                    logger->println(F("Servo detached."));
                }

                servoEnabled = state;
            }
        }

        void setTiming() {
            currentTimestamp = millis();
            lastToCurrent = (currentTimestamp - lastTimestamp) / 1000.0f;
            if (lastToCurrent <= 0.0f || lastToCurrent > 0.5f) {
                lastToCurrent = 0.02f;
            }
            lastTimestamp = currentTimestamp;
        }

        void runCalibration() {
            delay(300);
            logger->println(F("Calibrating roll angle..."));
            const Accel a = sensor->calibrateAccel();
            logger->println(F("Calibrating Gyro-Bias..."));
            const MotionData g = sensor->calibrateGyro();

            logger->println(F("Calibration done."));
            logger->printf("X-Offset = %.2f°, Y-Offset = %.2f°, Z-Offset = %.2f°\n", a.x, a.y, a.z);
            logger->printf("Gyro-X-Bias: %.3f °/s, Gyro-Z-Bias: %.3f °/s\n", g.gyroRoll, g.gyroYaw);

            turnNeutral();
        };

        void turnNeutral() {
            logger->println(F("Turn neutral"));
            currentServoAngle = neutralAngle();
            servo->write(currentServoAngle);
        }

        void turnRight() {
            logger->println(F("Turn right"));
            currentServoAngle = minAngle();
            servo->write(currentServoAngle);
        }

        void turnLeft() {
            logger->println(F("Turn left"));
            currentServoAngle = maxAngle();
            servo->write(currentServoAngle);
        }

        void handleCurve(MotionData motionData) {
            FilteredData filteredData = filter.handle(motionData);
            if (filteredData.isShock) return;

            // Adaptive Glättung: je kleiner Yaw, desto stärkeres LPF (Gerade ruhiger) ---
            float yawMag   = fabsf(filteredData.gyroYaw);
            float yawFrac  = fminf(yawMag / YAW_NORM, 1.0f);
            float dynTau = LPF_TAU_S * (1.0f + 0.6f * (1.0f - yawFrac));
            if (state == RideState::STRAIGHT) {
                dynTau *= 1.5f;
            }
            float alpha    = lastToCurrent / (dynTau + lastToCurrent);

            rollDegFiltered += alpha * (filteredData.accelRollDeg - rollDegFiltered);
            float rollFrac = fminf(fabsf(rollDegFiltered) / ROLL_NORM, 1.0f);
            float yawWeight = (1.0f - rollFrac) * yawFrac;
            yawWeight = clampf(yawWeight, 0.0f, 0.5f);
            float blended = rollDegFiltered + (K_YAW * yawWeight) * filteredData.gyroYaw;
            float targetDeg = neutralAngle() + SERVO::GAIN * blended;

            bool isCurve = detector.curveDetected(filteredData);
            float multiplier = 1.0f;
            if (curveBoostEnabled && isCurve) {
                float curveBias = detector.getCurveBias();
                int servoDir = (targetDeg > currentServoAngle) ? +1 : -1;

                if (curveBias * servoDir > 0.0f) {
                    multiplier = (1.0f + CURVE_BOOST_FACTOR * fabsf(curveBias));
                } else if (curveBias * servoDir < 0.0f) {
                    multiplier = (1.0f - CURVE_BOOST_FACTOR * fabsf(curveBias));
                }
            }

            // Zustandserkennung (Hysterese) – Eintritt erleichtern, wenn Yaw groß ---
            float absLean = fabsf(rollDegFiltered);
            // Dynamische Lean-Schwelle beibehalten, aber nicht unter 1° fallen lassen
            float leanEnterDyn = LEAN_ENTER_DEG - 1.5f * yawFrac;
            if (leanEnterDyn < 1.0f) leanEnterDyn = 1.0f;

            bool leanTrigger   = (absLean >= leanEnterDyn);
            bool yawAssist     = (yawMag >= YAW_ENTER_MIN_DPS);     // etwas Lenken
            bool yawStrongOnly = (yawMag >= YAW_ENTER_STRONG_DPS);  // sehr starke Lenkung

            switch (state) {
                case RideState::STRAIGHT:
                    if ((leanTrigger && yawAssist) || yawStrongOnly) {
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
            };

            if (state == RideState::STRAIGHT) {
                targetDeg = neutralAngle();
            } else {             
                targetDeg = clampf(targetDeg, minAngle(), maxAngle());
            }

            logEverything(
                filteredData.gyroYaw, filteredData.gyroRoll, filteredData.accelRollDeg,
                rollDegFiltered, yawFrac, state, multiplier, targetDeg
            );

            writeServoAngle(targetDeg, multiplier);
        }
};