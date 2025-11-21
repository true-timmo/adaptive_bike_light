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
  static constexpr int PWM_MIN              = 850;
  static constexpr int PWM_MAX              = 2000;
  static constexpr float MAX_SPEED_DPS      = 240.0f;
  static constexpr float MAX_DEG            = 180.0f;
  static constexpr float NEUTRAL_DEG        = 90.0f;
  static constexpr float MIN_DEG            = 0.0f;
  static constexpr float GEAR_RATIO         = 5.5f;
  static constexpr float GEAR_SIGN          = -1.0f;
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

        static constexpr float SYSTEM_CLK_MS = 5.0f;
        static constexpr float SYSTEM_DT_S = 0.001f * SYSTEM_CLK_MS;
        static constexpr float SERVO_CLK_MS = 20.0f;
        static constexpr float DEVICE_TIMEOUT_MS = 1000.0f * 120.0f;
        static constexpr float LPF_TAU_S = 0.12f;
        static constexpr float CURVE_BOOST_FACTOR = 0.5f;

        Stream *logger;
        MotionSensor *sensor;
        Servo *servo;
        RideState state = RideState::STRAIGHT;
        MotionFilter filter;
        CurveDetector detector;

        int8_t gearOffset         = 0;
        float gearRatio           = 5.5f;
        float rollDegFiltered     = 0.0f;
        bool servoEnabled         = false;
        bool loggingEnabled       = false;
        bool curveBoostEnabled    = false;
        float currentServoAngle   = SERVO::NEUTRAL_DEG;

        float lastServoWrittenAngle = SERVO::NEUTRAL_DEG;
        uint32_t lastServoWriteMs   = 0;

        uint32_t stateTimerStart  = 0;
        uint32_t currentTimestamp = 0;
        uint32_t lastTimestamp    = 0;
        uint32_t shockHoldUntil   = 0;

        bool writeServoAngle(float target, float multiplier = 1.0f) {
            const float step = (SERVO::MAX_SPEED_DPS * multiplier) / 1000 * SYSTEM_CLK_MS;
            const float delta  = clampf(target - currentServoAngle, -step, +step);
            const float next   = clampf(currentServoAngle + delta, minAngle(), maxAngle());
            currentServoAngle = next;

            if (servoEnabled 
                && currentTimestamp - lastServoWriteMs >= SERVO_CLK_MS
                && fabsf(currentServoAngle - lastServoWrittenAngle) > SERVO::WRITE_DEADBAND_DEG
            ) {
                lastServoWriteMs = currentTimestamp;
                servo->write(currentServoAngle);
                lastServoWrittenAngle = currentServoAngle;

                return true;
            }

            return false;
        };

        void logEverything(float gyroYaw, float gyroRoll, float accRollDeg, float accRollFiltered, float yawFrac, RideState rideState, float multiplier, float servoPos, bool servoInSync) {
            static uint32_t lastLogMs = currentTimestamp;

            if (loggingEnabled && currentTimestamp - lastLogMs >= 20) {
                lastLogMs = currentTimestamp;
                logger->printf("|%+.2f|%+.2f|%+.2f|%+.2f|%+.1f|%+.1f|%d\n",
                gyroYaw, gyroRoll, accRollDeg, accRollFiltered, multiplier, servoPos, servoInSync);
            }
        };

        static inline float clampf(float v, float lo, float hi) {
            return v < lo ? lo : (v > hi ? hi : v);
        };

        float neutralAngle() const {
            return SERVO::NEUTRAL_DEG + gearOffset;
        }

        float minAngle() {
            return SERVO::MIN_DEG + fabsf(gearOffset);

        };

        float maxAngle() {
            return SERVO::MAX_DEG - fabsf(gearOffset);
        }

    public:
        RideController(MotionSensor* s, Servo* v, Stream* l) : 
            sensor(s),
            servo(v),
            logger(l),
            filter(l),
            detector(currentTimestamp)
        {};

        bool isTimedOut() {
            return currentTimestamp - lastServoWriteMs >= DEVICE_TIMEOUT_MS;
        }

        void setCurveBoostState(bool _curveBoost) {
            curveBoostEnabled = _curveBoost;
        }

        void setGearOffset(int8_t offset) {
            gearOffset = offset;
        }

        void setGearRatio(float ratio) {
            gearRatio = ratio;
        }

        void setLoggingState(bool _state) {
            if (_state != loggingEnabled) {
                loggingEnabled = _state;
            }
        }

        void setServoState(bool _servoState) {
            if (_servoState == true) {
                servo->setPeriodHertz(50);
                servo->attach(SERVO::PIN, SERVO::PWM_MIN, SERVO::PWM_MAX);                    
                logger->printf("Servo attached. PIN:%d\n", SERVO::PIN);
                delay(100);
            } else {
                delay(100);
                servo->release();
                servo->detach();
                logger->println(F("Servo detached."));
            }

            servoEnabled = _servoState;
        }

        void setTiming() {
            currentTimestamp = millis();
            lastTimestamp = currentTimestamp;
        }

        void delayNext() {
            uint32_t elapsed = millis() - currentTimestamp;
            if (elapsed < SYSTEM_CLK_MS) {
                delay((uint32_t)(SYSTEM_CLK_MS - elapsed));
            }
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
            float alpha = SYSTEM_DT_S / (dynTau + SYSTEM_DT_S);

            rollDegFiltered += alpha * (filteredData.accelRollDeg - rollDegFiltered);
            float rollFrac = fminf(fabsf(rollDegFiltered) / ROLL_NORM, 1.0f);
            float yawWeight = (1.0f - rollFrac) * yawFrac;
            yawWeight = clampf(yawWeight, 0.0f, 0.5f);
            float blended = rollDegFiltered + (K_YAW * yawWeight) * filteredData.gyroYaw;
            float targetDeg = neutralAngle() + gearRatio * blended * SERVO::GEAR_SIGN;

            float multiplier = 1.0f;
            if (curveBoostEnabled && detector.curveDetected(filteredData)) {
                float curveBias = detector.getCurveBias();
                int servoDir = (targetDeg > currentServoAngle) ? -1 : +1;

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
            bool curveAssist   = fabsf(1.0 - multiplier) > 0.3;
            bool yawAssist     = (yawMag >= YAW_ENTER_MIN_DPS);    // etwas Lenken
            bool yawStrongOnly = (yawMag >= YAW_ENTER_STRONG_DPS);  // sehr starke Lenkung

            switch (state) {
                case RideState::STRAIGHT:
                    if ((leanTrigger && yawAssist) || yawStrongOnly || curveAssist) {
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

            bool servoWritten = writeServoAngle(targetDeg, multiplier);

            logEverything(
                filteredData.gyroYaw, filteredData.gyroRoll, filteredData.accelRollDeg,
                rollDegFiltered, yawFrac, state, multiplier, targetDeg, servoWritten
            );

            delayNext();
        }
};