#ifndef _CurveDetector_h
#define _CurveDetector_h

#include <Arduino.h>

#ifndef _MotionFilter_h
#include "MotionFilter.h"
#endif

enum Direction {
    LEFT, NEUTRAL, RIGHT
};

class CurveDetector {
private:
    static constexpr float CURVE_CHECK_INTERVAL_MS = 50.0f;
    static constexpr float GYRO_THRESHOLD          = 0.5f;
    static constexpr float ROLL_DEG_THRESHOLD      = 0.2f;

    uint32_t& now;
    uint32_t  lastTimestamp = 0;

    float  neutralAngle;
    float& currentServoAngle;

    float lastServoAngle     = NAN;
    float lastGyroYaw        = 0.0f;
    float lastAccelRollDeg   = 0.0f;

    Direction lastGyroYawDirection   = Direction::NEUTRAL;
    Direction lastAccellRollDirection= Direction::NEUTRAL;
    Direction lastServoDirection     = Direction::NEUTRAL;

    // neue State-Variablen:
    Direction currentCurveDir        = Direction::NEUTRAL;
    float     currentCurveBias       = 0.0f;
    Direction lastStableCurveDir     = Direction::NEUTRAL;
    uint8_t   stableCount            = 0;

    Stream* logger;

    void setLastData(FilteredData& filteredData) {
        // Beim allerersten Mal ist lastServoAngle noch NAN → Servo-Richtung nicht aus Delta ableiten
        if (isfinite(lastServoAngle)) {
            lastServoDirection = getIntervalDirection(lastServoAngle, currentServoAngle, 0.0f);
        } else {
            lastServoDirection = getServoDirection();
        }
        lastServoAngle = currentServoAngle;

        lastGyroYawDirection = getIntervalDirection(lastGyroYaw, filteredData.gyroYaw, GYRO_THRESHOLD);
        lastGyroYaw          = filteredData.gyroYaw;

        lastAccellRollDirection = getIntervalDirection(lastAccelRollDeg, filteredData.accelRollDeg, ROLL_DEG_THRESHOLD);
        lastAccelRollDeg        = filteredData.accelRollDeg;

        lastTimestamp = now;
    }

    Direction getDirection(float deg) {
        if (deg == 0.0f) return Direction::NEUTRAL;
        return deg > 0.0f ? Direction::RIGHT : Direction::LEFT;
    }

    Direction getIntervalDirection(float from, float to, float neutralThreshold) {
        if (fabsf(to - from) < neutralThreshold) return Direction::NEUTRAL;
        return (from > to) ? Direction::LEFT : Direction::RIGHT;
    }

    Direction getServoDirection() {
        if (currentServoAngle == neutralAngle) return Direction::NEUTRAL;
        // bei dir: größerer Winkel = nach links schauen
        return currentServoAngle > neutralAngle ? Direction::LEFT : Direction::RIGHT;
    }

    // Hilfsfunktion: Bias anpassen je nach Richtung
    void addWeight(Direction d, float w, float& bias) {
        if (d == Direction::LEFT)  bias -= w;
        if (d == Direction::RIGHT) bias += w;
    }

public:
    CurveDetector(Stream* l,  uint32_t& currentTimestamp, float& csa, float na)
    : logger(l), now(currentTimestamp), currentServoAngle(csa), neutralAngle(na) {}

    Direction getCurveDirection() const { return currentCurveDir; }
    float     getCurveBias() const      { return currentCurveBias; }

    bool curveDetected(FilteredData& filteredData) {
        // Erstes Sample: nur initialisieren
        if (!isfinite(lastServoAngle)) {
            setLastData(filteredData);
            currentCurveDir  = Direction::NEUTRAL;
            currentCurveBias = 0.0f;
            return false;
        }

        // Nur alle 50 ms echte Analyse machen:
        if (now - lastTimestamp < CURVE_CHECK_INTERVAL_MS) {
            // schon einmal etwas erkannt? -> Status bleibt gültig
            return (currentCurveDir != Direction::NEUTRAL);
        }

        // Bewegungsrichtungen seit letztem Check bestimmen
        Direction yawDir   = getIntervalDirection(lastGyroYaw,        filteredData.gyroYaw,      GYRO_THRESHOLD);
        Direction rollDir  = getIntervalDirection(lastAccelRollDeg,   filteredData.accelRollDeg, ROLL_DEG_THRESHOLD);
        Direction servoDir = getServoDirection(); // absolute Richtung, kein Delta

        float bias = 0.0f;

        // Yaw etwas stärker gewichten (Lenkeinschlag / Kurveneinleitung)
        addWeight(yawDir,  0.6f, bias);
        // Roll etwas schwächer (kommt leicht verzögert)
        addWeight(rollDir, 0.4f, bias);

        // Servo-Bewegung, wenn sie in die gleiche Richtung geht, leicht verstärken
        Direction servoMoveDir = getIntervalDirection(lastServoAngle, currentServoAngle, 0.5f);
        addWeight(servoMoveDir, 0.2f, bias);

        // Physikalische Konsistenz prüfen:
        // Wenn Yaw und Roll sich widersprechen, Bias stark reduzieren
        if (yawDir != Direction::NEUTRAL && rollDir != Direction::NEUTRAL && yawDir != rollDir) {
            bias *= 0.3f; // eher "Neutral / unsicher"
        }

        // Hart unmögliche Situation (z.B. starker Yaw rechts, starker Roll links) komplett neutralisieren
        if (yawDir != Direction::NEUTRAL && rollDir != Direction::NEUTRAL && yawDir != rollDir &&
            fabsf(filteredData.gyroYaw) > 5.0f && fabsf(filteredData.accelRollDeg) > 3.0f) {
            bias = 0.0f;
        }

        // clamp auf [-1..+1]
        if (bias >  1.0f) bias =  1.0f;
        if (bias < -1.0f) bias = -1.0f;

        // Aus Bias eine Richtung ableiten
        Direction candidateDir = Direction::NEUTRAL;
        if (fabsf(bias) >= 0.2f) { // kleine Bias als "neutral" werten
            candidateDir = (bias > 0.0f) ? Direction::RIGHT : Direction::LEFT;
        }

        // Stabilität / S-Kurven-Detektion:
        if (candidateDir == Direction::NEUTRAL) {
            stableCount = 0;
        } else {
            if (candidateDir == lastStableCurveDir) {
                stableCount++;
            } else {
                // Richtungswechsel → potentielle S-Kurve
                // Du kannst hier z.B. den Bias für kurze Zeit härter machen:
                if (lastStableCurveDir != Direction::NEUTRAL) {
                    // S-Kurve erkannt → stärkere Betonung der neuen Richtung
                    bias = (candidateDir == Direction::RIGHT) ? 1.0f : -1.0f;
                }
                stableCount = 1;
                lastStableCurveDir = candidateDir;
            }
        }

        currentCurveDir  = candidateDir;
        currentCurveBias = bias;

        setLastData(filteredData);

        return (currentCurveDir != Direction::NEUTRAL);
    }
};
#endif //_CurveDetector_h