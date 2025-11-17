#ifndef _CurveDetector_h
#define _CurveDetector_h

#include <Arduino.h>

#ifndef _MotionFilter_h
#include "MotionFilter.h"
#endif

enum class Direction: int8_t {
    LEFT = -1,
    NEUTRAL = 0,
    RIGHT = 1
};

class CurveDetector {
private:
    static constexpr float GYRO_THRESHOLD          = 0.5f;
    static constexpr float ROLL_DEG_THRESHOLD      = 0.2f;
    static constexpr float LPF_ALPHA               = 0.25f;
    static constexpr float DIR_MIN_BIAS            = 0.15f;

    static constexpr float GYRO_YAW_DEV_MAX       = 30.0f;
    static constexpr float GYRO_ROLL_DEV_MAX      = 30.0f;
    static constexpr float ACCEL_ROLL_DEV_MAX     = 3.6f;

    uint32_t& now;
    uint32_t  lastTimestamp = 0;

    float  neutralAngle;
    float& currentServoAngle;

    float lastServoAngle  = NAN;
    float lastGyroYaw = 0.0f;
    float lastGyroRoll = 0.0f;
    float lastAccelRoll = 0.0f;

    float lastGyroYawDev      = 0.0f;
    float lastGyroRollDev     = 0.0f;
    float lastAccelRollDev    = 0.0f;

    Direction lastServoDirection     = Direction::NEUTRAL;
    Direction currentCurveDir        = Direction::NEUTRAL;
    float     currentCurveBias       = 0.0f;
    Direction lastStableCurveDir     = Direction::NEUTRAL;
    uint8_t   stableCount            = 0;

    Stream* logger;

    float getFilteredDeviation(float current, float last) {
        return last + LPF_ALPHA * (current - last);
    }

    float calculateDeviationNormalized(float current, float last, float threshold, float maxDevAbs) {
        float delta = current - last;
        if (fabsf(delta) < threshold) {
            return 0.0f;
        }
        float norm = delta / maxDevAbs;
        if (norm > 1.0f)  norm = 1.0f;
        if (norm < -1.0f) norm = -1.0f;
        return norm;
    }

    float calculateLastGyroYawDev(float currentGyroYaw) {
        float devNorm = calculateDeviationNormalized(currentGyroYaw, lastGyroYaw,
                                                     GYRO_THRESHOLD, GYRO_YAW_DEV_MAX);
        return getFilteredDeviation(devNorm, lastGyroYawDev);
    }

    float calculateLastGyroRollDev(float currentGyroRoll) {
        float devNorm = calculateDeviationNormalized(currentGyroRoll, lastGyroRoll,
                                                     GYRO_THRESHOLD, GYRO_ROLL_DEV_MAX);
        return getFilteredDeviation(devNorm, lastGyroRollDev);
    }

    float calculateLastAccelRollDev(float currentAccelRoll) {
        float devNorm = calculateDeviationNormalized(currentAccelRoll, lastAccelRoll,
                                                     ROLL_DEG_THRESHOLD, ACCEL_ROLL_DEV_MAX);
        return getFilteredDeviation(devNorm, lastAccelRollDev);
    }

    void setLastData(FilteredData& filteredData) {
        lastTimestamp = now;

        if (isfinite(lastServoAngle)) {
            lastServoDirection = getIntervalDirection(lastServoAngle, currentServoAngle, 0.0f);
        } else {
            lastServoDirection = getServoDirection();
        }
        lastServoAngle = currentServoAngle;

        lastGyroYaw   = filteredData.gyroYaw;
        lastGyroRoll  = filteredData.gyroRoll;
        lastAccelRoll = filteredData.accelRollDeg;
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
        return currentServoAngle > neutralAngle ? Direction::LEFT : Direction::RIGHT;
    }

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
        if (filteredData.isShock) return false;
        if (!isfinite(lastServoAngle)) {
            lastServoAngle      = currentServoAngle;
            lastGyroYaw         = filteredData.gyroYaw;
            lastGyroRoll        = filteredData.gyroRoll;
            lastAccelRoll       = filteredData.accelRollDeg;
            lastGyroYawDev      = 0.0f;
            lastGyroRollDev     = 0.0f;
            lastAccelRollDev    = 0.0f;
            lastStableCurveDir  = Direction::NEUTRAL;
            stableCount         = 0;
            currentCurveDir     = Direction::NEUTRAL;
            currentCurveBias    = 0.0f;
            lastTimestamp       = now;
            return false;
        }

        lastGyroYawDev   = calculateLastGyroYawDev(filteredData.gyroYaw);
        lastGyroRollDev  = calculateLastGyroRollDev(filteredData.gyroRoll);
        lastAccelRollDev = calculateLastAccelRollDev(filteredData.accelRollDeg);

        float bias = 0.6f * lastGyroYawDev
                   + 0.25f * lastGyroRollDev
                   + 0.15f * lastAccelRollDev;

        if (bias > 1.0f)  bias = 1.0f;
        if (bias < -1.0f) bias = -1.0f;

        Direction candidateDir = Direction::NEUTRAL;
        if (fabsf(bias) >= DIR_MIN_BIAS) {
            candidateDir = (bias > 0.0f) ? Direction::RIGHT : Direction::LEFT;
        }

        // Detect S-Curve:
        if (candidateDir == Direction::NEUTRAL) {
            stableCount = 0;
        } else {
            if (candidateDir == lastStableCurveDir) {
                if (stableCount < 255) stableCount++;
            } else {
                // Curve detected = boost new direction
                if (lastStableCurveDir != Direction::NEUTRAL) {
                    bias = (candidateDir == Direction::RIGHT) ? 1.0f : -1.0f;
                }
                stableCount        = 1;
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