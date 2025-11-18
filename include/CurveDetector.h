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
    static constexpr float LPF_ALPHA               = 0.25f;
    static constexpr float DIR_MIN_BIAS            = 0.15f;

    static constexpr float GYRO_YAW_MAX       = 30.0f;
    static constexpr float GYRO_ROLL_MAX      = 30.0f;
    static constexpr float ACCEL_ROLL_MAX     = 3.6f;

    uint32_t& now;
    uint32_t  lastTimestamp = 0;

    float  neutralAngle;
    float& currentServoAngle;

    float lastServoAngle  = NAN;
    float filteredGyroYaw = 0.0f;
    float filteredGyroRoll = 0.0f;
    float filteredAccelRoll = 0.0f;

    Direction lastServoDirection     = Direction::NEUTRAL;
    Direction currentCurveDir        = Direction::NEUTRAL;
    float     currentCurveBias       = 0.0f;
    Direction lastStableCurveDir     = Direction::NEUTRAL;
    uint8_t   stableCount            = 0;

    Stream* logger;

    float getFilteredValue(float current, float last) {
        return last + LPF_ALPHA * (current - last);
    }

    float calculateNormalized(float value, float maxAbsValue) {
        float norm = value / maxAbsValue;
        if (norm > 1.0f)  norm = 1.0f;
        if (norm < -1.0f) norm = -1.0f;
        return norm;
    }

    float calculateGyroYawNorm(float currentGyroYaw) {
        filteredGyroYaw = getFilteredValue(currentGyroYaw, filteredGyroYaw);
        return calculateNormalized(filteredGyroYaw, GYRO_YAW_MAX);
    }

    float calculateGyroRollNorm(float currentGyroRoll) {
        filteredGyroRoll = getFilteredValue(currentGyroRoll, filteredGyroRoll);
        return calculateNormalized(filteredGyroRoll, GYRO_ROLL_MAX);
    }
    
    float calculateAccelRollNorm(float currentAccelRoll) {
        filteredAccelRoll = getFilteredValue(currentAccelRoll, filteredAccelRoll);
        return calculateNormalized(filteredAccelRoll, ACCEL_ROLL_MAX);
    }

    void setLastData(FilteredData& filteredData) {
        lastTimestamp = now;

        if (isfinite(lastServoAngle)) {
            lastServoDirection = getIntervalDirection(lastServoAngle, currentServoAngle, 0.0f);
        } else {
            lastServoDirection = getServoDirection();
        }
        lastServoAngle = currentServoAngle;
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
            filteredGyroYaw     = filteredData.gyroYaw;
            filteredGyroRoll    = filteredData.gyroRoll;
            filteredAccelRoll   = filteredData.accelRollDeg;
            lastStableCurveDir  = Direction::NEUTRAL;
            stableCount         = 0;
            currentCurveDir     = Direction::NEUTRAL;
            currentCurveBias    = 0.0f;
            lastTimestamp       = now;
            return false;
        }

        float gyroYawNorm = calculateGyroYawNorm(filteredData.gyroYaw);
        float gyroRollNowm = calculateGyroRollNorm(filteredData.gyroRoll);
        float accelRollNorm = calculateAccelRollNorm(filteredData.accelRollDeg);

        float bias = 0.6f * gyroYawNorm
                   + 0.25f * gyroRollNowm
                   + 0.15f * accelRollNorm;

        if (bias > 1.0f)  bias = 1.0f;
        if (bias < -1.0f) bias = -1.0f;

        Direction candidateDir = Direction::NEUTRAL;
        if (fabsf(bias) >= DIR_MIN_BIAS) {
            candidateDir = (bias > 0.0f) ? Direction::RIGHT : Direction::LEFT;
        }

        if (candidateDir == Direction::NEUTRAL) {
            stableCount = 0;
        } else {
            if (candidateDir == lastStableCurveDir) {
                stableCount = min(stableCount+1, 10);
            } else {
                stableCount = 0;
                lastStableCurveDir = candidateDir;
            }

            bias += copysign(0.5f * stableCount * 0.1, bias); 
        }

        currentCurveDir  = candidateDir;
        currentCurveBias = bias;
        setLastData(filteredData);

        return (currentCurveDir != Direction::NEUTRAL);
    }
};
#endif //_CurveDetector_h