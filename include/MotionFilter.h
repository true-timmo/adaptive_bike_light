#ifndef _MotionFilter_h
#define _MotionFilter_h

#include <Arduino.h>

#ifndef MotionSensor_h
#include "MotionSensor.h"
#endif

class MotionFilter {
    private:
        static constexpr float NOISE_THR_ROLL  = 0.28f;
        static constexpr float NOISE_THR_YAW   = 0.23f;

        static constexpr float SHOCK_CAP_ROLL  = 18.6f;
        static constexpr float SHOCK_CAP_YAW   = 42.9f;
        static constexpr float SHOCK_THR_ROLL  = 7.6f;
        static constexpr float SHOCK_THR_YAW   = 5.3f;

        MotionData lastMotionData;
        float    prevYawRate   = 0.0f;

        uint32_t& now;
        float&    dtRef;
        Stream*   logger;

        void handleNoise(MotionData& motionData) {
            const float yawRate = motionData.gyroYaw;
            const float absRollDiff = fabsf(motionData.accel.rollDeg - lastMotionData.accel.rollDeg);
            const float absYawDiff = fabsf(motionData.gyroYaw - lastMotionData.gyroYaw);

            if (absRollDiff < NOISE_THR_ROLL) {
                motionData.accel = lastMotionData.accel;
            }

            if (absYawDiff < NOISE_THR_YAW) {
                motionData.gyroYaw = lastMotionData.gyroYaw;
                motionData.gyroRoll = lastMotionData.gyroRoll;
            }
        }

        bool handleShock(MotionData &motionData) {
            const bool shockByCap = motionData.gyroYaw > SHOCK_CAP_YAW || motionData.accel.rollDeg > SHOCK_CAP_ROLL;
            const float absRollDiff = fabsf(motionData.accel.rollDeg - lastMotionData.accel.rollDeg);
            const float absYawDiff = fabsf(motionData.gyroYaw - lastMotionData.gyroYaw);

            if (absRollDiff > SHOCK_THR_ROLL || absYawDiff > SHOCK_THR_YAW) {
                motionData.valid = false;
                return false;
            }

            return true;
        };

    public:
        MotionFilter(Stream* l, uint32_t& currentTimestamp, float& lastToCurrent)
        : now(currentTimestamp), dtRef(lastToCurrent), logger(l) {}

        void handle(MotionData& motionData) {
            if (!motionData.valid) return;
            if (!handleShock(motionData)) return;

            handleNoise(motionData);

            logger->printf("%.2f|%.2f|%.2f|%d\n", motionData.accel.rollDeg, motionData.gyroRoll, motionData.gyroYaw, motionData.valid);

            lastMotionData = motionData;
        };
};
#endif // _MotionFilter_h