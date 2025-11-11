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

        MotionData lastMotionData;
        float    prevYawRate   = 0.0f;

        uint32_t& now;
        float&    dtRef;
        Stream*   logger;

    public:
        MotionFilter(Stream* l, uint32_t& currentTimestamp, float& lastToCurrent)
        : now(currentTimestamp), dtRef(lastToCurrent), logger(l) {}

        void filterNoise(MotionData& motionData) {
            if (!motionData.valid) return;

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

            lastMotionData = motionData;
        }
};
#endif // _MotionFilter_h