#ifndef _MotionFilter_h
#define _MotionFilter_h

#include <Arduino.h>

#ifndef MotionSensor_h
#include "MotionSensor.h"
#endif

class MotionFilter {
    private:
        static constexpr float NOISE_DIFF_ROLL  = 0.28f;
        static constexpr float NOISE_DIFF_YAW   = 0.23f;

        static constexpr float IQR_DIFF_ROLL = 1.82f;
        static constexpr float IQR_CAP_YAW  = 12.7f;

        static constexpr float SHOCK_CAP_ROLL  = 18.6f;
        static constexpr float SHOCK_CAP_YAW   = 42.9f;
        static constexpr float SHOCK_DIFF_ROLL  = 7.6f;
        static constexpr float SHOCK_DIFF_YAW   = 5.3f;

        MotionData lastMotionData;

        uint32_t& now;
        float&    dtRef;
        Stream*   logger;

        void handleNoise(MotionData& motionData) {
            const float absgyroYaw = fabsf(motionData.gyroYaw);
            const float absRollDeg = fabsf(motionData.accel.rollDeg);
            const float absRollDiff = fabsf(motionData.accel.rollDeg - lastMotionData.accel.rollDeg);
            const float absYawDiff = fabsf(motionData.gyroYaw - lastMotionData.gyroYaw);

            if (absRollDiff < NOISE_DIFF_ROLL) {
                motionData.accel = lastMotionData.accel;
            }

            if (absYawDiff < NOISE_DIFF_YAW) {
                motionData.gyroYaw = lastMotionData.gyroYaw;
                motionData.gyroRoll = lastMotionData.gyroRoll;
            }

            // if (absRollDiff > IQR_DIFF_ROLL) {
            //     motionData.accel.rollDeg = (motionData.accel.rollDeg > lastMotionData.accel.rollDeg)
            //         ? lastMotionData.accel.rollDeg + IQR_DIFF_ROLL 
            //         : lastMotionData.accel.rollDeg - IQR_DIFF_ROLL;
            // }

            // if (absgyroYaw > IQR_CAP_YAW) {
            //     motionData.gyroYaw = (motionData.gyroYaw > 0) ? IQR_CAP_YAW : IQR_CAP_YAW * -1.0f;
            // }
        }

        bool handleShock(MotionData &motionData) {
            const bool shockByCap = motionData.gyroYaw > SHOCK_CAP_YAW || motionData.accel.rollDeg > SHOCK_CAP_ROLL;
            const float absRollDiff = fabsf(motionData.accel.rollDeg - lastMotionData.accel.rollDeg);
            const float absYawDiff = fabsf(motionData.gyroYaw - lastMotionData.gyroYaw);

            if (absRollDiff > SHOCK_DIFF_ROLL || absYawDiff > SHOCK_DIFF_YAW) {
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
            lastMotionData = motionData;
        };
};
#endif // _MotionFilter_h