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

        static constexpr float IQR_DIFF_ROLL = 3.6f;
        static constexpr float IQR_CAP_YAW  = 30.0f;

        static constexpr float SHOCK_CAP_ROLL  = 18.6f;
        static constexpr float SHOCK_CAP_YAW   = 42.9f;
        static constexpr float SHOCK_DIFF_ROLL  = 7.6f;
        static constexpr float SHOCK_DIFF_YAW   = 5.3f;

        uint32_t& now;
        float&    dtRef;

        float absRollDiff = 0.0f;
        float absYawDiff = 0.0f;

        Stream*   logger;
        MotionData lastMotionData{};
        bool hasLast = false;

        bool handleNoise(MotionData& motionData) {
            if (absRollDiff < NOISE_DIFF_ROLL) {
                motionData.accel = lastMotionData.accel;
            } else {
                if (absRollDiff > IQR_DIFF_ROLL) {
                    motionData.accel.rollDeg = lastMotionData.accel.rollDeg 
                            + constrain(motionData.accel.rollDeg - lastMotionData.accel.rollDeg,
                                        -IQR_DIFF_ROLL, IQR_DIFF_ROLL);
                }
            }

            if (absYawDiff < NOISE_DIFF_YAW) {
                motionData.gyroYaw = lastMotionData.gyroYaw;
                motionData.gyroRoll = lastMotionData.gyroRoll;
            }

            if (fabsf(motionData.gyroYaw) > IQR_CAP_YAW) {
                motionData.gyroYaw = copysign(IQR_CAP_YAW, motionData.gyroYaw);
            }
        }

        bool handleShock(MotionData &motionData) {
            const bool shockByCap = motionData.gyroYaw > SHOCK_CAP_YAW || motionData.accel.rollDeg > SHOCK_CAP_ROLL;

            if (shockByCap || absRollDiff > SHOCK_DIFF_ROLL || absYawDiff > SHOCK_DIFF_YAW) {
                return true;
            }

            return false;
        };

    public:
        MotionFilter(Stream* l, uint32_t& currentTimestamp, float& lastToCurrent)
        : now(currentTimestamp), dtRef(lastToCurrent), logger(l) {}

        bool handle(MotionData& motionData) {
            if (!motionData.valid) return false;
            
            if (!hasLast) {
                lastMotionData = motionData;
                hasLast = true;
                return false;
            }

            const bool hasShock = handleShock(motionData);
            if (hasShock) return false;

            absRollDiff = fabsf(motionData.accel.rollDeg - lastMotionData.accel.rollDeg);
            absYawDiff = fabsf(motionData.gyroYaw - lastMotionData.gyroYaw);
            handleNoise(motionData);

            lastMotionData = motionData;

            return true;
        };
};
#endif // _MotionFilter_h