#pragma once
#include <Arduino.h>
#include "MotionSensor.h"

struct FilteredData {
    float gyroRoll = 0.0f;
    float gyroYaw = 0.0f;
    float accelRollDeg = 0.0f;
    bool isShock = false;

    FilteredData() { isShock = true; }
    FilteredData(float _gr, float _gy, float _ard) : gyroRoll(_gr), gyroYaw(_gy), accelRollDeg(_ard) {}
};

class MotionFilter {
    private:
        static constexpr float NOISE_DIFF_ROLL  = 0.28f;
        static constexpr float NOISE_DIFF_YAW   = 0.23f;

        static constexpr float IQR_DIFF_ROLL = 3.6f;
        static constexpr float IQR_CAP_YAW  = 30.0f;

        static constexpr float SHOCK_CAP_ROLL  = 36.6f;
        static constexpr float SHOCK_CAP_YAW   = 42.9f;
        static constexpr float SHOCK_DIFF_ROLL  = 7.6f;
        static constexpr float SHOCK_DIFF_YAW   = 5.3f;

        Stream* logger;

        float lastGyroRoll = NAN;
        float lastGyroYaw = 0.0f;
        float lastAccelRollDeg = 0.0f;
        float absAccelRollDiff = 0.0f;
        float absGyroRollDiff = 0.0f;
        float absGyroYawDiff = 0.0f;

        void handleNoise(FilteredData *filteredData) {
            if (filteredData->isShock) return;

            if (absAccelRollDiff < NOISE_DIFF_ROLL) {
                filteredData->accelRollDeg = lastAccelRollDeg;
            } else {
                if (absAccelRollDiff > IQR_DIFF_ROLL) {
                    filteredData->accelRollDeg = lastAccelRollDeg
                            + constrain(filteredData->accelRollDeg - lastAccelRollDeg, -IQR_DIFF_ROLL, IQR_DIFF_ROLL);
                }
            }

            if (absGyroYawDiff < NOISE_DIFF_YAW) {
                filteredData->gyroYaw = lastGyroYaw;
                filteredData->gyroRoll = lastGyroRoll;
            }

            if (fabsf(filteredData->gyroYaw) > IQR_CAP_YAW) {
                filteredData->gyroYaw = copysign(IQR_CAP_YAW, filteredData->gyroYaw);
            }

            if (fabsf(filteredData->gyroRoll) > IQR_CAP_YAW) {
                filteredData->gyroRoll = copysign(IQR_CAP_YAW, filteredData->gyroRoll);
            }
        }

        void handleShock(FilteredData *filteredData) {
            bool shockByCap = abs(filteredData->gyroYaw) > SHOCK_CAP_YAW || abs(filteredData->gyroRoll) > SHOCK_CAP_ROLL;

            if (shockByCap || absAccelRollDiff > SHOCK_DIFF_ROLL || absGyroYawDiff > SHOCK_DIFF_YAW) {
                filteredData->isShock = true;
            }
        };

    public:
        MotionFilter(Stream* l) : logger(l) {}

        FilteredData handle(MotionData motionData) {
            if (!isfinite(lastGyroRoll)) {
                lastGyroRoll = motionData.gyroRoll;
                lastGyroYaw = motionData.gyroYaw;
                lastAccelRollDeg = motionData.accel.rollDeg;
            }

            FilteredData filteredData = FilteredData(motionData.gyroRoll, motionData.gyroYaw, motionData.accel.rollDeg);
            absAccelRollDiff = fabsf(filteredData.accelRollDeg - lastAccelRollDeg);
            absGyroYawDiff = fabsf(filteredData.gyroYaw - lastGyroYaw);
            absGyroRollDiff = fabsf(filteredData.gyroRoll - lastGyroRoll);

            handleShock(&filteredData);
            handleNoise(&filteredData);

            lastGyroRoll = filteredData.gyroRoll;
            lastGyroYaw = filteredData.gyroYaw;
            lastAccelRollDeg = filteredData.accelRollDeg;

            return filteredData;
        };
};