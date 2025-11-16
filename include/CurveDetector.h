#ifndef _CurveDetector_h
#define _CurveDetector_h

#include <Arduino.h>

#ifndef MotionSensor_h
#include "MotionSensor.h"
#endif

class CurveDetector {
    private:
        uint32_t& now;  
        float neutralAngle;
        float lastServoAngle;
        float& currentServoAngle;

        Stream *logger;

        float getDirectionSign() {
            if (currentServoAngle == neutralAngle) return 0.0f;

            return currentServoAngle > neutralAngle ? -1.0f : 1.0f;
        }

    public:
        CurveDetector(Stream* l,  uint32_t& currentTimestamp, float& csa, float na)
            :  logger(l), now(currentTimestamp), currentServoAngle(csa), neutralAngle(na) {}

        void handle(MotionData& motionData) {
            static uint32_t lastLogMs = now;

            // if (now - lastLogMs >= 20) {
            //     logger->printf("%+.2f|%+.2f|%+.2f|%+.1f|%+.0f\n", 
            //         motionData.accel.rollDeg, motionData.gyroRoll, motionData.gyroYaw, currentServoAngle, getDirectionSign());
            //     }

            lastServoAngle = currentServoAngle;
        };
};
#endif //_CurveDetector_h