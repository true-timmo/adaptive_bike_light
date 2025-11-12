#ifndef _CurveDetector_h
#define _CurveDetector_h

#include <Arduino.h>

#ifndef MotionSensor_h
#include "MotionSensor.h"
#endif

class CurveDetector {
    private:
        Stream *logger;
        float neutralAngle;
        float lastServoAngle;
        float currentServoAngle;

        float getDirectionSign() {
            if (currentServoAngle == neutralAngle) return 0.0f;

            return currentServoAngle > neutralAngle ? -1.0f : 1.0f;
        }

    public:
        CurveDetector(Stream* l, float& csa, float na) :  logger(l), currentServoAngle(csa), neutralAngle(na) {}

        void handle(MotionData& motionData) {

            logger->printf("%+.2f|%+.2f|%+.2f|%+.1f|%+f\n", 
                motionData.accel.rollDeg, motionData.gyroRoll, motionData.gyroYaw, currentServoAngle, getDirectionSign());

            lastServoAngle = currentServoAngle;
        };
};
#endif //_CurveDetector_h