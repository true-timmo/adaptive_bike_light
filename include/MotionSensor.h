#ifndef MotionSensor_h
#define MotionSensor_h

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <EEPROM.h>

struct Accel {
    float x;
    float y;
    float z;
    bool valid = false;

    Accel() {}
    Accel(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {valid = true;}
};

class MotionSensor
{
    private:
        static constexpr range_t ACC_RANGE = ADXL345_RANGE_4_G;
        static constexpr float ROLL_SIGN = -1.0f;

        Adafruit_ADXL345_Unified g_accel;

    public:
        MotionSensor(int32_t id=-1) : g_accel(id) {}

        void init(int sdaPin, int sclPin) {
            Wire.setPins(sdaPin, sclPin);
            if (!g_accel.begin()) {
                Serial.println("ADXL345 nicht gefunden! Verdrahtung prüfen.");
                while (true) delay(1000);
            }
            g_accel.setRange(ACC_RANGE);
        };

        Accel readAccel() {
            sensors_event_t event;
            g_accel.getEvent(&event);

            if (!isfinite(event.acceleration.x)
                || !isfinite(event.acceleration.y)
                || !isfinite(event.acceleration.z)
            ) { return Accel(); }

            return Accel(event.acceleration.x, event.acceleration.y, event.acceleration.z);
        };

        float readTiltAngle() {
            sensors_event_t event;
            g_accel.getEvent(&event);

            if (!isfinite(event.acceleration.x)
                || !isfinite(event.acceleration.y)
                || !isfinite(event.acceleration.z)
            ) { return NAN; }

            float rollRad = atan2f(event.acceleration.x, event.acceleration.z);
            float rollDeg = rollRad * 180.0f / PI;

            return ROLL_SIGN * rollDeg;
        }
};
#endif //MotionSensor_h