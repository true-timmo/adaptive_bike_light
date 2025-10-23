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

    Accel() {}
    Accel(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

class MotionSensor
{
    private:
        static constexpr range_t ACC_RANGE = ADXL345_RANGE_4_G;
        Adafruit_ADXL345_Unified g_accel;

    public:
        MotionSensor(int32_t id=-1) : g_accel(id) {}

        void init(int sdaPin, int sclPin) {
            Wire.begin(sdaPin, sclPin);
            if (!g_accel.begin()) {
                Serial.println("ADXL345 nicht gefunden! Verdrahtung prüfen.");
                while (true) delay(1000);
            }
            g_accel.setRange(ACC_RANGE);
        };

        bool readAccel(Accel *accel) {
            sensors_event_t event;
            g_accel.getEvent(&event);

            if (!isfinite(event.acceleration.x)
                || !isfinite(event.acceleration.y)
                || !isfinite(event.acceleration.z)
            ) { return false; }

            accel->x = event.acceleration.x,
            accel->y = event.acceleration.y,
            accel->z = event.acceleration.z;

            return true;
        };
};
#endif //MotionSensor_h