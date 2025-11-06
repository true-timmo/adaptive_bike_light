#ifndef MotionSensor_h
#define MotionSensor_h

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <math.h>
#include <EEPROM.h>

struct Accel {
    float x;
    float y;
    float z;
    float roll;
    bool valid = false;

    Accel() {}
    Accel(float x_, float y_, float z_) : x(x_), y(y_), z(z_) { valid = true; }
    Accel(float x_, float y_, float z_, float _roll) : x(x_), y(y_), z(z_), roll(_roll) { valid = true; }
};

struct MotionData {
    float roll = 0.0f;
    float yaw = 0.0f;
    Accel accel = Accel();
    bool valid = false;

    MotionData() {}
    MotionData(float roll_, float yaw_) : roll(roll_), yaw(yaw_) { valid = true; }
    MotionData(float roll_, float yaw_, Accel accel_) : roll(roll_), yaw(yaw_), accel(accel_) { valid = true; }
};

class MotionSensor {
  private:
    static constexpr float ROLL_SIGN = 1.0f;
    float gyroRoll = 0.0f;
    bool gyroRollReset = false;
    unsigned long tPrev = 0;
    float yawBias = 0.0f;
    float rollOffset = 0.0f;
    Adafruit_MPU6050 g_sensor;

  public:
    MotionSensor(int32_t id = -1) : g_sensor() {}

    void init(int sdaPin, int sclPin) {
        Wire.setPins(sdaPin, sclPin);
        Wire.setClock(400000);
        if (!g_sensor.begin()) {
            Serial.println("MPU6050 not found!");
            while (true) delay(1000);
        }
        g_sensor.setAccelerometerRange(MPU6050_RANGE_4_G);
        g_sensor.setGyroRange(MPU6050_RANGE_500_DEG);
        g_sensor.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    float calibrateGyroBias(uint16_t samples = 200, uint16_t delayMs = 5) {
        float sum = 0.0f;

        for (uint16_t i = 0; i < samples; i++) {
            sensors_event_t a, g, t;
            g_sensor.getEvent(&a, &g, &t);
            sum += g.gyro.z * 180.0f / M_PI;
            delay(delayMs);
        }
        yawBias = sum / samples;

        return yawBias;
    }

    float calibrateRollAngle(uint32_t duration_ms = 2000) {
        uint32_t t0 = millis();
        uint32_t n  = 0;
        double sum = 0.0;

        while (millis() - t0 < duration_ms) {
            MotionData data = readMotionData();
            if (data.valid) { sum += data.roll; n++; }
            delay(5);
        }
        rollOffset = (n > 0) ? (float)(sum / (double)n) : 0.0f;

        return rollOffset;
    };

    void resetGyroRoll() {
        gyroRollReset = false;
    }

    MotionData readMotionData() {
        sensors_event_t a, g, t;
        g_sensor.getEvent(&a, &g, &t);

        float ax = a.acceleration.x;
        float ay = a.acceleration.y;
        float az = a.acceleration.z;
        float roll = g.gyro.x * 180.0f / M_PI;
        float yaw = g.gyro.z * 180.0f / M_PI - yawBias;

        if (!isfinite(ax) || !isfinite(ay) || !isfinite(az) || !isfinite(roll) || !isfinite(yaw))
            return MotionData();

        unsigned long now = micros();
        float dt = (tPrev == 0) ? 0.01f : (now - tPrev) / 1e6f;
        tPrev = now;

        float accRoll = atan2f(ROLL_SIGN * ay, az) * 180.0f / M_PI - rollOffset;

        if (!gyroRollReset) {
            gyroRoll = accRoll;
            gyroRollReset = true;
        } else {
            gyroRoll += roll * dt;
        }

        return MotionData(gyroRoll, yaw, Accel(ax, ay, az, accRoll));
    }
};
#endif // MotionSensor_h
