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
    float rollDeg;
    bool valid = false;

    Accel() {}
    Accel(float x_, float y_, float z_) : x(x_), y(y_), z(z_) { valid = true; }
    Accel(float x_, float y_, float z_, float _roll) : x(x_), y(y_), z(z_), rollDeg(_roll) { valid = true; }
};

struct MotionData {
    float gyroRoll = 0.0f;
    float gyroYaw = 0.0f;
    Accel accel = Accel();
    bool valid = false;

    MotionData() {}
    MotionData(float roll_, float yaw_) : gyroRoll(roll_), gyroYaw(yaw_) { valid = true; }
    MotionData(float roll_, float yaw_, Accel accel_) : gyroRoll(roll_), gyroYaw(yaw_), accel(accel_) { valid = true; }
};

class MotionSensor {
  private:
    static constexpr float G_MPS2 = 9.80665f;
    static constexpr float GYRO_DEADZONE_YAW = 0.6f;
    static constexpr float GYRO_DEADZONE_ROLL = 0.31f;
    static constexpr float ACCEL_DEADZONE_ROLL = 0.05f;

    static constexpr float SIGN_X = 1.0f;
    static constexpr float SIGN_Y = 1.0f;
    static constexpr float SIGN_Z = 1.0f;

    float xOffset = 0.0f;
    float yOffset = 0.0f;
    float zOffset = 0.0f;

    float xBias = 0.0f;
    float zBias = 0.0f;
    Adafruit_MPU6050 g_sensor;

    inline float applyDeadzone(float v, float deadzone) {
        return (fabs(v) < deadzone) ? 0.0f : v - copysign(deadzone, v);
    }

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

    MotionData calibrateGyro(uint16_t samples = 200, uint16_t delayMs = 5) {
        float sumX = 0.0f;
        float sumZ = 0.0f;
        uint16_t used = 0;

        for (uint16_t i = 0; i < samples; i++) {
            sensors_event_t a, g, t;
            g_sensor.getEvent(&a, &g, &t);

            float gx = g.gyro.x * 180.0f / M_PI * SIGN_X;
            float gz = g.gyro.z * 180.0f / M_PI * SIGN_Z;

            if (fabsf(gx) > 2.0f || fabsf(gz) > 2.0f) {
                delay(delayMs);
                continue;
            }

            sumX += gx;
            sumZ += gz;
            used++;
            delay(delayMs);
        }
        if (used > 0) {
            xBias = sumX / used;
            zBias  = sumZ  / used;
        }

        return MotionData(zBias, xBias);
    }

    Accel calibrateAccel(uint32_t duration_ms = 2000) {
        uint32_t t0 = millis();
        uint32_t n  = 0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;

        while (millis() - t0 < duration_ms) {
            sensors_event_t a, g, t;
            g_sensor.getEvent(&a, &g, &t);

            float ax = a.acceleration.x;
            float ay = a.acceleration.y;
            float az = a.acceleration.z;

            float aNorm = sqrtf(ax*ax + ay*ay + az*az);
            if (!isfinite(ax) || fabsf(aNorm - G_MPS2) > 0.8f) { 
                delay(5);
                continue;
            }
            
            sumX += ax;
            sumY += ay; 
            sumZ += az; 
            n++;
        }
        xOffset = (n > 0) ? (float)(sumX / (double)n) : 0.0f;
        yOffset = (n > 0) ? (float)(sumY / (double)n) : 0.0f;
        zOffset = (n > 0) ? (float)(sumZ / (double)n) : 0.0f;

        return Accel(xOffset, yOffset, zOffset);
    };

    MotionData readMotionData() {
        sensors_event_t a, g, t;
        g_sensor.getEvent(&a, &g, &t);

        float ax = a.acceleration.x - xOffset;
        float ay = a.acceleration.y - yOffset;
        float az = a.acceleration.z - zOffset + G_MPS2;
        float gYaw = g.gyro.x * 180.0f / M_PI - xBias;
        float gRoll = g.gyro.z * 180.0f / M_PI - zBias;

        if (!isfinite(ax) || !isfinite(ay) || !isfinite(az) || !isfinite(gRoll) || !isfinite(gYaw))
            return MotionData();   

        float accRollDeg = atan2f(SIGN_X * ay, az) * 180.0f / M_PI;

        accRollDeg = applyDeadzone(accRollDeg, ACCEL_DEADZONE_ROLL);
        gYaw = applyDeadzone(gYaw, GYRO_DEADZONE_YAW);
        gRoll = applyDeadzone(gRoll, GYRO_DEADZONE_ROLL); 

        return MotionData(gRoll, gYaw, Accel(ax, ay, az, accRollDeg));
    }
};
#endif // MotionSensor_h
