#include <ESP32Servo.h>
#include "esp_sleep.h"
#include "RideController.h"
#include "MotionSensor.h"
#include "CalibrationStorage.h"
#include "Button.h"
#include "MotionLogger.h"

#define I2C_SDA 8
#define I2C_SCL 20
#define BUTTON_PIN D1

Servo g_servo;
MotionSensor sensor = MotionSensor(12345);
CalibrationStorage eeprom = CalibrationStorage();
Button button = Button(BUTTON_PIN, LOW);
RideController ride = RideController(&sensor, &g_servo);
MotionLogger logger = MotionLogger();

static void shutdownPeripherals() {
  g_servo.detach();
}

static void printWakeCause() {
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.printf("Wake cause: %d\n", (int)cause);
}

void handleSleepOnShortPress(ButtonEvent ev) {
  if (ev != BUTTON_SHORT) return;

  while (digitalRead(BUTTON_PIN) == LOW) delay(5);
  delay(30);

  shutdownPeripherals();

  const esp_deepsleep_gpio_wake_up_mode_t wakeLevel =
      button.getActiveLevel() ? ESP_GPIO_WAKEUP_GPIO_HIGH : ESP_GPIO_WAKEUP_GPIO_LOW;

  esp_deep_sleep_enable_gpio_wakeup(BIT(BUTTON_PIN), wakeLevel);

  delay(20);
  esp_deep_sleep_start();
}

void handleCalibrationOnLongPress(ButtonEvent ev) {
  if (ev != BUTTON_LONG) return;

  MotionData calibrationData = ride.runCalibration();

  eeprom.saveCalibration(calibrationData.roll, calibrationData.yaw);
  Serial.println("Kalibrierung übernommen und gespeichert.");
}

void setup() {
  Serial.begin(115200);
  eeprom.begin(64);
  delay(100);
  
  logger.begin();
  sensor.init(I2C_SDA, I2C_SCL);
  //CalibBlob calib = eeprom.loadCalibration();

  MotionData calibrationData = ride.runCalibration();
  ride.init(calibrationData);

  Serial.println("Starte Kurvenlicht-Regelung...");
}

void loop() {
  logger.handleSerialMenu();

  ButtonEvent ev = button.checkEvent();
  handleCalibrationOnLongPress(ev);
  handleSleepOnShortPress(ev);

  ride.setTiming();

  MotionData motionData = sensor.readMotionData();
  if (!motionData.valid) {
    ride.turnNeutral();
    return;
  }

  //logger.logMotion(rollDeg);
  ride.handleCurve(motionData);
}
