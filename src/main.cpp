#include <ESP32Servo.h>
#include "esp_sleep.h"
#include "RideController.h"
#include "MotionSensor.h"
#include "ConfigurationStorage.h"
#include "BTSerial.h"
#include "Button.h"

#define I2C_SDA 8
#define I2C_SCL 20
#define BUTTON_PIN D1
#define BT_NAME "Dynamic BeamAssist"

Servo g_servo;
MotionSensor sensor = MotionSensor(12345);
BTSerial logger = BTSerial();
ConfigurationStorage eeprom = ConfigurationStorage(&logger);
Button button = Button(BUTTON_PIN, LOW);
RideController ride = RideController(&sensor, &g_servo, &logger);
ConfigBlob config;

static void shutdownPeripherals() {
  g_servo.detach();
}

static void printWakeCause() {
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  logger.printf("Wake cause: %d\n", (int)cause);
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

void handleDevModeOnLongPress(ButtonEvent ev) {
  if (ev != BUTTON_LONG) return;

  config = eeprom.loadCalibration();
  config.devModeEnabled = !config.devModeEnabled;
  eeprom.saveCalibration(config);

  String sMode = (config.devModeEnabled) ? "ON" : "OFF";

  logger.printf("Toggle dev mode: %s\n", sMode);
}

void handleSerialCMD(String cmd) {
  cmd.trim();

  if (cmd.isEmpty()) return;
  if (cmd == F("help")) logger.println(F("ping set config")); return;

  if (cmd == F("r")) ride.turnRight(); return;
  if (cmd == F("l")) ride.turnLeft(); return;
  if (cmd == F("n")) ride.turnNeutral(); return;
  if (cmd == F("c")) ride.runCalibration(); return;

  if (cmd == F("cfg")) {
    logger.printf("CONFIG: offset=%.2f yaw=%.3f devMode=%d gain=%.3f gearOffset=%.3f leanEnter=%.3f leanExit=%.3f\n",
                        config.rollDegOffset, config.yawBias, (int)config.devModeEnabled, 
                        config.gain, config.gearOffset, config.leanEnterDeg, config.leanExitDeg);
  } 
}

void setup() {
  Serial.begin(115200);
  eeprom.begin(64);
  delay(100);
  
  sensor.init(I2C_SDA, I2C_SCL);
  config = eeprom.loadCalibration();

  if (config.devModeEnabled) {
    logger.begin(BT_NAME);
  }
  ride.init();

  Serial.println("Starte Kurvenlicht-Regelung...");
}

void loop() {
  // Eingabe testen
  if (logger.available()) {
    String cmd = logger.readStringUntil('\n');
    handleSerialCMD(cmd);
  }

  ButtonEvent ev = button.checkEvent();
  handleDevModeOnLongPress(ev);
  handleSleepOnShortPress(ev);

  ride.setTiming();

  MotionData motionData = sensor.readMotionData();
  if (!motionData.valid) {
    ride.turnNeutral();
    return;
  }

  ride.handleCurve(motionData);
}
