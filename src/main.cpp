#include <ESP32Servo.h>
#include "esp_sleep.h"
#include "RideController.h"
#include "MotionSensor.h"
#include "ConfigurationStorage.h"
#include "BTSerial.h"
#include "BTTerminal.h"
#include "Button.h"

#define I2C_SDA 9
#define I2C_SCL 10
#define BUTTON_PIN D3
#define BATTERY_PIN D0
#define BT_NAME "Dynamic BeamAssist"

Servo g_servo;
MotionSensor sensor = MotionSensor(12345);
BTSerial logger = BTSerial();
BTTerminal terminal = BTTerminal();
ConfigurationStorage eeprom = ConfigurationStorage(&logger);
Button button = Button(BUTTON_PIN, LOW);
RideController ride = RideController(&sensor, &g_servo, &logger);
ConfigBlob config;
bool sleepPending = false;

float lookupBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float v_adc = raw * (3.3 / 4095.0);
  const float TEILER = 1.805;

  return v_adc * TEILER;
}

String lookupBatteryStatus() {
  float v_batt = lookupBatteryVoltage();

  if (v_batt >= 4.15) return "100%";
  if (v_batt >= 4.10) return "90%";
  if (v_batt >= 4.05) return "80%";
  if (v_batt >= 4.00) return "70%";
  if (v_batt >= 3.95) return "60%";
  if (v_batt >= 3.90) return "50%";
  if (v_batt >= 3.85) return "40%";
  if (v_batt >= 3.80) return "30%";
  if (v_batt >= 3.75) return "20%";
  if (v_batt >= 3.70) return "10%";
  if (v_batt >= 3.55) return "5%";

  return "0%";
}

static void printWakeCause() {
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  logger.printf("Wake cause: %d\n", (int)cause);
}

void handleSleepOnShortPress(ButtonEvent ev) {
  if (ev != BUTTON_SHORT) return;

  while (digitalRead(BUTTON_PIN) == LOW) delay(5);
  delay(30);

  ride.turnNeutral();
  sleepPending = true;
}

void handleLoggingOnLongPress(ButtonEvent ev) {
  if (ev != BUTTON_LONG) return;

  config = eeprom.load();
  config.logging = !config.logging;
  eeprom.save(config);
  ride.setLoggingState(config.logging);

  String sMode = (config.logging) ? "ON" : "OFF";

  logger.printf("Toggle logging mode: %s\n", sMode);
}

bool handleSerialCMD(String input) {
  input.trim();

  if (input.isEmpty()) return false;

  CMD cmd; String value;
  terminal.splitCommand(input, cmd, value);

  switch (cmd) {
    case CMD::LEFT:
      ride.turnLeft();
      break;
    case CMD::RIGHT:
      ride.turnRight();
      break;
    case CMD::NEUTRAL:
      ride.turnNeutral();
      break;
    case CMD::CALIBRATE:
      ride.runCalibration();
      break;
    case CMD::BATTERY:
      logger.printf("Battery status: %s, Voltage: %.2fV\n", lookupBatteryStatus(), lookupBatteryVoltage());
      break;
    case CMD::TOGGLE_SERVO:
      config.servo = !config.servo;
      eeprom.save(config);
      ride.setServoState(config.servo);
      break;
    case CMD::TOGGLE_LOGS:
      config.logging= !config.logging;
      eeprom.save(config);
      ride.setLoggingState(config.logging);
      break;
    case CMD::TOGGLE_BOOST:
      config.curveBoost= !config.curveBoost;
      eeprom.save(config);
      ride.setCurveBoostState(config.curveBoost);
      logger.printf("Toggle curve boost: %s\n", (config.curveBoost) ? F("ON") : F("OFF"));
      break;
    case CMD::SET_OFFSET:
      config.gearOffset = value.toInt();
      eeprom.save(config);
      ride.setGearOffset(config.gearOffset);
      logger.printf("Mechanical gear offset set to: %d\n", config.gearOffset);
      break;
    case CMD::SET_RATIO:
      config.gearRatio = value.toFloat();
      eeprom.save(config);
      ride.setGearRatio(config.gearRatio);
      logger.printf("Mechanical gear ratio set to: %d\n", config.gearRatio);
      break;
    case CMD::DUMP_CFG:
      eeprom.dump(config);
      break;
    default:
      logger.println(F("COMMANDS:"));
      logger.println(F("  l=left, r=right, n=neutral\n  c=calibrate, b=toggle boost\n  log=toggle logs, cfg=dump config\n  so=set offset, sr=set ratio\n  v=battery voltage"));
      break;
  }

  return true;
}

void goSleep() {
  delay(50);
  ride.setServoState(false);
  sensor.sleep(true);
  sleepPending = false;

  const esp_deepsleep_gpio_wake_up_mode_t wakeLevel =
      button.getActiveLevel() ? ESP_GPIO_WAKEUP_GPIO_HIGH : ESP_GPIO_WAKEUP_GPIO_LOW;

  esp_deep_sleep_enable_gpio_wakeup(BIT(BUTTON_PIN), wakeLevel);

  delay(20);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  eeprom.begin(64);
  delay(100);
  
  analogReadResolution(12);  // 0–4095
  analogSetAttenuation(ADC_11db); // bis ca. 3.3V

  logger.begin(BT_NAME);
  if (sensor.init(I2C_SDA, I2C_SCL)) {
    config = eeprom.load();
    ride.setLoggingState(config.logging);
    ride.setServoState(config.servo);
    ride.setGearOffset(config.gearOffset);
    ride.runCalibration();

    logger.println("Dynamic Beam Assist ready!");
  } else {
    logger.println("Motion Sensor failure!");
  }
}

void loop() {
  if (logger.available()) {
    String serialCmd = logger.readStringUntil('\n');
    if (handleSerialCMD(serialCmd)) return;
  }

  ButtonEvent ev = button.checkEvent();
  handleLoggingOnLongPress(ev);
  handleSleepOnShortPress(ev);

  ride.syncTiming();
  if (ride.handleStrictServoAngle() == true) {
    return;
  }

  if (sleepPending == true) {
    goSleep();
    return;
  }

  MotionData motionData = sensor.readMotionData();
  if (!motionData.valid) {
    ride.turnNeutral();
    return;
  }

  ride.handleCurve(motionData);
}
