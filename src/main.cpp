#include <ESP32Servo.h>
#include "esp_sleep.h"
#include "RideController.h"
#include "MotionSensor.h"
#include "ConfigurationStorage.h"
#include "BTSerial.h"
#include "Button.h"

#define I2C_SDA 9
#define I2C_SCL 10
#define BUTTON_PIN D3
#define BATTERY_PIN D0
#define BT_NAME "Dynamic BeamAssist"

Servo g_servo;
MotionSensor sensor = MotionSensor(12345);
BTSerial logger = BTSerial();
ConfigurationStorage eeprom = ConfigurationStorage(&logger);
Button button = Button(BUTTON_PIN, LOW);
RideController ride = RideController(&sensor, &g_servo, &logger);
ConfigBlob config;

enum class CMD {
  LEFT,
  RIGHT,
  NEUTRAL,
  DUMP_CFG,
  CALIBRATE,
  HELP,
  TOGGLE_SERVO,
  TOGGLE_LOGS,
  TOGGLE_BOOST,
  SET_OFFSET,
  SET_RATIO,
  BATTERY
};

float lookupBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float v_adc = raw * (3.3 / 4095.0);
  float v_batt = v_adc * 2.0;

  return v_batt;
}

String lookupBatteryStatus() {
  float v_batt = lookupBatteryVoltage();

  if (v_batt >= 4.41) return "charging";
  if (v_batt >= 4.20) return "100%";
  if (v_batt >= 4.08) return "80%";
  if (v_batt >= 3.98) return "60%";
  if (v_batt >= 3.92) return "50%";
  if (v_batt >= 3.82) return "30%";
  if (v_batt >= 3.73) return "10%";
  if (v_batt >= 3.50) return "5%";

  return "0%";
}

static CMD resolveCMD(String cmd) {
  if (cmd == F("r")) return CMD::RIGHT;
  if (cmd == F("l")) return CMD::LEFT;
  if (cmd == F("n")) return CMD::NEUTRAL;
  if (cmd == F("c")) return CMD::CALIBRATE;
  if (cmd == F("s")) return CMD::TOGGLE_SERVO;
  if (cmd == F("b")) return CMD::TOGGLE_BOOST;
  if (cmd == F("log")) return CMD::TOGGLE_LOGS;
  if (cmd == F("cfg")) return CMD::DUMP_CFG;
  if (cmd == F("so")) return CMD::SET_OFFSET;
  if (cmd == F("sr")) return CMD::SET_RATIO;
  if (cmd == F("v")) return CMD::BATTERY;

  return CMD::HELP;
}

void splitCommand(String input, CMD &cmd, String &value) {
    int spacePos = input.indexOf(' ');

    if (spacePos == -1) {
        cmd = resolveCMD(input);
        value = "";
    } else {
        cmd   = resolveCMD(input.substring(0, spacePos));
        value = input.substring(spacePos + 1);
        value.trim();
    }
}

static void shutdownPeripherals() {
  ride.turnNeutral();
  delay(100);
  ride.setServoState(false);
  sensor.sleep(true);
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
  splitCommand(input, cmd, value);

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

void setup() {
  Serial.begin(115200);
  eeprom.begin(64);
  delay(100);
  
  analogReadResolution(12);  // 0–4095
  analogSetAttenuation(ADC_11db); // bis ca. 3.3V

  logger.begin(BT_NAME);
  sensor.init(I2C_SDA, I2C_SCL);

  config = eeprom.load();
  ride.setLoggingState(config.logging);
  ride.setServoState(config.servo);
  ride.setGearOffset(config.gearOffset);
  ride.runCalibration();

  logger.println("Dynamic Beam Assist ready!");
}

void loop() {
  if (logger.available()) {
    String serialCmd = logger.readStringUntil('\n');
    if (handleSerialCMD(serialCmd)) return;
  }

  ButtonEvent ev = button.checkEvent();
  handleLoggingOnLongPress(ev);
  handleSleepOnShortPress(ev);

  ride.setTiming();

  MotionData motionData = sensor.readMotionData();
  if (!motionData.valid) {
    ride.turnNeutral();
    return;
  }

  ride.handleCurve(motionData);
}
