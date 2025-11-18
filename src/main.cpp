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
  SET_OFFSET
};

static CMD resolveCMD(String cmd) {
  if (cmd == F("r")) return CMD::RIGHT;
  if (cmd == F("l")) return CMD::LEFT;
  if (cmd == F("n")) return CMD::NEUTRAL;
  if (cmd == F("c")) return CMD::CALIBRATE;
  if (cmd == F("s")) return CMD::TOGGLE_SERVO;
  if (cmd == F("b")) return CMD::TOGGLE_BOOST;
  if (cmd == F("log")) return CMD::TOGGLE_LOGS;
  if (cmd == F("cfg")) return CMD::DUMP_CFG;
  if (cmd == F("sos")) return CMD::SET_OFFSET;

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
      logger.printf("Toggle curve boost: %s\n", (ride.toggleCurveBoostState()) ? F("ON") : F("OFF"));
      break;
    case CMD::SET_OFFSET:
      config.gearOffset = value.toInt();
      eeprom.save(config);
      ride.setGearOffset(config.gearOffset);
      logger.printf("Mechanical gear offset set to: %d\n", config.gearOffset);
      break;
    case CMD::DUMP_CFG:
      logger.printf("CONFIG: offset=%.2f yaw=%.3f logging=%d servo=%d gain=%.3f gearOffset=%d\n",
                        config.rollDegOffset, config.yawBias, (int)config.logging, (int)config.servo,
                        config.gain, config.gearOffset);
      break;
      default:
      logger.println(F("COMMANDS: l=left, r=right, n=neutral, c=calibrate, b=toggle boost, log=toggle logs, cfg=dump config"));
      break;
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  eeprom.begin(64);
  delay(100);
  
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
