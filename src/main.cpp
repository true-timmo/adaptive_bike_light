#include <ESP32Servo.h>
#include "esp_sleep.h"
#include "RideController.h"
#include "MotionSensor.h"
#include "ConfigurationStorage.h"
#include "BTSerial.h"
#include "BTTerminal.h"
#include "Button.h"
#include "PowerManager.h"

#define I2C_SDA 9
#define I2C_SCL 10
#define BUTTON_PIN D3
#define VUSB_PIN D0
#define VBAT_PIN D1
#define PWR_PIN D8
#define BT_NAME "Dynamic BeamAssist #2"

Servo g_servo;
MotionSensor sensor = MotionSensor(12345);
BTSerial logger = BTSerial();
BTTerminal terminal = BTTerminal();
ConfigurationStorage eeprom = ConfigurationStorage(&logger);
Button button = Button(BUTTON_PIN, LOW);
PowerManager power = PowerManager(VBAT_PIN, VUSB_PIN, PWR_PIN);
RideController ride = RideController(&sensor, &g_servo, &logger);
ConfigBlob config;
bool sleepPending = false;

static CalibBlob mapMotionDataToCalibBlob(MotionData motionData) {
  return CalibBlob(
    motionData.gyroRoll, motionData.gyroYaw,
    motionData.accel.x, motionData.accel.y, motionData.accel.z
  );
}

void handleSleepOnShortPress(ButtonEvent ev) {
  if (ev != BUTTON_SHORT) return;

  while (digitalRead(BUTTON_PIN) == LOW) delay(5);
  delay(30);

  ride.turnNeutral();
  sleepPending = true;
}

void switchBluetoothOnLongPress(ButtonEvent ev) {
  if (ev != BUTTON_LONG) return;

    bool btEnabled = config.bluetooth;
    if (btEnabled) {
      logger.stop();
      config.bluetooth = false;
    }
    else {
      logger.begin(BT_NAME);
      config.bluetooth = true;
    }

    eeprom.save(config);    
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
      config.calibData = mapMotionDataToCalibBlob(sensor.readCalibration());
      eeprom.save(config);
      break;
    case CMD::BATTERY:
      logger.printf("Battery: %d%% (%.2fV) | VUSB: %.2fV\n", power.readBatteryPercent(), power.readVBattery(), power.readVUSB());
      break;
    case CMD::TOGGLE_SERVO:
      config.servo = !config.servo;
      eeprom.save(config);
      power.enablePower(config.servo);
      logger.printf("Toggle servo: %s\n", (config.servo) ? F("ON") : F("OFF"));
      break;
    case CMD::TOGGLE_LOGS:
      config.logging= !config.logging;
      eeprom.save(config);
      ride.setLoggingState(config.logging);
      logger.printf("Toggle logging: %s\n", (config.logging) ? F("ON") : F("OFF"));
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
  ride.setServoActive(false);
  power.enablePower(false);
  sensor.sleep(true);
  delay(20);
  sleepPending = false;

  const esp_deepsleep_gpio_wake_up_mode_t wakeLevel =
      button.getActiveLevel() ? ESP_GPIO_WAKEUP_GPIO_HIGH : ESP_GPIO_WAKEUP_GPIO_LOW;

  esp_deep_sleep_enable_gpio_wakeup(BIT(BUTTON_PIN), wakeLevel);
  esp_deep_sleep_start();
}

void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  eeprom.begin(64);
  delay(30);
  config = eeprom.load();
  delay(50);

  if (config.bluetooth) {
    logger.begin(BT_NAME);
  }

  if (sensor.init(I2C_SDA, I2C_SCL)) {
    CalibBlob calibData = config.calibData;
    sensor.writeCalibration(MotionData(
      calibData.rollBias,
      calibData.yawBias,
      Accel(calibData.xOffset, calibData.yOffset, calibData.zOffset)
    ));
    power.enablePower(config.servo);
    ride.setLoggingState(config.logging);
    ride.setGearOffset(config.gearOffset);

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
  switchBluetoothOnLongPress(ev);
  handleSleepOnShortPress(ev);

  ride.init(power.isPowerEnabled());
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
