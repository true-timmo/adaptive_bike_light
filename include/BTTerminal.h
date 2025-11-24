#ifndef _BTTerminal_h_
#define _BTTerminal_h_

#include <Arduino.h>

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

class BTTerminal {
public:
    BTTerminal() = default;

    CMD resolveCMD(String cmd) {
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
};

#endif // _BTTerminal_h_