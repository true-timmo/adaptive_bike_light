#pragma once
#include <Arduino.h>

enum ButtonEvent {
  BUTTON_NONE,
  BUTTON_SHORT,
  BUTTON_LONG
};

class Button {
    private:
        static constexpr int LONG_PRESS_TIME = 2000;
        static constexpr int DEBOUNCE_TIME = 50;

        ButtonEvent event;

        uint8_t pin;
        uint8_t activeState;
        bool lastStableState = HIGH;
        unsigned long lastChange = 0;
        unsigned long pressStart = 0;
        bool longPressHandled = false;

        bool getDebouncedState(bool raw, unsigned long now) {
            if (raw != lastStableState && (now - lastChange) > DEBOUNCE_TIME) {
                lastStableState = raw;
                lastChange = now;
            }
            return lastStableState;
        }

        inline bool isPressedRaw(int level) const {
            return level == activeState;
        }

    public:
        Button(uint8_t gpioPin, uint8_t activeLevel = LOW) : pin(gpioPin), activeState(activeLevel) {
            pinMode(pin, (activeState == LOW) ? INPUT_PULLUP : INPUT_PULLDOWN);
        }

        void checkEvent() {            
            unsigned long now = millis();
            bool raw = digitalRead(pin);
            bool state = getDebouncedState(raw, now);
            ButtonEvent event = BUTTON_NONE;

            if (state == activeState) {
                if (pressStart == 0) {
                    pressStart = now;
                    longPressHandled = false;
                } else if (!longPressHandled && (now - pressStart >= LONG_PRESS_TIME)) {
                    this->event = BUTTON_LONG;
                    longPressHandled = true;
                }
            } else {
                if (pressStart != 0 && (now - pressStart < LONG_PRESS_TIME)) {
                    this->event = BUTTON_SHORT;
                }
                pressStart = 0;
            }
        };

        inline uint8_t getPin() const {
            return pin;
        }

        inline uint8_t getActiveLevel() const {
            return activeState;
        }

        bool isShortPressed () {
            if (event != BUTTON_SHORT) return false;

            while (digitalRead(getPin()) == LOW) delay(5);
            delay(30);

            return true;
        }

        bool isLongPressed() {
            return (event == BUTTON_LONG);
        }
};