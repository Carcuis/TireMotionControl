//
// Created by cui on 2023/4/8.
//

#ifndef ARDUINO_TEST_DIGITALPIN_H
#define ARDUINO_TEST_DIGITALPIN_H

#include <Arduino.h>

enum PinState
{
    ON,
    OFF
};

class DigitalPin
{
public:
    explicit DigitalPin(int pin_num);
    void on();
    void off();
    PinState getState();

private:
    int pin_num;
    PinState state = PinState::OFF;
};

#endif //ARDUINO_TEST_DIGITALPIN_H
