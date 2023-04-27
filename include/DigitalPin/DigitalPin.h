//
// Created by cui on 2023/4/8.
//

#ifndef TIRE_MOTION_CONTROL_DIGITAL_PIN_H
#define TIRE_MOTION_CONTROL_DIGITAL_PIN_H

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

#endif //TIRE_MOTION_CONTROL_DIGITAL_PIN_H
