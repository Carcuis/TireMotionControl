//
// Created by cui on 2023/4/8.
//

#include "DigitalPin/DigitalPin.h"

DigitalPin::DigitalPin(int pin_num)
{
    this->pin_num = pin_num;
    pinMode(pin_num, OUTPUT);
    this->off();
}

void DigitalPin::on()
{
    digitalWrite(pin_num, HIGH);
    this->state = PinState::ON;
}

void DigitalPin::off()
{
    digitalWrite(pin_num, LOW);
    this->state = PinState::OFF;
}

PinState DigitalPin::getState()
{
    return this->state;
}
