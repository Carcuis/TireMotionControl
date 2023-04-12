//
// Created by cui on 2023/4/8.
//

#ifndef ARDUINO_TEST_PINMANAGER_H
#define ARDUINO_TEST_PINMANAGER_H

#include "DigitalPin.h"

class PinManager
{
public:
    PinManager();

public:
    DigitalPin led = DigitalPin(LED_BUILTIN);

};

#endif //ARDUINO_TEST_PINMANAGER_H
