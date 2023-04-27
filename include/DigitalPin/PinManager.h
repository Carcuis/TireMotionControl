//
// Created by cui on 2023/4/8.
//

#ifndef TIRE_MOTION_CONTROL_PIN_MANAGER_H
#define TIRE_MOTION_CONTROL_PIN_MANAGER_H

#include "DigitalPin.h"

class PinManager
{
public:
    PinManager();

public:
    DigitalPin led = DigitalPin(LED_BUILTIN);

};

#endif //TIRE_MOTION_CONTROL_PIN_MANAGER_H
