//
// Created by cui on 2023/4/10.
//

#ifndef TIRE_MOTION_CONTROL_SERIAL_INPUT_H
#define TIRE_MOTION_CONTROL_SERIAL_INPUT_H

#include <Arduino.h>
#include "../lib/FlexiTimer2/FlexiTimer2.h"

class SerialInput
{
public:
    SerialInput();

    void onScan();

    void clear();

public:
    String input_message = "";

};

#endif //TIRE_MOTION_CONTROL_SERIAL_INPUT_H
