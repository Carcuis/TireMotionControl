//
// Created by cui on 2023/4/10.
//

#ifndef ARDUINO_TEST_SERIALINPUT_H
#define ARDUINO_TEST_SERIALINPUT_H

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

#endif //ARDUINO_TEST_SERIALINPUT_H
