//
// Created by cui on 2023/4/8.
//

#ifndef ARDUINO_TEST_SPINTEST_H
#define ARDUINO_TEST_SPINTEST_H

#include <Arduino.h>
#include "../lib/AccelStepper/AccelStepper.h"

class SpinTest
{
public:
    SpinTest();

    void init();
    void run();

private:
    const byte stepPinH = 10;
    const byte dirPinH = 11;
    const long stepPerRound = 800;        // 电机每转一圈所需的脉冲数

    AccelStepper stepperH = AccelStepper(AccelStepper::DRIVER, stepPinH, dirPinH);
};

#endif //ARDUINO_TEST_SPINTEST_H
