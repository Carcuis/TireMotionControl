//
// Created by cui on 2023/4/8.
//

#ifndef TIRE_MOTION_CONTROL_SPIN_TEST_H
#define TIRE_MOTION_CONTROL_SPIN_TEST_H

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

#endif //TIRE_MOTION_CONTROL_SPIN_TEST_H
