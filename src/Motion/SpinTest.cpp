//
// Created by cui on 2023/4/8.
//

#include "Motion/SpinTest.h"

SpinTest::SpinTest()
{
}

void SpinTest::init()
{
    //stepperV.setMaxSpeed(1600.0);
    stepperH.setMaxSpeed(50.0);

    //stepperV.setAcceleration(800.0);
    stepperH.setAcceleration(100.0);

    //stepperV.setCurrentPosition(0);
    stepperH.setCurrentPosition(0);

    stepperH.setSpeed(50.0);
}

void SpinTest::run()
{
    stepperH.move(200);
    while(stepperH.isRunning()) {
        stepperH.run();
    }
    Serial.println("delay 1s");
    delay(1000);
}
