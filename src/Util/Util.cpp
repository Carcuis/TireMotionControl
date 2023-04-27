//
// Created by cui on 2023/4/8.
//

#include "Util/Util.h"

Util::Util()
= default;

void Util::init()
{
    Serial.begin(115200, SERIAL_8N1);
}
