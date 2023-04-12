#include "Util/Util.h"
#include "Motion/Motion.h"
#include "SerialInput/SerialInput.h"

SerialInput serial_input;
Motion motion(serial_input);

void setup()
{
    Util::init();
    motion.init();
}

void loop()
{
    motion.run();
}
