//
// Created by cui on 2023/4/9.
//

#include "Motion/Motion.h"
#include "SerialInput/SerialInput.h"

// Motion 全局指针
Motion *motion_ptr = nullptr;

Motion::Motion(const SerialInput &m_serial_input)
{
    motion_ptr = this;
    this->serial_input_manager = m_serial_input;
}

void Motion::init()
{
    Timer1.initialize(3000000);
    Timer1.stop();

    isRunning_X = false;
    isRunning_Y = false;
    isRunning_C = false;
    isStopped = false;
    rotate_loop_c_running = false;

    direction = Direction::NONE;

    // 电机速度、加速度设置
    initSteppers();
    // 限位开关设置（下降沿触发）
    initLimitSwitchINT();

    Serial.println("init ok...");
}

void Motion::initSteppers()
{
    stepperX.setMaxSpeed(float(SPEED_DEFAULT * SPR));
    stepperY.setMaxSpeed(float(SPEED_DEFAULT * SPR));
    stepperC.setMaxSpeed(float(ROTATE_SPEED_DEFAULT * SPR_C) / 360);
    stepperX.setAcceleration(float(ACCELERATION_DEFAULT * SPR));
    stepperY.setAcceleration(float(ACCELERATION_DEFAULT * SPR));
    stepperC.setAcceleration(float(ROTATE_ACC_DEFAULT * SPR_C) / 360);
}

void Motion::initLimitSwitchINT() const
{
    pinMode(PIN_MIN_X, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_MIN_X),
                    [](){ motion_ptr->onStopAndSetPos0_X(); },
                    FALLING);

    pinMode(PIN_MIN_Y, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_MIN_Y),
                    [](){ motion_ptr->onStopAndSetPos0_Y(); },
                    FALLING);

    pinMode(PIN_MAX_X, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_MAX_X),
                    [](){ motion_ptr->onStop_X(); },
                    FALLING);

    pinMode(PIN_MAX_Y, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_MAX_Y),
                    [](){ motion_ptr->onStop_Y(); },
                    FALLING);
}

void Motion::run()
{
    // 解析串口输入
    processSerialInput();
    // 解析命令
    processCmd();
    // 执行运动指令
    runSteppers();
}

// 电机复位
void Motion::resetAllSteppers()
{
    stepperX.setCurrentPosition(GUIDE_MAX_LENGTH_X * MM);   // 先将电机坐标置于较远位置（210mm）
    stepperY.setCurrentPosition(GUIDE_MAX_LENGTH_Y * MM);
    direction = Direction::LEFT;
    resetOneStepper(stepperX, PIN_MIN_X);// 然后将电机向原点运动，直至触发限位开关
    direction = Direction::UP;
    resetOneStepper(stepperY, PIN_MIN_Y);
}
void Motion::resetOneStepper(AccelStepper& stepper, const byte& stopPin) const
{
    // 电机复位前先检查一下是否已经在初始位置了，如果已经在初始位置，则无需移动，位置直接置为0即可
    if (digitalRead(stopPin) == LOW) {
        stepper.stop();
        stepper.setCurrentPosition(0);
        return;
    }
    if (!isStopped)
        stepper.runToNewPosition(0);
}

/* 外部中断-限位器中断服务函数 */
// MIN_X
void Motion::onStopAndSetPos0_X()
{
    if (direction != Direction::LEFT)
        return;
    stepperX.stop();                    // 停止电机
    stepperX.setCurrentPosition(0);     // 并将此处置为原点，注意：setCurrentPosition为阻塞运动
    Serial.println("Interrupted by PIN_MIN_X, stepperX stopped...Reached min X");
}
// MIN_Y
void Motion::onStopAndSetPos0_Y()
{
    if (direction != Direction::UP)
        return;
    stepperY.stop();
    stepperY.setCurrentPosition(0);
    Serial.println("Interrupted by PIN_MIN_Y, stepperY stopped...Reached min Y");
}
// MAX_X
void Motion::onStop_X()
{
    if (direction != Direction::RIGHT)
        return;
    stepperX.stop();
    Serial.println("Interrupted by PIN_MAX_X, stepperX stopped...Reached max X");
}
// MAX_Y
void Motion::onStop_Y()
{
    if (direction != Direction::DOWN)
        return;
    stepperY.stop();
    Serial.println("Interrupted by PIN_MAX_Y, stepperY stopped...Reached max Y");
}

void Motion::processSerialInput()
{
    this->serial_message = this->serial_input_manager.input_message;
    if (this->serial_message.length() == 0)
        return;

    // 一旦发现有stop信号，立马急停
    if (hasChar(serial_message, STOP_ALL))
    {
        stepperX.stop();
        stepperY.stop();

        Serial.println("found STOP_ALL char from serial_input, stop all motors...");
        clearCurrentCmd();
        clearSerialMessage();
        isStopped = true;
        return;
    }
    if (hasChar(serial_message, STOP_X))
    {
        Serial.println("found STOP_X char from serial_input, stop stepperX...");
        stepperX.stop();
        clearCurrentCmd();
        clearSerialMessage();
        return;
    }
    if (hasChar(serial_message, STOP_Y))
    {
        Serial.println("found STOP_Y char from serial_input, stop stepperY...");
        stepperY.stop();
        clearCurrentCmd();
        clearSerialMessage();
        return;
    }
    if (hasChar(serial_message, STOP_C))
    {
        Serial.println("found STOP_C char from serial_input, stop stepperC...");
        stepperC.stop();
        if (rotate_loop_c_running)
        {
            rotate_loop_c_running = false;
            Timer1.stop();
        }
        clearCurrentCmd();
        clearSerialMessage();
        return;
    }

    // 把消息赋值给命令去解析并执行
    current_cmd = serial_message;
    Serial.println("input serial to current_cmd: " + String(current_cmd));
    clearSerialMessage();
}

void Motion::processCmd()
{
    // 校验命令长度
    if (current_cmd.length() <= 0)
    {
        return;
    }
    if (current_cmd.length() > 10)
    {
        Serial.println("Invalid command length, last: " + String(current_cmd.length()));
        clearCurrentCmd();
        return;
    }
    Serial.println("....................");
    Serial.println("processCmd() start running...");
    // 只有电机静止状态才能响应其他指令(除非STOP_ALL急停指令）
    if (isRunning_X || isRunning_Y)
    {
        Serial.println("The stepper is running, please try again later...");
        clearCurrentCmd();
        return;
    }
    Serial.println("current_command: " + String(current_cmd));

    // 解析cmd
    if (hasChar(current_cmd, '/'))
    {
        int splitIndex = current_cmd.indexOf('/');
        cmd_type = (int) current_cmd.substring(0, splitIndex).toInt();
        cmd_value = (long) current_cmd.substring(splitIndex + 1).toDouble();
    }
    else
    {
        cmd_type = (int) current_cmd.toInt();
        cmd_value = -1;
    }
    Serial.println("cmd_type: " + String(cmd_type));
    Serial.println("cmd_value: " + String(cmd_value));

    // 判断命令类型是否有效(cmd不是以数字开头会使得toInt()函数转换失败即为0)
    if (cmd_type <= 0)
    {
        Serial.println("Invalid command type: " + String(cmd_type));
        clearCurrentCmd();
        return;
    }

    switch (cmd_type)
    {
        // 第一组
        // 1 急停恢复
        case RELIEVE_STOPALL:
            isStopped = false;
            Serial.println("...relieve all steppers from stop finished...");
            break;

        // 2 所有电机复位
        case RESET_ALL_STEPPER:
            resetAllSteppers();
            Serial.println("...reset all steppers finished...");
            break;

        // 3 重置arduino程序，运行setup()
        case SETUP_SYSTEM:
            this->init();
            Serial.println("...re-init program finished...");
            break;

        // 4 获取所有电机信息(电机X，Y的坐标和速度)
        //case GET_STEPPER_INFO:
        //    String info;
        //    stepperX.currentPosition();
        //    Serial.println(stepperY.currentPosition() / MM);

        // X 命令组
        // 10 X电机向右移动一定的距离
        case MOVE_X:
            // 想要增加X坐标时，需要确保X未达到MAX_X
            if (reachMaxX())
                Serial.println("X has reached MAX_X, can't move right...");
            else if (cmd_value <= 0)
                Serial.println("Error: cmd_value <= 0, use cmd_type: 11 to move left.");
            else
            {
                direction = Direction::RIGHT;
                stepperX.move(cmd_value * MM);
            }
            break;

        // 11 X电机向左移动一定的距离
        case REVERSE_MOVE_X:
            // 想要减小X坐标时，需要确保未达到MIN_X
            if (reachMinX())
            {
                Serial.println("X has reached MIN_X, can't move left...");
                break;
            }
            if (cmd_value <= 0)
                Serial.println("Error: cmd_value <= 0, use cmd_type: 10 to move right.");
            else
            {
                direction = Direction::LEFT;
                stepperX.move(-cmd_value * MM);
            }
            break;

        // 12 X电机移动到某绝对坐标
        case MOVE_X_TO:
            // 想要到达某个X坐标时，先确保目标X坐标大于等于0；且（未达到MAX_X或者目标坐标比当前坐标小）
            if (cmd_value < 0)
            {
                Serial.println("Error: cannot move to absolute position less than 0.");
                break;
            }
            if (reachMaxX() && cmd_value * MM >= stepperX.currentPosition())
            {
                Serial.println("Error: cannot move to absolute position greater than MAX_X.");
                break;
            }
            Serial.println("move stepper X to: " + String(cmd_value));
            if (cmd_value * MM > stepperX.currentPosition())
                direction = Direction::RIGHT;
            else
                direction = Direction::LEFT;
            stepperX.moveTo(cmd_value * MM);
            break;

        // 13 获取电机X当前位置
        case GET_POSITION_X:
            Serial.println("stepper X current position: " + String(stepperX.currentPosition() / MM));
            break;

        // 14 设置电机X的最大速度
        case SET_SPEED_X:
            if (cmd_value > SPEED_MAX)
            {
                Serial.println("Error: cmd_value > SPEED_MAX");
            }
            else if (cmd_value <= 0)
            {
                Serial.println("Error: cmd_value <= 0");
            }
            else
            {
                Serial.println("set stepper X max speed: " + String(cmd_value));
                stepperX.setMaxSpeed(float(cmd_value * SPR));
            }
            break;

        // 15 获取电机X当前最大速度
        case GET_SPEED_X:
            Serial.println("stepper X max speed: " + String(stepperX.maxSpeed() / float(MM)));
            break;

        // 16 设置电机X的加速度
        case SET_ACCELERATION_X:
            if (cmd_value > ACCELERATION_MAX)
            {
                Serial.println("Error: cmd_value > ACCELERATION_MAX");
            }
            else if (cmd_value <= 0)
            {
                Serial.println("Error: cmd_value <= 0");
            }
            else
            {
                Serial.println("set stepper X acceleration: " + String(cmd_value));
                stepperX.setAcceleration(float(cmd_value * SPR));
            }
            break;

        // 17 电机X复位
        case RESET_X:
            stepperX.setCurrentPosition(GUIDE_MAX_LENGTH_X * MM);    // 先将电机坐标置于较远位置
            direction = Direction::LEFT;
            resetOneStepper(stepperX, PIN_MIN_X);// 然后将电机向原点运动，直至触发限位开关
            Serial.println("reset stepper X finished...");
            break;

        // Y 命令组
        // 20
        case MOVE_Y:
            // 想要增加Y坐标时，需要确保Y达到MAX_Y
            if (reachMaxY())
                Serial.println("Y has reached MAX_Y, can't move down...");
            else if (cmd_value <= 0)
                Serial.println("Error: cmd_value <= 0, use cmd_type: 21 to move down.");
            else
            {
                direction = Direction::DOWN;
                stepperY.move(cmd_value * MM);
            }
            break;

        // 21
        case REVERSE_MOVE_Y:
            // 想要减小Y坐标时，需要确保未达到MIN_Y
            if (reachMinY())
            {
                Serial.println("Y has reached MIN_Y, can't move up...");
                break;
            }
            if (cmd_value <= 0)
                Serial.println("Error: cmd_value <= 0, use cmd_type: 20 to move up.");
            else
            {
                direction = Direction::UP;
                stepperY.move(-cmd_value * MM);
            }
            break;

        // 22
        case MOVE_Y_TO:
            // 想要到达某个Y坐标时，先确保目标Y坐标大于等于0；且（未达到MAX_Y或者目标坐标比当前坐标小）
            if (cmd_value < 0)
            {
                Serial.println("Error: cannot move to absolute position less than 0.");
                break;
            }
            if (reachMaxY() && cmd_value * MM >= stepperY.currentPosition())
            {
                Serial.println("Error: cannot move to absolute position greater than MAX_Y.");
                break;
            }
            Serial.println("move stepper Y to: " + String(cmd_value));
            if (cmd_value * MM > stepperY.currentPosition())
                direction = Direction::DOWN;
            else
                direction = Direction::UP;
            stepperY.moveTo(cmd_value * MM);
            break;

        // 23 获取电机Y当前位置
        case GET_POSITION_Y:
            Serial.println("stepper Y current position: " + String(stepperY.currentPosition() / MM));
            break;

        // 24 设置电机Y的最大速度
        case SET_SPEED_Y:
            if (cmd_value > SPEED_MAX)
            {
                Serial.println("Error: cmd_value > SPEED_MAX");
            }
            else if (cmd_value <= 0)
            {
                Serial.println("Error: cmd_value <= 0");
            }
            else
            {
                Serial.println("set stepper Y max speed: " + String(cmd_value));
                stepperY.setMaxSpeed(float(cmd_value * SPR));
            }
            break;

        // 25 获取电机Y当前最大速度
        case GET_SPEED_Y:
            Serial.println("stepper Y max speed: " + String(stepperY.maxSpeed() / float(MM)));
            break;

        // 26 设置电机Y的加速度
        case SET_ACCELERATION_Y:
            if (cmd_value > ACCELERATION_MAX)
            {
                Serial.println("Error: cmd_value > ACCELERATION_MAX");
            }
            else if (cmd_value <= 0)
            {
                Serial.println("Error: cmd_value <= 0");
            }
            else
            {
                Serial.println("set stepper Y acceleration: " + String(cmd_value));
                stepperY.setAcceleration(float(cmd_value * SPR));
            }
            break;

        // 27 电机Y复位
        case RESET_Y:
            stepperY.setCurrentPosition(GUIDE_MAX_LENGTH_Y * MM);    // 先将电机坐标置于较远位置
            direction = Direction::UP;
            resetOneStepper(stepperY, PIN_MIN_Y);// 然后将电机向原点运动，直至触发限位开关
            Serial.println("reset stepper Y finished...");
            break;

        // C 命令组
        // 30 C电机向顺时针旋转一定的角度
        case ROTATE_C:
            if (cmd_value <= 0)
                Serial.println("Error: cmd_value <= 0, use cmd_type: 31 to rotate counterclockwise.");
            else
            {
                direction = Direction::CLOCKWISE;
                stepperC.move(cmd_value * SPR_C / 360);
            }
            break;

        // 31 C电机向逆时针旋转一定的角度
        case REVERSE_ROTATE_C:
            if (cmd_value <= 0)
                Serial.println("Error: cmd_value <= 0, use cmd_type: 30 to rotate clockwise.");
            else
            {
                direction = Direction::COUNTERCLOCKWISE;
                stepperC.move(-cmd_value * SPR_C / 360);
            }
            break;

        // 32 C电机移动到某绝对角度
        case ROTATE_C_TO:
            Serial.println("rotate stepper C to: " + String(cmd_value));
            if (cmd_value * MM > stepperC.currentPosition())
                direction = Direction::CLOCKWISE;
            else
                direction = Direction::COUNTERCLOCKWISE;
            stepperC.moveTo(cmd_value * SPR_C / 360);
            break;

        // 33 获取电机C当前位置
        case GET_POSITION_C:
            Serial.println("stepper C current position: " + String(stepperC.currentPosition() * 360 / SPR_C));
            break;

        // 34 设置电机C的最大速度
        case SET_SPEED_C:
            if (cmd_value > ROTATE_SPEED_MAX)
            {
                Serial.println("Error: cmd_value > SPEED_MAX");
            }
            else if (cmd_value <= 0)
            {
                Serial.println("Error: cmd_value <= 0");
            }
            else
            {
                Serial.println("set stepper C max speed: " + String(cmd_value));
                stepperC.setMaxSpeed(float(cmd_value * SPR_C) / 360);
            }
            break;

        // 35 获取电机C当前最大速度
        case GET_SPEED_C:
            Serial.println("stepper C max speed: " + String(stepperX.maxSpeed()));
            break;

        // 36 设置电机C的加速度
        case SET_ACCELERATION_C:
            if (cmd_value > ROTATE_ACC_MAX)
            {
                Serial.println("Error: cmd_value > ACCELERATION_MAX");
            }
            else if (cmd_value <= 0)
            {
                Serial.println("Error: cmd_value <= 0");
            }
            else
            {
                Serial.println("set stepper C acceleration: " + String(cmd_value));
                stepperC.setAcceleration(float(cmd_value));
            }
            break;

        // 37 电机C复位 - 隐藏
        // case RESET_C:
        //     stepperC.runToNewPosition(0);
        //     Serial.println("reset stepper C finished...");
        //     break;

        // 38 循环：电机C运动cmd_value角度后停止1s，默认30度
        case ROTATE_LOOP_C:
            rotate_loop_c_running = true;
            if (cmd_value <= 0)
                cmd_value = 20;
            direction = Direction::COUNTERCLOCKWISE;
            Timer1.attachInterrupt([](){ motion_ptr->onRotateInterval(); });
            Timer1.start();
            break;

        // 39 让电机C一直旋转
        case ROTATE_FOREVER_C:
            direction = Direction::CLOCKWISE;
            stepperC.move(1000000);
            Serial.println("rotate stepper C forever...");
            break;

        default:
            Serial.println("ERROR: command: " + String(cmd_type) + " didn't match!!!");
            break;
    }
    Serial.println("processCmd() finished...");
    Serial.println("--------------------");
    clearCurrentCmd();    // 处理完命令，及时清空命令字符串(应该放在if里面，不满足if条件则不用清空，否则可能会误删cmd)
}

void Motion::runSteppers()
{
    isRunning_X_old = isRunning_X;
    isRunning_X = stepperX.run();
    isArrived_X = isRunning_X_old && !isRunning_X;
    if (isArrived_X) Serial.println("...stepper X arrived...ok");

    isRunning_Y_old = isRunning_Y;
    isRunning_Y = stepperY.run();
    isArrived_Y = isRunning_Y_old && !isRunning_Y;
    if (isArrived_Y) Serial.println("...stepper Y arrived...ok");

    isRunning_C_old = isRunning_C;
    isRunning_C = stepperC.run();
    isArrived_C = isRunning_C_old && !isRunning_C;
    if (isArrived_C) Serial.println("...stepper C arrived...ok");
}

bool Motion::hasChar(const String& str, char c)
{
    return str.indexOf(c) != -1;
}

bool Motion::reachMaxX() const
{
    return digitalRead(PIN_MAX_X) == LOW;
}

bool Motion::reachMaxY() const
{
    return digitalRead(PIN_MAX_Y) == LOW;
}

bool Motion::reachMinX() const
{
    return digitalRead(PIN_MIN_X) == LOW;
}

bool Motion::reachMinY() const
{
    return digitalRead(PIN_MIN_Y) == LOW;
}

void Motion::clearSerialMessage()
{
    serial_message = "";
    this->serial_input_manager.clear();
}

void Motion::clearCurrentCmd()
{
    current_cmd = "";
}

void Motion::onRotateInterval()
{
    stepperC.move(-cmd_value * SPR_C / 360);
    Serial.println("start rotating stepper C step: " + String(cmd_value) + " ...controlled by interrupt");
}
