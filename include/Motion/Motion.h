//
// Created by cui on 2023/4/9.
//

#ifndef ARDUINO_TEST_MOTION_H
#define ARDUINO_TEST_MOTION_H

#include <Arduino.h>
#include <../lib/FlexiTimer2/FlexiTimer2.h>
#include <../lib/AccelStepper/AccelStepper.h>
#include <../lib/TimerOne/TimerOne.h>
#include "SerialInput/SerialInput.h"

enum CmdType
{
    RELIEVE_STOPALL     = 1,
    RESET_ALL_STEPPER   = 2,
    SETUP_SYSTEM        = 3,
    GET_STEPPER_INFO    = 4,    // 获取所有电机信息

    MOVE_X              = 10,
    REVERSE_MOVE_X      = 11,
    MOVE_X_TO           = 12,
    GET_POSITION_X      = 13,
    SET_SPEED_X         = 14,
    GET_SPEED_X         = 15,
    SET_ACCELERATION_X  = 16,
    RESET_X             = 17,

    MOVE_Y              = 20,
    REVERSE_MOVE_Y      = 21,
    MOVE_Y_TO           = 22,
    GET_POSITION_Y      = 23,
    SET_SPEED_Y         = 24,
    GET_SPEED_Y         = 25,
    SET_ACCELERATION_Y  = 26,
    RESET_Y             = 27,

    ROTATE_C            = 30,
    REVERSE_ROTATE_C    = 31,
    ROTATE_C_TO         = 32,
    GET_POSITION_C      = 33,
    SET_SPEED_C         = 34,
    GET_SPEED_C         = 35,
    SET_ACCELERATION_C  = 36,
    RESET_C             = 37,
    ROTATE_LOOP_C       = 38,
    ROTATE_FOREVER_C    = 39,
};

enum Direction
{
    NONE                = 0,
    LEFT                = 1,
    RIGHT               = 2,
    UP                  = 3,
    DOWN                = 4,
    CLOCKWISE           = 5,
    COUNTERCLOCKWISE    = 6,
};

class Motion
{
public:
    explicit Motion(const SerialInput &m_serial_input);

    void init();

    void initSteppers();

    void initLimitSwitchINT() const;

    void run();

    void resetAllSteppers();

    void resetOneStepper(AccelStepper &stepper, const byte &stopPin) const;

    void onStopAndSetPos0_X();

    void onStopAndSetPos0_Y();

    void onStop_X();

    void onStop_Y();

    void processSerialInput();

    void processCmd();

    void runSteppers();

    void onRotateInterval();

    static inline bool hasChar(const String& str, char c);

    inline bool reachMaxX() const;

    inline bool reachMaxY() const;

    inline bool reachMinX() const;

    inline bool reachMinY() const;

    inline void clearSerialMessage();

    inline void clearCurrentCmd();

    inline void stopStepperX();

    inline void stopStepperY();

    inline void stopStepperC();

private:
    SerialInput serial_input_manager;

    // 电机单位
    const long MM       = 40;           // millimeter
    const long SPR      = 400;          // steps/rev 400脉冲每转
    const long SPR_C    = 800;          // 底部旋转电机每转800脉冲

    // 电机速度，加速度默认设置
    const uint8_t SPEED_DEFAULT         = 3;    // 电机默认速度：3r/s
    const uint8_t SPEED_MAX             = 3;    // 电机所允许设置的最大转速设置：3r/s
    const uint8_t ACCELERATION_DEFAULT  = 6;    // 电机默认加速度：6r/s2
    const uint8_t ACCELERATION_MAX      = 10;   // 电机所允许设置的最大加速度设置：10r/s2
    const uint8_t ROTATE_SPEED_DEFAULT  = 25;   // 电机默认旋转速度：25度/s
    const uint8_t ROTATE_SPEED_MAX      = 100;  // 电机所允许设置的旋转最大转速设置：100度/s
    const uint8_t ROTATE_ACC_DEFAULT    = 60;   // 电机默认角加速度：60度/s2
    const uint8_t ROTATE_ACC_MAX        = 200;  // 电机所允许设置的最大角加速度设置：200度/s2

    // 步进电机（脉冲和方向）引脚
    const byte PIN_STEP_X   = 4;
    const byte PIN_DIR_X    = 5;
    const byte PIN_STEP_Y   = 6;
    const byte PIN_DIR_Y    = 7;
    const byte PIN_STEP_C   = 10;
    const byte PIN_DIR_C    = 11;

    // （限位器）外部中断引脚定义
    const byte PIN_MIN_X = 2;       // interruptNum 0
    const byte PIN_MAX_X = 3;       // interruptNum 1
    const byte PIN_MIN_Y = 21;      // interruptNum 4
    const byte PIN_MAX_Y = 20;      // interruptNum 5

    // 导轨参数
    const long GUIDE_MAX_LENGTH_X = 210; // mm
    const long GUIDE_MAX_LENGTH_Y = 210; // mm

    char STOP_ALL   = 'A';  // 'A' == 65
    char STOP_X     = 'X';  // 'X' == 88
    char STOP_Y     = 'Y';  // 'Y' == 89
    char STOP_C     = 'C';  // 'C' == 67

    String serial_message   = "";       // 上位机发来的消息
    String current_cmd      = "";       // 本机要执行的命令字符串

    int cmd_type;
    long cmd_value;

    bool isRunning_X;       // X电机是否正在运动，若正在运动，则不允许执行其他指令（除了STOP_ALL）
    bool isRunning_X_old;
    bool isRunning_Y;       // Y电机是否正在运动，若正在运动，则不允许执行其他指令（除了STOP_ALL）
    bool isRunning_Y_old;
    bool isRunning_C;       // C电机是否正在运动，若正在运动，则不允许执行其他指令（除了STOP_ALL）
    bool isRunning_C_old;
    bool isStopped;
    bool rotate_loop_c_running;

    bool isArrived_X;
    bool isArrived_Y;
    bool isArrived_C;

    AccelStepper stepperY = AccelStepper(AccelStepper::DRIVER, PIN_STEP_Y, PIN_DIR_Y);
    AccelStepper stepperX = AccelStepper(AccelStepper::DRIVER, PIN_STEP_X, PIN_DIR_X);
    AccelStepper stepperC = AccelStepper(AccelStepper::DRIVER, PIN_STEP_C, PIN_DIR_C);

    Direction direction;
};

#endif //ARDUINO_TEST_MOTION_H
