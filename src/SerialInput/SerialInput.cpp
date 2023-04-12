//
// Created by cui on 2023/4/10.
//

#include "SerialInput/SerialInput.h"

// SerialInput 全局指针
SerialInput *serial_input_ptr = nullptr;

SerialInput::SerialInput()
{
    serial_input_ptr = this;
    // 每250ms尝试读取一下串口是否有消息
    FlexiTimer2::set(500, 1.0/1000, []() { serial_input_ptr->onScan(); });
    FlexiTimer2::start();
}

// 定时器中断服务函数（定时扫描串口是否有数据可读）
void SerialInput::onScan()
{
    if (Serial.available() <= 0)
        return;

    // read serial data
    while (Serial.available() > 0)
    {
        input_message += (char) Serial.read();
    }
    Serial.println("--------------------");
    Serial.println("serial_input: " + String(input_message));

}

void SerialInput::clear()
{
    input_message = "";
}
