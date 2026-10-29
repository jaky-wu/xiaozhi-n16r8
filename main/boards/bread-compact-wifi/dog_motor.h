#pragma once

#include <string>

// 四路直流电机驱动（DRV8833），差速转向。
// 引脚分配见 config.h：M1(4/5) M2(6/9) M3(10/11) M4(12/13)，STBY=14
class DogMotor {
public:
    static DogMotor& GetInstance();

    void Init();

    // direction: forward / back / left / right / stop
    // speed: 0-100, duration_ms: 0 表示一直动，直到 Stop()
    void Move(const std::string& direction, int speed, int duration_ms);
    void Stop();

    std::string Status();

private:
    DogMotor() = default;
    void SetChannel(int index, int percent);
    void Drive(int forward, int turn);

    bool inited_ = false;
    int speed_ = 0;
    std::string direction_ = "stop";
    void* stop_timer_ = nullptr;
};
