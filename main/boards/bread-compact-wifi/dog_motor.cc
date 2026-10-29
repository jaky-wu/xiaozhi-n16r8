#include "dog_motor.h"
#include "config.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_timer.h>
#include <esp_log.h>

#include <algorithm>
#include <cstdlib>

#define TAG "DogMotor"

// 四路电机：0=左前 1=右前 2=左后 3=右后
static const gpio_num_t kIn1[4] = {MOTOR_M1_IN1, MOTOR_M2_IN1, MOTOR_M3_IN1, MOTOR_M4_IN1};
static const gpio_num_t kIn2[4] = {MOTOR_M1_IN2, MOTOR_M2_IN2, MOTOR_M3_IN2, MOTOR_M4_IN2};
static const gpio_num_t kStby = MOTOR_STBY;

static const ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t kTimer[4] = {LEDC_TIMER_0, LEDC_TIMER_1, LEDC_TIMER_2, LEDC_TIMER_3};
static const ledc_channel_t kChA[4] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
static const ledc_channel_t kChB[4] = {LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_CHANNEL_6, LEDC_CHANNEL_7};

static const int kDutyMax = 1023;  // 10 bit

static void StopTimerCallback(void* arg) {
    DogMotor::GetInstance().Stop();
}

DogMotor& DogMotor::GetInstance() {
    static DogMotor instance;
    return instance;
}

void DogMotor::Init() {
    if (inited_) {
        return;
    }

    gpio_config_t io = {};
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    uint64_t mask = 0;
    for (int i = 0; i < 4; i++) {
        mask |= (1ULL << kIn1[i]) | (1ULL << kIn2[i]);
    }
    mask |= (1ULL << kStby);
    io.pin_bit_mask = mask;
    gpio_config(&io);

    // 待机脚拉高，否则 DRV8833 不输出
    gpio_set_level(kStby, 1);

    ledc_timer_config_t tcfg = {};
    tcfg.speed_mode = kMode;
    tcfg.duty_resolution = LEDC_TIMER_10_BIT;
    tcfg.freq_hz = 20000;
    tcfg.clk_cfg = LEDC_AUTO_CLK;

    ledc_channel_config_t ccfg = {};
    ccfg.speed_mode = kMode;
    ccfg.intr_type = LEDC_INTR_DISABLE;
    ccfg.duty = 0;
    ccfg.hpoint = 0;

    for (int i = 0; i < 4; i++) {
        tcfg.timer_num = kTimer[i];
        ledc_timer_config(&tcfg);

        ccfg.channel = kChA[i];
        ccfg.timer_sel = kTimer[i];
        ccfg.gpio_num = kIn1[i];
        ledc_channel_config(&ccfg);

        ccfg.channel = kChB[i];
        ccfg.timer_sel = kTimer[i];
        ccfg.gpio_num = kIn2[i];
        ledc_channel_config(&ccfg);
    }

    esp_timer_create_args_t args = {};
    args.callback = &StopTimerCallback;
    args.name = "dog_stop";
    esp_timer_handle_t handle = nullptr;
    if (esp_timer_create(&args, &handle) == ESP_OK) {
        stop_timer_ = handle;
    }

    inited_ = true;
    ESP_LOGI(TAG, "motor driver ready (4 channels, 20kHz)");
}

void DogMotor::SetChannel(int index, int percent) {
    percent = std::clamp(percent, -100, 100);
    int duty = std::abs(percent) * kDutyMax / 100;

    int duty_a = 0;
    int duty_b = 0;
    if (percent > 0) {
        duty_a = duty;
    } else if (percent < 0) {
        duty_b = duty;
    }
    ledc_set_duty(kMode, kChA[index], duty_a);
    ledc_update_duty(kMode, kChA[index]);
    ledc_set_duty(kMode, kChB[index], duty_b);
    ledc_update_duty(kMode, kChB[index]);
}

void DogMotor::Drive(int forward, int turn) {
    int left = std::clamp(forward + turn, -100, 100);
    int right = std::clamp(forward - turn, -100, 100);
    SetChannel(0, left);
    SetChannel(2, left);
    SetChannel(1, right);
    SetChannel(3, right);
}

void DogMotor::Move(const std::string& direction, int speed, int duration_ms) {
    if (!inited_) {
        return;
    }
    speed = std::clamp(speed, 0, 100);

    if (stop_timer_ != nullptr) {
        esp_timer_stop(static_cast<esp_timer_handle_t>(stop_timer_));
    }

    direction_ = direction;
    speed_ = speed;

    if (direction == "forward") {
        Drive(speed, 0);
    } else if (direction == "back") {
        Drive(-speed, 0);
    } else if (direction == "left") {
        Drive(0, -speed);
    } else if (direction == "right") {
        Drive(0, speed);
    } else if (direction == "stop") {
        Stop();
        return;
    } else {
        ESP_LOGW(TAG, "unknown direction: %s", direction.c_str());
        return;
    }

    ESP_LOGI(TAG, "move %s speed=%d duration=%dms", direction.c_str(), speed, duration_ms);

    if (duration_ms > 0 && stop_timer_ != nullptr) {
        esp_timer_start_once(static_cast<esp_timer_handle_t>(stop_timer_),
                             static_cast<uint64_t>(duration_ms) * 1000ULL);
    }
}

void DogMotor::Stop() {
    for (int i = 0; i < 4; i++) {
        SetChannel(i, 0);
    }
    direction_ = "stop";
    speed_ = 0;
    if (stop_timer_ != nullptr) {
        esp_timer_stop(static_cast<esp_timer_handle_t>(stop_timer_));
    }
}

std::string DogMotor::Status() {
    std::string json = "{\"direction\":\"" + direction_ + "\",\"speed\":" +
                       std::to_string(speed_) + "}";
    return json;
}
