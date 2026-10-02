```cpp
#pragma once

#include "mcp_server.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

class MotorController {
public:
    MotorController(
        gpio_num_t left_ena,
        gpio_num_t left_in1,
        gpio_num_t left_in2,
        gpio_num_t right_enb,
        gpio_num_t right_in3,
        gpio_num_t right_in4
    );

    void Stop();

    // speed: -100 ... +100
    // > 0 = forward
    // < 0 = backward
    void SetLeftSpeed(int speed);
    void SetRightSpeed(int speed);

    // Differential drive
    // left/right: -100 ... +100
    void SetWheelSpeeds(int left_speed, int right_speed);

private:
    static constexpr int PWM_FREQ_HZ = 20000;
    static constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_10_BIT;
    static constexpr int PWM_MAX = 1023;

    gpio_num_t left_ena_;
    gpio_num_t left_in1_;
    gpio_num_t left_in2_;

    gpio_num_t right_enb_;
    gpio_num_t right_in3_;
    gpio_num_t right_in4_;

    int left_speed_ = 0;
    int right_speed_ = 0;

    void ConfigureGPIO(gpio_num_t gpio);
    void ConfigurePwm();
    void SetMotor(
        int speed,
        gpio_num_t en,
        gpio_num_t in1,
        gpio_num_t in2,
        ledc_channel_t channel
    );

    int ClampSpeed(int speed);

    void RegisterMcpTools();
};
```
