```cpp
#include "motor_controller.h"

#include <algorithm>

#include "esp_log.h"
#include "driver/gpio.h"

#define TAG "MotorController"

// ============================================================
// GPIO + PWM
// ============================================================
//
// L298 MINI
//
// Left motor:
//   ENA -> GPIO1
//   IN1 -> GPIO2
//   IN2 -> GPIO4
//
// Right motor:
//   ENB -> GPIO5
//   IN3 -> GPIO6
//   IN4 -> GPIO7
//
// ============================================================

MotorController::MotorController(
    gpio_num_t left_ena,
    gpio_num_t left_in1,
    gpio_num_t left_in2,
    gpio_num_t right_enb,
    gpio_num_t right_in3,
    gpio_num_t right_in4
)
    : left_ena_(left_ena),
      left_in1_(left_in1),
      left_in2_(left_in2),
      right_enb_(right_enb),
      right_in3_(right_in3),
      right_in4_(right_in4) {

    ConfigureGPIO(left_in1_);
    ConfigureGPIO(left_in2_);
    ConfigureGPIO(right_in3_);
    ConfigureGPIO(right_in4_);

    ConfigurePwm();

    Stop();

    RegisterMcpTools();

    ESP_LOGI(TAG, "2-wheel motor controller initialized");
}

void MotorController::ConfigureGPIO(gpio_num_t gpio) {
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << gpio);
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&config));
    gpio_set_level(gpio, 0);
}

void MotorController::ConfigurePwm() {

    ledc_timer_config_t timer_config = {};

    timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_config.duty_resolution = PWM_RESOLUTION;
    timer_config.timer_num = LEDC_TIMER_0;
    timer_config.freq_hz = PWM_FREQ_HZ;
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t left_channel = {};
    left_channel.gpio_num = left_ena_;
    left_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    left_channel.channel = LEDC_CHANNEL_0;
    left_channel.intr_type = LEDC_INTR_DISABLE;
    left_channel.timer_sel = LEDC_TIMER_0;
    left_channel.duty = 0;
    left_channel.hpoint = 0;

    ESP_ERROR_CHECK(ledc_channel_config(&left_channel));

    ledc_channel_config_t right_channel = {};
    right_channel.gpio_num = right_enb_;
    right_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    right_channel.channel = LEDC_CHANNEL_1;
    right_channel.intr_type = LEDC_INTR_DISABLE;
    right_channel.timer_sel = LEDC_TIMER_0;
    right_channel.duty = 0;
    right_channel.hpoint = 0;

    ESP_ERROR_CHECK(ledc_channel_config(&right_channel));
}

int MotorController::ClampSpeed(int speed) {
    if (speed > 100) {
        return 100;
    }

    if (speed < -100) {
        return -100;
    }

    return speed;
}

void MotorController::SetMotor(
    int speed,
    gpio_num_t en,
    gpio_num_t in1,
    gpio_num_t in2,
    ledc_channel_t channel
) {
    speed = ClampSpeed(speed);

    // Stop
    if (speed == 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);

        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            channel,
            0
        );

        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            channel
        );

        return;
    }

    // Forward
    if (speed > 0) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    }
    // Reverse
    else {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
    }

    int duty = (abs(speed) * PWM_MAX) / 100;

    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        channel,
        duty
    );

    ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        channel
    );
}

void MotorController::SetLeftSpeed(int speed) {
    left_speed_ = ClampSpeed(speed);

    SetMotor(
        left_speed_,
        left_ena_,
        left_in1_,
        left_in2_,
        LEDC_CHANNEL_0
    );
}

void MotorController::SetRightSpeed(int speed) {
    right_speed_ = ClampSpeed(speed);

    SetMotor(
        right_speed_,
        right_enb_,
        right_in3_,
        right_in4_,
        LEDC_CHANNEL_1
    );
}

void MotorController::SetWheelSpeeds(
    int left_speed,
    int right_speed
) {
    left_speed_ = ClampSpeed(left_speed);
    right_speed_ = ClampSpeed(right_speed);

    SetMotor(
        left_speed_,
        left_ena_,
        left_in1_,
        left_in2_,
        LEDC_CHANNEL_0
    );

    SetMotor(
        right_speed_,
        right_enb_,
        right_in3_,
        right_in4_,
        LEDC_CHANNEL_1
    );
}

void MotorController::Stop() {
    left_speed_ = 0;
    right_speed_ = 0;

    SetMotor(
        0,
        left_ena_,
        left_in1_,
        left_in2_,
        LEDC_CHANNEL_0
    );

    SetMotor(
        0,
        right_enb_,
        right_in3_,
        right_in4_,
        LEDC_CHANNEL_1
    );
}

// ============================================================
// MCP TOOLS
// ============================================================

void MotorController::RegisterMcpTools() {

    auto& mcp_server = McpServer::GetInstance();

    // --------------------------------------------------------
    // Basic robot control
    // --------------------------------------------------------

    mcp_server.AddTool(
        "self.robot.basic_control",

        "Control a two-wheel differential drive robot. "
        "Actions: "
        "forward, backward, turn_left, turn_right, stop. "
        "Speed is from 0 to 100 percent.",

        PropertyList({
            Property("action", kPropertyTypeString),
            Property("speed", kPropertyTypeInteger, 0, 100)
        }),

        [this](const PropertyList& properties) -> ReturnValue {

            const std::string& action =
                properties["action"].value<std::string>();

            int speed =
                properties["speed"].value<int>();

            speed = std::clamp(speed, 0, 100);

            if (action == "forward") {

                SetWheelSpeeds(
                    speed,
                    speed
                );

            } else if (action == "backward") {

                SetWheelSpeeds(
                    -speed,
                    -speed
                );

            } else if (action == "turn_left") {

                SetWheelSpeeds(
                    -speed,
                    speed
                );

            } else if (action == "turn_right") {

                SetWheelSpeeds(
                    speed,
                    -speed
                );

            } else if (action == "stop") {

                Stop();

            } else {

                ESP_LOGW(
                    TAG,
                    "Unknown action: %s",
                    action.c_str()
                );

                return false;
            }

            ESP_LOGI(
                TAG,
                "Action=%s speed=%d",
                action.c_str(),
                speed
            );

            return true;
        }
    );

    // --------------------------------------------------------
    // Individual wheel control
    // --------------------------------------------------------

    mcp_server.AddTool(
        "self.robot.set_wheel_speeds",

        "Set left and right wheel speeds independently. "
        "Range -100 to 100. "
        "Positive is forward, negative is backward, zero is stop.",

        PropertyList({
            Property("left", kPropertyTypeInteger, -100, 100),
            Property("right", kPropertyTypeInteger, -100, 100)
        }),

        [this](const PropertyList& properties) -> ReturnValue {

            int left =
                properties["left"].value<int>();

            int right =
                properties["right"].value<int>();

            SetWheelSpeeds(
                left,
                right
            );

            ESP_LOGI(
                TAG,
                "Left=%d Right=%d",
                left,
                right
            );

            return true;
        }
    );

    // --------------------------------------------------------
    // Emergency stop
    // --------------------------------------------------------

    mcp_server.AddTool(
        "self.robot.stop",

        "Immediately stop both robot motors.",

        PropertyList(),

        [this](const PropertyList& properties) -> ReturnValue {

            Stop();

            ESP_LOGI(
                TAG,
                "Emergency stop"
            );

            return true;
        }
    );
}
```
