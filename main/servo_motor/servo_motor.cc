#include "servo_motor.h"

#include <driver/ledc.h>
#include <esp_log.h>

namespace {

constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kTimer = LEDC_TIMER_2;
constexpr ledc_channel_t kChannel = LEDC_CHANNEL_2;
constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_14_BIT;
constexpr uint32_t kFrequencyHz = 50;
constexpr uint32_t kPeriodUs = 1000000 / kFrequencyHz;
constexpr uint32_t kMaxDuty = (1U << 14) - 1;

const char* const TAG = "ServoMotor";

}  // namespace

ServoMotor::ServoMotor(gpio_num_t gpio, uint32_t stop_pulse_us,
                       uint32_t clockwise_pulse_us, uint32_t counterclockwise_pulse_us)
    : stop_pulse_us_(stop_pulse_us),
      clockwise_pulse_us_(clockwise_pulse_us),
      counterclockwise_pulse_us_(counterclockwise_pulse_us) {
    const ledc_timer_config_t timer_config = {
        .speed_mode = kSpeedMode,
        .duty_resolution = kDutyResolution,
        .timer_num = kTimer,
        .freq_hz = kFrequencyHz,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    const ledc_channel_config_t channel_config = {
        .gpio_num = gpio,
        .speed_mode = kSpeedMode,
        .channel = kChannel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = kTimer,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {
            .output_invert = 0,
        },
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    const esp_timer_create_args_t toggle_timer_args = {
        .callback = [](void* arg) {
            auto servo = static_cast<ServoMotor*>(arg);
            if (servo->toggles_remaining_ == 0) {
                esp_timer_stop(servo->toggle_timer_);
                return;
            }
            servo->ToggleDirection();
            --servo->toggles_remaining_;
            if (servo->toggles_remaining_ == 0) {
                esp_timer_stop(servo->toggle_timer_);
            }
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "servo_toggle",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&toggle_timer_args, &toggle_timer_));

    Stop();
    ESP_LOGI(TAG, "Initialized on GPIO %d", gpio);
}

ServoMotor::~ServoMotor() {
    if (toggle_timer_ != nullptr && esp_timer_is_active(toggle_timer_)) {
        esp_timer_stop(toggle_timer_);
    }
    Stop();
    if (toggle_timer_ != nullptr) {
        esp_timer_delete(toggle_timer_);
    }
    ledc_stop(kSpeedMode, kChannel, 0);
}

void ServoMotor::ToggleDirection() {
    clockwise_ = !clockwise_;
    if (clockwise_) {
        SetPulseWidth(clockwise_pulse_us_);
        ESP_LOGI(TAG, "Rotating clockwise continuously");
    } else {
        SetPulseWidth(counterclockwise_pulse_us_);
        ESP_LOGI(TAG, "Rotating counterclockwise continuously");
    }
}

void ServoMotor::ToggleDirectionRepeated(uint32_t count, uint32_t interval_ms) {
    if (toggle_timer_ != nullptr && esp_timer_is_active(toggle_timer_)) {
        ESP_ERROR_CHECK(esp_timer_stop(toggle_timer_));
    }
    toggles_remaining_ = count;
    if (toggles_remaining_ == 0) {
        return;
    }

    ToggleDirection();
    --toggles_remaining_;
    if (toggles_remaining_ > 0) {
        ESP_ERROR_CHECK(esp_timer_start_periodic(
            toggle_timer_, static_cast<uint64_t>(interval_ms) * 1000));
    }
    ESP_LOGI(TAG, "Started direction toggle sequence: %lu times, %lu ms interval",
        static_cast<unsigned long>(count), static_cast<unsigned long>(interval_ms));
}

void ServoMotor::Stop() {
    SetPulseWidth(stop_pulse_us_);
    ESP_LOGI(TAG, "Stopped");
}

void ServoMotor::SetPulseWidth(uint32_t pulse_width_us) {
    const uint32_t duty = static_cast<uint32_t>(
        (static_cast<uint64_t>(pulse_width_us) * kMaxDuty) / kPeriodUs);
    ESP_ERROR_CHECK(ledc_set_duty(kSpeedMode, kChannel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(kSpeedMode, kChannel));
}
