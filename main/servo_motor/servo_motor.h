#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <driver/gpio.h>
#include <esp_timer.h>

class ServoMotor {
public:
    ServoMotor(gpio_num_t gpio, uint32_t stop_pulse_us,
               uint32_t clockwise_pulse_us, uint32_t counterclockwise_pulse_us);
    ~ServoMotor();

    void ToggleDirection();
    void ToggleDirectionRepeated(uint32_t count, uint32_t interval_ms);
    void Stop();

private:
    void SetPulseWidth(uint32_t pulse_width_us);

    uint32_t stop_pulse_us_;
    uint32_t clockwise_pulse_us_;
    uint32_t counterclockwise_pulse_us_;
    bool clockwise_ = false;
    uint32_t toggles_remaining_ = 0;
    esp_timer_handle_t toggle_timer_ = nullptr;
};

#endif  // SERVO_MOTOR_H
