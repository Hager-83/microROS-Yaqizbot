#include "motor_hal.hpp"


// Constructor
MotorHal::MotorHal(uint in1, uint in2, uint en)
    : _in1(in1), _in2(in2), _en(en), _pwm_slice(0) {}

// Hardware init
void MotorHal::motorInit() {

    if (_initialized) 
    {
        return;
    }

    // Direction pins
    gpio_init(_in1);
    gpio_init(_in2);
    gpio_set_dir(_in1, GPIO_OUT);
    gpio_set_dir(_in2, GPIO_OUT);

    // PWM pin
    gpio_set_function(_en, GPIO_FUNC_PWM);
    _pwm_slice = pwm_gpio_to_slice_num(_en);

    pwm_set_wrap(_pwm_slice, PWM_WRAP);  // PWM resolution
    pwm_set_enabled(_pwm_slice, true);

    _initialized = true;

    stop();
}


// Set motor direction and speed (0.0 -> 1.0)
void MotorHal::setMotorOutput(MotorState state, float duty) {

    if (!_initialized) 
    {
        return;
    }
    
    // Limit duty
    if (duty < 0) duty = 0.0f;
    if (duty > 1) duty = 1.0f;

    switch (state) {
        case MotorState::FORWARD:    // FORWARD(CW)
            gpio_put(_in1, 1);
            gpio_put(_in2, 0);
            break;
        case MotorState::BACKWARD:   // BACKWARD(CCW)
            gpio_put(_in1, 0);
            gpio_put(_in2, 1);
            break;
        case MotorState::STOP:
        default:
            gpio_put(_in1, 0);
            gpio_put(_in2, 0);
            break;
    }

    pwm_set_gpio_level(_en, duty * PWM_WRAP); // Apply PWM duty cycle
}

void MotorHal::stop() {
    setMotorOutput(MotorState::STOP, 0.0f);
}