#ifndef MOTOR_HAL_HPP
#define MOTOR_HAL_HPP

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "motor_config.hpp"

/***************************************************************
 * Enum: MotorState
 * Description:
 *     - Defines motor direction and state.
 *     - Used to control H-bridge inputs.
 *
 * Values:
 *     STOP      : Motor is stopped
 *     FORWARD   : Clockwise rotation (CW)
 *     BACKWARD  : Counter-clockwise rotation (CCW)
 ****************************************************************/
enum class MotorState {
    STOP,
    FORWARD,   // CW
    BACKWARD   // CCW
};


/***************************************************************
 * Class: MotorHal
 * Layer: HAL (Hardware Abstraction Layer)
 * Description:
 *     - Low-level interface for motor driver control.
 *     - Handles GPIO setup, PWM generation, and direction control.
 *     - Provides basic motor operations (init, drive, stop).
 ****************************************************************/
class MotorHal {
public:
    /***************************************************************
     * Constructor
     * Description:
     *     - Initializes motor control pins.
     *     - Does NOT configure hardware (call motorInit separately).
     *
     * Parameters:
     *     in1 (uint): Direction control pin 1
     *     in2 (uint): Direction control pin 2
     *     en  (uint): PWM enable pin
     ***************************************************************/
    MotorHal(uint in1, uint in2, uint en);

    /***************************************************************
     * Method: motorInit
     * Description:
     *     - Configures GPIO pins and PWM for motor control.
     *     - Initializes PWM slice and sets default state.
     *     - Must be called before using the motor.
     ***************************************************************/
    void motorInit();

    /***************************************************************
     * Method: setMotorOutput
     * Description:
     *     - Controls motor direction and speed.
     *     - Sets GPIO direction pins and PWM duty cycle.
     *
     * Parameters:
     *     state (MotorState): Desired motor direction
     *     duty  (float)     : PWM duty cycle [0.0 → 1.0]
     ***************************************************************/
    void setMotorOutput(MotorState state, float duty); // duty: 0.0 -> 1.0

    /***************************************************************
     * Method: stop
     * Description:
     *     - Stops the motor immediately.
     *     - Disables PWM output and sets safe state.
     ***************************************************************/
    void stop();

private:
    uint _in1, _in2, _en;       // Motor control pins
    uint _pwm_slice;            // PWM slice used for enable pin
    bool _initialized = false;  // Tracks initialization status
};

#endif