#ifndef MOTOR_SERVICE_HPP
#define MOTOR_SERVICE_HPP

#include "motor_hal.hpp"


/***************************************************************
 * Class: MotorService
 * Layer: Service Layer
 * Description:
 *     - High-level interface for motor control.
 *     - Abstracts MotorHal from the application layer.
 *     - Handles speed commands, direction, and safety limits.
 ****************************************************************/
class MotorService {
public:
    /***************************************************************
     * Constructor
     * Description:
     *     - Initializes MotorService with a reference to MotorHal.
     *     - Uses dependency injection to decouple Service from HAL.
     *     - Does NOT initialize hardware (done explicitly via motor_init).
     ***************************************************************/
    MotorService(MotorHal& driver);  
    
    /***************************************************************
    * Method: motor_init
    * Description:
    *     - Initializes the underlying Motor HAL.
    *     - Abstracts hardware layer from the application.
    *     - Application layer should NOT call MotorHal directly.
    ***************************************************************/
    void motorInit();   
    
    /***************************************************************
    * Method: setTargetSpeed
    * Description:
    *     - Sets motor speed and direction.
    *     - Input range: [-1.0, 1.0]
    *         > positive  → FORWARD
    *         > negative  → BACKWARD
    *         > zero      → STOP
    *     - Applies safety clamp to limit maximum speed.
    *     - Converts signed speed into:
    *         → direction (MotorState)
    *         → duty cycle (absolute value)
    *
    * Parameters:
    *     speed (float): Desired motor speed [-1.0 to 1.0]
    ***************************************************************/
    void setTargetSpeed(float speed);  // -1.0 -> +1.0

    /***************************************************************
    * Method: stop
    * Description:
    *     - Stops the motor immediately.
    *     - Resets duty cycle to zero.
    *     - Calls HAL stop to disable PWM output.
    ***************************************************************/
    void stop();

    /***************************************************************
    * Method: getCurrentDuty
    * Description:
    *     - Returns the current duty cycle applied to the motor.
    *     - Range: [0.0, 1.0]
    *
    * Returns:
    *     float: Current duty cycle
    ***************************************************************/
    float getCurrentDuty() const;
    
private:
    MotorHal& _driver;
    float _current_duty = 0.0f;
};

#endif // MOTOR_SERVICE_HPP