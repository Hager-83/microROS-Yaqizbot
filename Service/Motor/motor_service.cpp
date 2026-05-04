#include <cmath>
#include "motor_service.hpp"


MotorService::MotorService(MotorHal& driver)
    : _driver(driver) {}


void MotorService::motorInit()
{
    _driver.motorInit();
    _driver.stop();      // Safety: ensure motor is stopped after init
}


void MotorService::setTargetSpeed(float speed) {
    
    // MAXIMUM SPEED (80%)
    const float MAX_SPEED = 0.8f;
    
    // Clamp speed to safe limits
    if (speed > MAX_SPEED)
    { 
        speed = MAX_SPEED;
    }else{}

    if (speed < -MAX_SPEED) 
    {
        speed = -MAX_SPEED;
    }else{}

    // Store absolute duty cycle
    _current_duty = std::fabs(speed);

    // Determine direction and apply output
    if (speed > 0) 
    {
        _driver.setMotorOutput(MotorState::FORWARD, speed);
    } 
    else if (speed < 0) 
    {
        _driver.setMotorOutput(MotorState::BACKWARD, -speed);
    } 
    else 
    {
        stop();
    }
}


void MotorService::stop() {
    _current_duty = 0.0f;
    _driver.stop();
}


float MotorService::getCurrentDuty() const {
    return _current_duty;
}