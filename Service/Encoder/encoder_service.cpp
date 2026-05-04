/***************************************************************
 *  File: encoder_service.cpp
 *  Layer: Service Layer
 *  Description:
 *      - Converts raw encoder ticks from HAL into meaningful physical
 *        values such as RPM, linear speed (cm/s), distance traveled, and rotations.
 *      - Uses a periodic timer (100ms interval) to continuously update these values.
 *      - Provides a higher-level API for application layer access.
 ****************************************************************/
#include <cmath>
#include "encoder_service.hpp"

// ---------------------------
// Constructor
// ---------------------------
// Initializes the service layer object.
// - Takes reference to EncoderHAL object (dependency injection).
// - Initializes internal variables for tick counting, RPM, speed, distance.
EncoderService::EncoderService(EncoderHAL& encoder)
    : _encoder(encoder),
      _lastTicks(0), _currentTicks(0),
      _rpm(0), _speedCmS(0), _distanceCm(0){}



/***************************************************************
 * Method: encoderInit
 * Description:
 *     - Initializes the underlying HAL encoder.
 *     - This abstracts the HAL layer from the Application layer.
 *     - Application should ONLY call this function instead of
 *       directly calling HAL functions.
 ***************************************************************/
void EncoderService::encoderInit()
{
    _encoder.encoderInit();
}

// ---------------------------
// Method: encoderStart
// ---------------------------
// Description:
//     - Starts a repeating timer with 100ms period.
//     - Timer periodically calls static timerCallback function, which
//       in turn updates RPM, speed, distance, and rotations.
// Notes:
//     - Uses pico-sdk's add_repeating_timer_ms API for scheduling periodic updates.
void EncoderService::encoderStart() {
    add_repeating_timer_ms(100, EncoderService::timerCallback, this, &_timer);
}

// ---------------------------
// Static timer callback
// ---------------------------
// Description:
//     - Static method required by pico-sdk timer API.
//     - Casts user_data pointer to EncoderService* and calls update().
//     - Returns true to keep the timer repeating.
bool EncoderService::timerCallback(struct repeating_timer* t) {
    EncoderService* service = static_cast<EncoderService*>(t->user_data);
    service->update();
    return true;
}

// ---------------------------
// Method: update
// ---------------------------
// Description:
//     - Reads current tick count from HAL.
//     - Calculates change in ticks since last update.
//     - Converts ticks into physical values:
//         1. RPM: Revolutions per minute
//         2. Distance (cm) traveled based on wheel radius and encoder CPR
//         3. Linear speed (cm/s) based on distance traveled over 100ms
//     - Updates lastTicks for next iteration.
void EncoderService::update() {
    _currentTicks = _encoder.encoderGetTicks();   
    int32_t delta = _currentTicks - _lastTicks;

    // RPM calculation: delta ticks over 0.1s interval -> RPM
    _rpm = (delta * 600.0f) / ENCODER_CPR;

    // Distance calculation in cm
    _distanceCm += (delta / static_cast<float>(ENCODER_CPR)) * 2.0f * 3.1415926f * WHEEL_RADIUS_CM;

    // Linear speed calculation in cm/s over 100ms
    _speedCmS = (delta / static_cast<float>(ENCODER_CPR)) * 2.0f * 3.1415926f * WHEEL_RADIUS_CM / 0.1f;

    _lastTicks = _currentTicks;
}

// ---------------------------
// Getter Methods
// ---------------------------
// These methods provide read-only access to computed values.
// Marked const to indicate they do not modify object state.

float EncoderService::encoderGetRPM() const { return _rpm; }
float EncoderService::encoderGetSpeedCmS() const { return _speedCmS; }
float EncoderService::encoderGetDistanceCm() const { return _distanceCm; }

// Returns total number of rotations calculated from ticks
float EncoderService::encoderGetRotations() const { return _currentTicks / static_cast<float>(ENCODER_CPR); }
