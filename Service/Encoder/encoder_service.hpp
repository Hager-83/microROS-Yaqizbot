 
#ifndef ENCODER_SERVICE_HPP
#define ENCODER_SERVICE_HPP

#include "encoder_hal.hpp"
#include "hardware/timer.h"

/***************************************************************
 * Class: EncoderService
 * Layer: Service Layer
 * Description:
 *     - Provides high-level interface to compute physical metrics
 *       from raw encoder ticks: RPM, linear speed, distance, rotations.
 *     - Runs a repeating timer (100ms) to update these metrics automatically.
 *     - Uses composition: holds a reference to HAL object.
 ****************************************************************/

class EncoderService {
public:
    /***********************************************************
     * Constructor: EncoderService
     * Parameters:
     *     - encoder: reference to EncoderHAL object
     * Description:
     *     - Initializes internal state variables and stores HAL reference.
     ***********************************************************/
    EncoderService(EncoderHAL& encoder);

    void encoderInit();

    /***********************************************************
     * Method: encoderStart
     * Description:
     *     - Starts a periodic repeating timer (every 100ms) that updates
     *       the RPM, speed, distance, and rotations values.
     ***********************************************************/
    void encoderStart();

    /***********************************************************
     * Method: encoderGetRPM
     * Description:
     *     - Returns the most recent computed RPM value.
     *     - Const method; does not modify internal state.
     ***********************************************************/
    float encoderGetRPM() const;

    /***********************************************************
     * Method: encoderGetSpeedCmS
     * Description:
     *     - Returns the most recent computed linear speed in cm/s.
     *     - Const method; does not modify internal state.
     ***********************************************************/
    float encoderGetSpeedCmS() const;

    /***********************************************************
     * Method: encoderGetDistanceCm
     * Description:
     *     - Returns the accumulated distance in cm since start or reset.
     *     - Const method; does not modify internal state.
     ***********************************************************/
    float encoderGetDistanceCm() const;

    /***********************************************************
     * Method: encoderGetRotations
     * Description:
     *     - Returns the total number of rotations completed since start.
     *     - Const method; does not modify internal state.
     ***********************************************************/
    float encoderGetRotations() const;


    /***********************************************************
     * Method: encoderGetPositionRad
     * Description:
     *     - Returns the current wheel angular position in radians
     *       computed from encoder ticks.
     *     - Const method; does not modify internal state.
     ***********************************************************/
    float encoderGetPositionRad() const;

private:
    /***********************************************************
     * Static Timer Callback: timerCallback
     * Description:
     *     - Called by hardware repeating timer every 100ms.
     *     - Delegates update to actual object instance using user_data pointer.
     ***********************************************************/
    static bool timerCallback(struct repeating_timer* t);

    /***********************************************************
     * Method: update
     * Description:
     *     - Computes delta ticks since last update.
     *     - Updates RPM, distance, speed based on CPR and wheel radius.
     *     - Updates internal tick tracking variables.
     ***********************************************************/
    void update();

    EncoderHAL& _encoder;                  // Reference to HAL object
    int32_t _lastTicks;                    // Tick count from previous timer cycle
    int32_t _currentTicks;                 // Tick count for current timer cycle
    float _rpm;                            // Computed RPM
    float _speedCmS;                       // Computed linear speed in cm/s
    float _distanceCm;                     // Computed total distance
    struct repeating_timer _timer;         // Hardware timer structure
};


#endif
