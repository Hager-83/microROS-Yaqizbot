#ifndef ENCODER_HAL_HPP
#define ENCODER_HAL_HPP

#include "pico/stdlib.h"
#include "encoder_config.hpp"

/***************************************************************
 * Enum: EncoderDirection
 * Description:
 *     - Represents detected rotation direction.
 ****************************************************************/
enum class EncoderDirection { UNKNOWN, FORWARD, BACKWARD };

/***************************************************************
 * Class: EncoderHAL
 * Layer: HAL (Hardware Abstraction Layer)
 * Description:
 *     - Low-level interface for a quadrature encoder.
 *     - Supports multiple encoder instances.
 *     - Handles GPIO initialization, interrupts, and tick counting.
 ****************************************************************/
class EncoderHAL {
public:
    /***********************************************************
     * Constructor: EncoderHAL
     * Parameters:
     *     - pinA: GPIO pin connected to encoder channel A
     *     - pinB: GPIO pin connected to encoder channel B
     * Description:
     *     - Initializes internal variables.
     *     - Registers this instance for ISR handling.
     ***********************************************************/
    EncoderHAL(uint pinA, uint pinB);

    /***********************************************************
     * Method: encoder_init
     * Description:
     *     - Initializes GPIO pins and enables interrupts.
     ***********************************************************/
    void encoderInit();

    /***********************************************************
     * Method: encoder_getTicks
     * Description:
     *     - Returns current encoder tick count.
     ***********************************************************/
    int32_t encoderGetTicks() const;

    /***********************************************************
     * Method: encoder_clear
     * Description:
     *     - Resets tick counter and direction.
     ***********************************************************/
    void encoderClear();

    /***********************************************************
     * Method: encoder_getDirection
     * Description:
     *     - Returns last detected rotation direction.
     ***********************************************************/
    EncoderDirection encoderGetDirection() const;

private:
    /***********************************************************
     * Static ISR Callback: encoder_gpioCallback
     * Description:
     *     - Called on GPIO interrupt.
     *     - Dispatches interrupt to the correct encoder instance.
     ***********************************************************/
    static void encoderGpioCallback(uint gpio, uint32_t events);

    /***********************************************************
     * Method: handleEncoder
     * Description:
     *     - Processes quadrature logic.
     *     - Updates tick count and direction.
     ***********************************************************/
    void handleEncoder();

    uint _pinA, _pinB;                      // Encoder GPIO pins
    volatile int32_t _ticks;                // Tick counter
    volatile EncoderDirection _direction;   // Rotation direction
    volatile bool _lastA, _lastB;           // Previous pin states

    // -------- Static instance registry --------
    static EncoderHAL* instances[4];        // List of encoder instances
    static int instanceCount;               // Number of registered encoders
};

#endif // ENCODER_HAL_HPP
