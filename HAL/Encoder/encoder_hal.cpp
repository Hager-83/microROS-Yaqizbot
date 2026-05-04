#include "encoder_hal.hpp"


/***************************************************************
 * Static Members Initialization
 ***************************************************************/
EncoderHAL* EncoderHAL::instances[4] = {nullptr, nullptr, nullptr, nullptr};
int EncoderHAL::instanceCount = 0;

/***************************************************************
 * Constructor
 ***************************************************************/
EncoderHAL::EncoderHAL(uint pinA, uint pinB)
    : _pinA(pinA), _pinB(pinB),
      _ticks(0), _direction(EncoderDirection::UNKNOWN),
      _lastA(false), _lastB(false)
{
    // Register this encoder instance for ISR handling
    if (instanceCount < 4)
    instances[instanceCount++] = this;
}

/***************************************************************
 * Method: encoder_init
 ***************************************************************/
void EncoderHAL::encoderInit() {
    gpio_init(_pinA);
    gpio_init(_pinB);

    gpio_set_dir(_pinA, GPIO_IN);
    gpio_set_dir(_pinB, GPIO_IN);

    gpio_pull_up(_pinA);
    gpio_pull_up(_pinB);

    _lastA = gpio_get(_pinA);
    _lastB = gpio_get(_pinB);

    // Enable interrupts on both edges
    gpio_set_irq_enabled_with_callback(
        _pinA,
        GPIO_IRQ_EDGE_RISE,
        true,
        &EncoderHAL::encoderGpioCallback
    );

    gpio_set_irq_enabled(
        _pinB,
        GPIO_IRQ_EDGE_RISE,
        true
    );
}

/***************************************************************
 * Static ISR Callback
 ****************************************************************/
void EncoderHAL::encoderGpioCallback(uint gpio, uint32_t events) {
    // Dispatch interrupt to the correct encoder instance
    for (int i = 0; i < instanceCount; i++) {
        if (gpio == instances[i]->_pinA || gpio == instances[i]->_pinB) {
            instances[i]->handleEncoder();
            return;
        }
    }
}

/***************************************************************
 * Method: handleEncoder
 ***************************************************************/
void EncoderHAL::handleEncoder() {
    bool a = gpio_get(_pinA);
    bool b = gpio_get(_pinB);

    if (a != _lastA || b != _lastB) {
        if (_lastA == _lastB) {
            if (a != b) { _ticks++; _direction = EncoderDirection::FORWARD;  }
            else        { _ticks--; _direction = EncoderDirection::BACKWARD; }
        } else {
            if (a == b) { _ticks++; _direction = EncoderDirection::FORWARD;  }
            else        { _ticks--; _direction = EncoderDirection::BACKWARD; }
        }
    }

    _lastA = a;
    _lastB = b;
}

/***************************************************************
 * Getter Methods
 ***************************************************************/
int32_t EncoderHAL::encoderGetTicks() const { 
    return _ticks;
}

void EncoderHAL::encoderClear() {
    _ticks = 0;
    _direction = EncoderDirection::UNKNOWN;
}

EncoderDirection EncoderHAL::encoderGetDirection() const {
    return _direction;
}

