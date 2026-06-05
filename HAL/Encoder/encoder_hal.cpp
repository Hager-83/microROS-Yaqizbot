#include "encoder_hal.hpp"

// ---------------- static ----------------
EncoderHAL* EncoderHAL::instances[4] = {nullptr, nullptr, nullptr, nullptr};
int EncoderHAL::instanceCount = 0;

// ---------------- constructor ----------------
EncoderHAL::EncoderHAL(uint pinA, uint pinB)
    : _pinA(pinA),
      _pinB(pinB),
      _ticks(0),
      _direction(EncoderDirection::UNKNOWN),
      _lastState(0)
{
    if (instanceCount < 4)
        instances[instanceCount++] = this;
}

// ---------------- init ----------------
void EncoderHAL::encoderInit() {
    gpio_init(_pinA);
    gpio_init(_pinB);

    gpio_set_dir(_pinA, GPIO_IN);
    gpio_set_dir(_pinB, GPIO_IN);

    gpio_pull_up(_pinA);
    gpio_pull_up(_pinB);

    // encode initial state
    _lastState = (gpio_get(_pinA) << 1) | gpio_get(_pinB);

    gpio_set_irq_enabled_with_callback(
        _pinA,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &EncoderHAL::encoderGpioCallback
    );

    gpio_set_irq_enabled(
        _pinB,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );
}

// ---------------- ISR ----------------
void EncoderHAL::encoderGpioCallback(uint gpio, uint32_t events) {
    for (int i = 0; i < instanceCount; i++) {
        if (gpio == instances[i]->_pinA || gpio == instances[i]->_pinB) {
            instances[i]->handleEncoder();
            return;
        }
    }
}

// ---------------- CORE FIX ----------------
void EncoderHAL::handleEncoder() {

    uint8_t state = (gpio_get(_pinA) << 1) | gpio_get(_pinB);

    uint8_t transition = (_lastState << 2) | state;

    switch (transition) {

        // FORWARD
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            _ticks++;
            _direction = EncoderDirection::FORWARD;
            break;

        // BACKWARD
        case 0b0010:
        case 0b0100:
        case 0b1101:
        case 0b1011:
            _ticks--;
            _direction = EncoderDirection::BACKWARD;
            break;

        default:
            // ignore noise / invalid transitions
            break;
    }

    _lastState = state;
}

// ---------------- getters ----------------
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