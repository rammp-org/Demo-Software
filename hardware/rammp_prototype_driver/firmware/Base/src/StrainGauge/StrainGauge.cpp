#include "StrainGauge.h"

StrainGauge::StrainGauge(int pin, float lpf_alpha)
    : pin(pin),
      lpf_alpha(lpf_alpha),
      _filtered_value(0.0f)
{
    pinMode(pin, INPUT);
}

void StrainGauge::update(float dt) {
    // dt is accepted for API consistency but is unused at this time.
    // It is available for future derivative / rate-of-change computations.
    (void)dt;

    float raw = (float)analogRead(pin);

    // First-order IIR low-pass filter — same pattern as Motor::updateSensorData()
    // and EncoderContainer::retrieve_readings():
    //   output = output + alpha * (input - output)
    _filtered_value += lpf_alpha * (raw - _filtered_value);
}

float StrainGauge::getValue() const {
    return _filtered_value;
}

void StrainGauge::setLpfAlpha(float alpha) {
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    lpf_alpha = alpha;
}

float StrainGauge::getLpfAlpha() const {
    return lpf_alpha;
}
