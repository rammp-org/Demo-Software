#ifndef STRAIN_GAUGE_H
#define STRAIN_GAUGE_H

#include <Arduino.h>

/**
 * StrainGauge
 *
 * Wraps a single analog load-cell / strain-gauge channel.
 * Performs an analogRead() every update() call and smooths the
 * result with a first-order IIR low-pass filter — the same
 * pattern used by EncoderContainer and Motor::updateSensorData().
 *
 * Usage (in Base.ino):
 *   StrainGauge sg_fc(FC_LOADCELL_PIN);   // default alpha = 0.5
 *   // inside loop():
 *   sg_fc.update(dt);
 *   float val = sg_fc.getValue();
 */
class StrainGauge {
public:
  /**
   * @param pin       Teensy analog pin number (e.g. A17)
   * @param lpf_alpha IIR coefficient [0, 1].
   *                  1.0 = no filter (pass-through).
   *                  0.5 = moderate smoothing (default).
   *                  Lower values = more smoothing, more lag.
   */
  StrainGauge(int pin, float lpf_alpha = 0.5f);

  /**
   * Read the ADC and update the filtered value.
   * Call once per control loop cycle.
   * dt is accepted for API consistency with other sensor classes
   * and is reserved for future derivative / rate computations.
   */
  void update(float dt);

  /** Return the current filtered ADC reading. */
  float getValue() const;

  /** Set the IIR filter coefficient. Clamped to [0, 1]. */
  void setLpfAlpha(float alpha);

  /** Return the current IIR filter coefficient. */
  float getLpfAlpha() const;

  // Public members (consistent with Motor / PIDController style)
  int pin;
  float lpf_alpha;
  float _filtered_value;
};

#endif // STRAIN_GAUGE_H
