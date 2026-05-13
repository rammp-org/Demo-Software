#ifndef ENCODER_CONTAINER_H
#define ENCODER_CONTAINER_H

#include <Encoder.h>

class EncoderContainer {
public:
  // Quadrature encoder pin pairs. Logical indices 1–12 → joints: see
  // EncoderContainer.cpp (must match motor_map in Base.ino).
  float K_sensors = 1;
  // Encoder Enc2 = Encoder(1, 0);    // RC top — pins repurposed (e.g. ODrive)
  Encoder Enc1 = Encoder(3, 2); // RC bottom          (3,2)
  // Encoder Enc3 = Encoder(5, 4);    // FC top — not populated on current build
  Encoder Enc6 = Encoder(7, 6);    // FC bottom         (7,6)
  Encoder Enc5 = Encoder(9, 8);    // ML drive wheel    (9,8)
  Encoder Enc10 = Encoder(11, 10); // MR carriage         (11,10)
  Encoder Enc7 = Encoder(24, 12);  // ML carriage       (24,12)
  Encoder Enc8 = Encoder(26, 25);  // MR drive wheel    (26,25)
  Encoder Enc11 = Encoder(32, 31); // ML back           (32,31)
  Encoder Enc12 = Encoder(36, 37); // MR back           (36,37)
  signed long encoder[13]; // = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  signed long encoderf[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  // Encoder offsets for zeroing (RAM only, resets on power cycle)
  signed long encoder_offset[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  void retrieve_readings();
  void zeroEncoder(int index);                   // Zero single encoder
  void setOffset(int index, signed long offset); // Set custom offset
  signed long getRawReading(int index);          // Get raw value before offset
};

#endif
