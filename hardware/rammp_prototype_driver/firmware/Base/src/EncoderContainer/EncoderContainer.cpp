#include "EncoderContainer.h"

// Logical indices 1–12 must stay consistent with motor_map in Base.ino.
// Physical Encoder objects and pins are defined only in EncoderContainer.h.

void EncoderContainer::retrieve_readings() {
  encoder[3] = Enc1.read() - encoder_offset[3]; // RC bottom → joint rc (enc 3)
  encoder[1] =
      encoder[3]; // slot 1 not in motor_map; old RC-top (Enc2) pins repurposed
  encoder[2] = Enc6.read() - encoder_offset[2]; // FC bottom → joint fc
  encoder[4] = 0; // FC top (Enc3) removed per EncoderContainer.h — slot unused
  encoder[5] = Enc12.read() - encoder_offset[5]; // MR back → joint mr
  encoder[6] = 0; // no sensor; legacy “ML front” slot unused by motor_map
  encoder[7] = Enc11.read() - encoder_offset[7]; // ML back → joint ml
  encoder[8] = 0; // MR-front aux removed (Enc9/Enc10 pins → carriage / odrive)
  encoder[9] = Enc5.read() - encoder_offset[9];    // ML drive wheel
  encoder[10] = Enc8.read() - encoder_offset[10];  // MR drive wheel
  encoder[11] = Enc7.read() - encoder_offset[11];  // ML carriage (pins 24,12)
  encoder[12] = Enc10.read() - encoder_offset[12]; // MR carriage (pins 11,10)

  encoderf[1] = encoderf[1] + K_sensors * (encoder[1] - encoderf[1]);
  encoderf[2] = encoderf[2] + K_sensors * (encoder[2] - encoderf[2]);
  encoderf[3] = encoderf[3] + K_sensors * (encoder[3] - encoderf[3]);
  encoderf[4] = encoderf[4] + K_sensors * (encoder[4] - encoderf[4]);
  encoderf[5] = encoderf[5] + K_sensors * (encoder[5] - encoderf[5]);
  encoderf[6] = encoderf[6] + K_sensors * (encoder[6] - encoderf[6]);
  encoderf[7] = encoderf[7] + K_sensors * (encoder[7] - encoderf[7]);
  encoderf[8] = encoderf[8] + K_sensors * (encoder[8] - encoderf[8]);
  encoderf[9] = encoderf[9] + K_sensors * (encoder[9] - encoderf[9]);
  encoderf[10] = encoderf[10] + K_sensors * (encoder[10] - encoderf[10]);
  encoderf[11] = encoderf[11] + K_sensors * (encoder[11] - encoderf[11]);
  encoderf[12] = encoderf[12] + K_sensors * (encoder[12] - encoderf[12]);
}

void EncoderContainer::zeroEncoder(int index) {
  if (index >= 1 && index <= 12) {
    switch (index) {
    case 1:
      // Keep slot 1 aligned with slot 3 (same Enc1 / RC bottom).
      encoder_offset[3] = Enc1.read();
      encoder_offset[1] = encoder_offset[3];
      encoderf[1] = encoderf[3] = 0;
      break;
    case 2:
      encoder_offset[index] = Enc6.read();
      break;
    case 3:
      encoder_offset[index] = Enc1.read();
      encoder_offset[1] = encoder_offset[3];
      encoderf[1] = encoderf[3] = 0;
      break;
    case 4:
      encoder_offset[index] = 0;
      encoderf[index] = 0;
      break;
    case 5:
      encoder_offset[index] = Enc12.read();
      break;
    case 6:
      encoder_offset[index] = 0;
      encoderf[index] = 0;
      break;
    case 7:
      encoder_offset[index] = Enc11.read();
      break;
    case 8:
      encoder_offset[index] = 0;
      encoderf[index] = 0;
      break;
    case 9:
      encoder_offset[index] = Enc5.read();
      break;
    case 10:
      encoder_offset[index] = Enc8.read();
      break;
    case 11:
      encoder_offset[index] = Enc7.read();
      break;
    case 12:
      encoder_offset[index] = Enc10.read();
      break;
    }
    if (index != 1 && index != 3)
      encoderf[index] = 0;
  }
}

void EncoderContainer::setOffset(int index, signed long offset) {
  if (index >= 1 && index <= 12) {
    encoder_offset[index] = offset;
    encoderf[index] =
        (signed long)(getRawReading(index) - encoder_offset[index]);
    if (index == 3) {
      encoder_offset[1] = encoder_offset[3];
      encoderf[1] = encoderf[3];
    }
    if (index == 1) {
      encoder_offset[3] = encoder_offset[1];
      encoderf[3] = encoderf[1];
    }
  }
}

signed long EncoderContainer::getRawReading(int index) {
  switch (index) {
  case 1:
  case 3:
    return Enc1.read();
  case 2:
    return Enc6.read();
  case 4:
    return 0;
  case 5:
    return Enc12.read();
  case 6:
    return 0;
  case 7:
    return Enc11.read();
  case 8:
    return 0;
  case 9:
    return Enc5.read();
  case 10:
    return Enc8.read();
  case 11:
    return Enc7.read();
  case 12:
    return Enc10.read();
  default:
    return 0;
  }
}
