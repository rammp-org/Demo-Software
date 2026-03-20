#include "EncoderContainer.h"

void EncoderContainer::retrieve_readings() {
  // Retrieve current encoder counters (motor joint, drive wheel carriage and
  // speed) - subtract offsets for zeroing
  encoder[1] = Enc2.read() - encoder_offset[1];   // RC top
  encoder[2] = Enc4.read() - encoder_offset[2];   // FC bottom
  encoder[3] = Enc1.read() - encoder_offset[3];   // RC bottom 0-850
  encoder[4] = Enc3.read() - encoder_offset[4];   // FC top
  encoder[5] = Enc12.read() - encoder_offset[5];  // MR back
  encoder[6] = Enc6.read() - encoder_offset[6];   // ML front
  encoder[7] = Enc11.read() - encoder_offset[7];  // ML back
  encoder[8] = Enc10.read() - encoder_offset[8];  // MR front
  encoder[9] = Enc5.read() - encoder_offset[9];   // ML drive wheel
  encoder[10] = Enc8.read() - encoder_offset[10]; // MR drive wheel
  encoder[11] = Enc7.read() - encoder_offset[11]; // ML carriage
  encoder[12] = Enc9.read() - encoder_offset[12]; // MR carriage
  encoderf[1] = encoderf[1] + K_sensors * (encoder[1] - encoderf[1]);
  encoderf[2] = encoderf[2] + K_sensors * (encoder[2] - encoderf[2]);
  encoderf[3] = encoderf[3] + K_sensors * (encoder[3] - encoderf[3]);
  encoderf[4] = encoderf[4] + K_sensors * (encoder[4] - encoderf[4]);
  encoderf[5] = encoderf[5] + K_sensors * (encoder[5] - encoderf[5]);
  // encoderf[6] = encoderf[6] + K_sensors * (encoder[6] - encoderf[6]); // Not
  // used
  encoderf[7] = encoderf[7] + K_sensors * (encoder[7] - encoderf[7]);
  // encoderf[8] = encoderf[8] + K_sensors * (encoder[8] - encoderf[8]); // Not
  // used
  encoderf[9] = encoderf[9] + K_sensors * (encoder[9] - encoderf[9]);
  encoderf[10] = encoderf[10] + K_sensors * (encoder[10] - encoderf[10]);
  encoderf[11] = encoderf[11] + K_sensors * (encoder[11] - encoderf[11]);
  encoderf[12] = encoderf[12] + K_sensors * (encoder[12] - encoderf[12]);
}

void EncoderContainer::zeroEncoder(int index) {
  // Zero encoder by setting offset to current raw reading
  if (index >= 1 && index <= 12) {
    switch (index) {
      case 1: encoder_offset[index] = Enc2.read(); break;
      case 2: encoder_offset[index] = Enc4.read(); break;
      case 3: encoder_offset[index] = Enc1.read(); break;
      case 4: encoder_offset[index] = Enc3.read(); break;
      case 5: encoder_offset[index] = Enc12.read(); break;
      case 6: encoder_offset[index] = Enc6.read(); break;
      case 7: encoder_offset[index] = Enc11.read(); break;
      case 8: encoder_offset[index] = Enc10.read(); break;
      case 9: encoder_offset[index] = Enc5.read(); break;
      case 10: encoder_offset[index] = Enc8.read(); break;
      case 11: encoder_offset[index] = Enc7.read(); break;
      case 12: encoder_offset[index] = Enc9.read(); break;
    }
    // Also reset filtered value to prevent jumps
    encoderf[index] = 0;
  }
}

void EncoderContainer::setOffset(int index, signed long offset) {
  if (index >= 1 && index <= 12) {
    encoder_offset[index] = offset;
    // Immediately update the filtered value to match the new logical position
    encoderf[index] = (signed long)(getRawReading(index) - encoder_offset[index]);
  }
}

signed long EncoderContainer::getRawReading(int index) {
  switch (index) {
    case 1: return Enc2.read();
    case 2: return Enc4.read();
    case 3: return Enc1.read();
    case 4: return Enc3.read();
    case 5: return Enc12.read();
    case 6: return Enc6.read();
    case 7: return Enc11.read();
    case 8: return Enc10.read();
    case 9: return Enc5.read();
    case 10: return Enc8.read();
    case 11: return Enc7.read();
    case 12: return Enc9.read();
    default: return 0;
  }
}
