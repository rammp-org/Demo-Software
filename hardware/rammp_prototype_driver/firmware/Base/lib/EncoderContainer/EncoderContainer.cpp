#include <EncoderContainer.h>

void EncoderContainer::retrieve_readings() {
  // Retrieve current encoder counters (motor joint, drive wheel carriage and
  // speed)
  encoder[1] = Enc2.read();  // RC top
  encoder[2] = Enc4.read();  // FC bottom
  encoder[3] = Enc1.read();  // RC bottom 0-850
  encoder[4] = Enc3.read();  // FC top
  encoder[5] = Enc12.read(); // MR back
  encoder[6] = Enc6.read();  // ML front
  encoder[7] = Enc11.read(); // ML back
  encoder[8] = Enc10.read(); // MR front
  encoder[9] = Enc5.read();  // ML drive wheel
  encoder[10] = Enc8.read(); // MR drive wheel
  encoder[11] = Enc7.read(); // ML carriage
  encoder[12] = Enc9.read(); // MR carriage
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
};
