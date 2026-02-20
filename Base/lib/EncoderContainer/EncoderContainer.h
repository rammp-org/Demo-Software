#ifndef ENCODER_CONTAINER_H
#define ENCODER_CONTAINER_H

#include <Encoder.h>

class EncoderContainer
{
public:
    // Position sensors pin selection
    float K_sensors = 0.1;
    Encoder Enc1 = Encoder(38, 39);  // RC bottom         (1,0)
    Encoder Enc2 = Encoder(36, 37);  // RC top            (3,2)
    Encoder Enc3 = Encoder(33, 34);  // FC top            (5,4)
    Encoder Enc4 = Encoder(53, 40);  // FC bottom         (7,6)
    Encoder Enc5 = Encoder(52, 41);  // ML drive wheel    (9,8)
    Encoder Enc6 = Encoder(57, 49);  // ML front          (11,10)
    Encoder Enc7 = Encoder(54, 45);  // ML carriage       (24,12)
    Encoder Enc8 = Encoder(56, 55);  // MR drive wheel    (26,25)
    Encoder Enc9 = Encoder(44, 46);  // MR carriage       (28,27)
    Encoder Enc10 = Encoder(43, 50); // MR front          (30,29)
    Encoder Enc11 = Encoder(42, 51); // ML back           (32,31)
    Encoder Enc12 = Encoder(27, 26); // MR back           (36,37)
    signed long encoder[13];         // = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    signed long encoderf[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    void retrieve_readings();
};

#endif