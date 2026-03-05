#ifndef ENCODER_CONTAINER_H
#define ENCODER_CONTAINER_H

#include <Encoder.h>

class EncoderContainer
{
public:
    // Position sensors pin selection
    float K_sensors = 0.1;
    Encoder Enc1 = Encoder(1, 0);  // RC bottom         (1,0)
    Encoder Enc2 = Encoder(3, 2);  // RC top            (3,2)
    Encoder Enc3 = Encoder(5, 4);  // FC top            (5,4)
    Encoder Enc4 = Encoder(7, 6);  // FC bottom         (7,6)
    Encoder Enc5 = Encoder(9, 8);  // ML drive wheel    (9,8)
    Encoder Enc6 = Encoder(11, 10);  // ML front          (11,10)
    Encoder Enc7 = Encoder(24, 12);  // ML carriage       (24,12)
    Encoder Enc8 = Encoder(26, 25);  // MR drive wheel    (26,25)
    Encoder Enc9 = Encoder(28, 27);  // MR carriage       (28,27)
    Encoder Enc10 = Encoder(30, 29); // MR front          (30,29)
    Encoder Enc11 = Encoder(32, 31); // ML back           (32,31)
    Encoder Enc12 = Encoder(36, 37); // MR back           (36,37)
    signed long encoder[13];         // = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    signed long encoderf[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    void retrieve_readings();
};

#endif