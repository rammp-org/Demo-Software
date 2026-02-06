#ifndef ENCODER_CONTAINER_H
#define ENCODER_CONTAINER_H

#include <Encoder.h>

class EncoderContainer
{
public:
    // Position sensors pin selection
    float K_sensors = 0.1;
    Encoder Enc1 = Encoder(38, 39);  // RC bottom
    Encoder Enc2 = Encoder(36, 37);  // RC top
    Encoder Enc3 = Encoder(33, 34);  // FC top
    Encoder Enc4 = Encoder(53, 40);  // FC bottom
    Encoder Enc5 = Encoder(52, 41);  // ML drive wheel
    Encoder Enc6 = Encoder(57, 49);  // ML front
    Encoder Enc7 = Encoder(54, 45);  // ML carriage
    Encoder Enc8 = Encoder(56, 55);  // MR drive wheel
    Encoder Enc9 = Encoder(44, 46);  // MR carriage
    Encoder Enc10 = Encoder(43, 50); // MR front
    Encoder Enc11 = Encoder(42, 51); // ML back
    Encoder Enc12 = Encoder(27, 26); // MR back
    signed long encoder[13];         // = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    signed long encoderf[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    void retrieve_readings();
};

#endif