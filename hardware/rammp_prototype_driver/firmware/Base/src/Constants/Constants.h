#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "../RoboClaw/RoboClaw.h"

// RoboClaw Objects
extern RoboClaw roboclaw_casters;
extern RoboClaw roboclaw_main;
extern RoboClaw roboclaw_carriages;

#define FC_LOADCELL_PIN A17
#define ML_LOADCELL_PIN A15
#define MR_LOADCELL_PIN A14
#define RC_LOADCELL_PIN A16

#define CARRIAGE_SW1_PIN 23 // ML carriage forward limit switch
#define CARRIAGE_SW2_PIN 22 // ML carriage backward limit switch
#define CARRIAGE_SW3_PIN 13 // MR carriage forward limit switch
#define CARRIAGE_SW4_PIN 33 // MR carriage backward limit switch

const float DG = 180 / 3.14;

#endif
