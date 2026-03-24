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

// TODO: Tune these conversion factors later
// Roughly 350 ticks per 20cm of travel -> 17.5 ticks/cm
const float CM_TO_TICKS = 17.5f;
const float ML_CM_TO_TICKS = CM_TO_TICKS;
const float MR_CM_TO_TICKS = CM_TO_TICKS;
const float RC_CM_TO_TICKS = CM_TO_TICKS;
const float CARRIAGE_CM_TO_TICKS = CM_TO_TICKS;
const float FC_MAX_TICKS = 0.0f; // Hardcoded top-of-range for front caster

const float DG = 180.0f / 3.14159f;

#endif
