#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <RoboClaw.h>

// RoboClaw Objects
extern RoboClaw roboclaw_casters;
extern RoboClaw roboclaw_main;
extern RoboClaw roboclaw_carriages;

// GLOBAL CONSTANTS
//----------------------------------------
// #define FC_DIR_PIN 30
// #define FC_PWM_PIN 29
#define FC_LOADCELL_PIN A17 // already correct

// #define ML_DIR_PIN 11
// #define ML_PWM_PIN 10
#define ML_LOADCELL_PIN A15 // A15

// #define MR_DIR_PIN 8
// #define MR_PWM_PIN 9
#define MR_LOADCELL_PIN A14 // A14

// #define RC_DIR_PIN 6
// #define RC_PWM_PIN 7
#define RC_LOADCELL_PIN A16 // A16

// #define ML_CARRIAGE_DIR_PIN 4
// #define ML_CARRIAGE_PWM_PIN 5

// #define MR_CARRIAGE_DIR_PIN 3
// #define MR_CARRIAGE_PWM_PIN 2

#define CARRIAGE_SW1_PIN 23 // ML carriage forward limit switch
#define CARRIAGE_SW2_PIN 22 // ML carriage backward limit switch
#define CARRIAGE_SW3_PIN 13 // MR carriage forward limit switch
#define CARRIAGE_SW4_PIN 33 // MR carriage backward limit switch

// Omni2 variables for SL and CC applications
// variable names need to be changed with schematics later
// TODO: remove
#define OMNI2_PROFILE_PIN -1
#define OMNI2_FORWARD_PIN -1
#define OMNI2_LEFT_PIN -1
#define OMNI2_RIGHT_PIN -1
#define OMNI2_REVERSE_PIN -1

// Joystick Input/Outputs
// TODO: remove
#define JS_SD_PIN -1
#define JS_PFILE_PIN -1
#define JS_SUP_PIN -1
#define JS_X_PIN -1
#define JS_Y_PIN -1
#define JS_SW_PIN -1

#define BNO055_SAMPLERATE_DELAY_MS 10
#define PI_MOTORS Serial6
#define PI_BT Serial1

// Variables needed to enable auto_curb_climb
#define NUM_READINGS                                                           \
  10 // sample window is 10, use define instead of int to save memory
#define PACKET_LENGTH 16
#define ANGLE_THRESHOLD 5.0 // Adjust this threshold as needed
#define TEST_INPUT_CURB_150CM_L45D "0.211.12026.210071.211"
// #define TEST_INPUT_CURB_150CM_L45D "0.211.40020.840065.841"

#define JOY_X_DELTA -1
#define RIGHT_TURN_JOY_X -1
#define RIGHT_TURN_JOY_Y -1
#define SLOW_RIGHT_TURN_JOY_X -1

#define LEFT_TURN_JOY_X -1
#define LEFT_TURN_JOY_Y -1
#define SLOW_LEFT_TURN_JOY_X -1

const float DG = 180 / 3.14;
#define NO_PIN -1

#endif
