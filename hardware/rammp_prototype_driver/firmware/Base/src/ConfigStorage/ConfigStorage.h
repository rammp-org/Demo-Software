#ifndef CONFIG_STORAGE_H
#define CONFIG_STORAGE_H

#include <Arduino.h>
#include <EEPROM.h>

struct MotorConfig {
  int8_t motor_dir;
  int8_t encoder_dir;
  float lpf_input_alpha;
  float pos_p, pos_i, pos_d, pos_ff;
  float pos_lpf_alpha;
  float pos_max_ramp_rate;
  float vel_p, vel_i, vel_d, vel_ff;
  float vel_lpf_alpha;
  float vel_max_ramp_rate;
  float saved_position;
  int32_t pos_limit_min;
  int32_t pos_limit_max;
};

/**
 * ConfigStorage - EEPROM storage for motor configuration
 *
 * EEPROM memory map:
 * Address 0-1: Magic number for validity check (0xABD0)
 * Address 10+: Array of 6 MotorConfig structs
 */
class ConfigStorage {
public:
  static const uint16_t MAGIC_NUMBER =
      0xABD0; // Bumped: added max_ramp_rate fields
  static const int MAGIC_ADDR = 0;
  static const int CONFIG_START_ADDR = 10;
  static const int NUM_MOTORS = 6;

  // Initialize storage, check validity
  static void begin();

  // Save/load single motor config
  static void saveMotorConfig(int motor_id, const MotorConfig &config);
  static MotorConfig loadMotorConfig(int motor_id);

  // Helper to just save position
  static void save_position(int motor_id, float position);

  // Check if EEPROM has valid data
  static bool isValid();

  // Initialize defaults (called on first boot or corruption)
  static void initializeDefaults();
};

#endif
