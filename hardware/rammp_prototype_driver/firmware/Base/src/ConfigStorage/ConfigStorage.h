#ifndef CONFIG_STORAGE_H
#define CONFIG_STORAGE_H

#include <Arduino.h>
#include <EEPROM.h>

struct MotorConfig {
    int8_t motor_dir;
    int8_t encoder_dir;
    float pos_p, pos_i, pos_d, pos_ff;
    float vel_p, vel_i, vel_d, vel_ff;
};

/**
 * ConfigStorage - EEPROM storage for motor configuration
 * 
 * EEPROM memory map:
 * Address 0-1: Magic number for validity check (0xABCD)
 * Address 10+: Array of 6 MotorConfig structs
 */
class ConfigStorage {
public:
    static const uint16_t MAGIC_NUMBER = 0xABCD;
    static const int MAGIC_ADDR = 0;
    static const int CONFIG_START_ADDR = 10;
    static const int NUM_MOTORS = 6;
    
    // Initialize storage, check validity
    static void begin();
    
    // Save/load single motor config
    static void saveMotorConfig(int motor_id, const MotorConfig& config);
    static MotorConfig loadMotorConfig(int motor_id);
    
    // Check if EEPROM has valid data
    static bool isValid();
    
    // Initialize defaults (called on first boot or corruption)
    static void initializeDefaults();
};

#endif
