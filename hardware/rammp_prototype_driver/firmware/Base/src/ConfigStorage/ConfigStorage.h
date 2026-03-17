#ifndef CONFIG_STORAGE_H
#define CONFIG_STORAGE_H

#include <Arduino.h>
#include <EEPROM.h>

/**
 * ConfigStorage - EEPROM storage for motor configuration
 * 
 * EEPROM memory map:
 * Address 0-5: Motor directions (6 motors, 1 byte each: 1 = normal, -1 = inverted)
 * Address 6-7: Magic number for validity check (0xABCD)
 * Address 100+: Reserved for future use
 */
class ConfigStorage {
public:
    static const uint16_t MAGIC_NUMBER = 0xABCD;
    static const int MAGIC_ADDR = 6;
    static const int DIR_START_ADDR = 0;
    static const int NUM_MOTORS = 6;
    
    // Initialize storage, check validity
    static void begin();
    
    // Save/load single motor direction
    static void saveMotorDirection(int motor_id, int8_t direction);
    static int8_t loadMotorDirection(int motor_id);
    
    // Save/load all motor directions at once
    static void saveAllDirections(int8_t* dirs);
    static void loadAllDirections(int8_t* dirs);
    
    // Check if EEPROM has valid data
    static bool isValid();
    
    // Initialize defaults (called on first boot or corruption)
    static void initializeDefaults();
};

#endif
