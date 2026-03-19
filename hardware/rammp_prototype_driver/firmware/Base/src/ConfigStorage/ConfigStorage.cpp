#include "ConfigStorage.h"

// Out-of-class definitions for static const members (required for linker)
const uint16_t ConfigStorage::MAGIC_NUMBER;
const int ConfigStorage::MAGIC_ADDR;
const int ConfigStorage::CONFIG_START_ADDR;
const int ConfigStorage::NUM_MOTORS;

void ConfigStorage::begin() {
    if (!isValid()) {
        initializeDefaults();
    }
}

bool ConfigStorage::isValid() {
    uint16_t val;
    EEPROM.get(MAGIC_ADDR, val);
    return val == MAGIC_NUMBER;
}

void ConfigStorage::initializeDefaults() {
    MotorConfig def_config;
    def_config.motor_dir = 1;
    def_config.encoder_dir = 1;
    def_config.lpf_input_alpha = 0.5f;
    def_config.pos_p = 0.0f;
    def_config.pos_i = 0.0f;
    def_config.pos_d = 0.0f;
    def_config.pos_ff = 0.0f;
    def_config.pos_lpf_alpha = 1.0f;
    def_config.vel_p = 0.0f;
    def_config.vel_i = 0.0f;
    def_config.vel_d = 0.0f;
    def_config.vel_ff = 0.0f;
    def_config.vel_lpf_alpha = 1.0f;
    def_config.saved_position = 0;
    def_config.pos_limit_min = 0;
    def_config.pos_limit_max = 0;
    
    for (int i = 1; i <= NUM_MOTORS; i++) {
        saveMotorConfig(i, def_config);
    }
    EEPROM.put(MAGIC_ADDR, MAGIC_NUMBER);
}

void ConfigStorage::saveMotorConfig(int motor_id, const MotorConfig& config) {
    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
        int addr = CONFIG_START_ADDR + (motor_id - 1) * sizeof(MotorConfig);
        EEPROM.put(addr, config);
    }
}

MotorConfig ConfigStorage::loadMotorConfig(int motor_id) {
    MotorConfig config;
    // Default fallback
    config.motor_dir = 1;
    config.encoder_dir = 1;
    config.lpf_input_alpha = 0.5f;
    config.pos_p = 0.0f;
    config.pos_i = 0.0f;
    config.pos_d = 0.0f;
    config.pos_ff = 0.0f;
    config.pos_lpf_alpha = 1.0f;
    config.vel_p = 0.0f;
    config.vel_i = 0.0f;
    config.vel_d = 0.0f;
    config.vel_ff = 0.0f;
    config.vel_lpf_alpha = 1.0f;
    config.saved_position = 0;
    config.pos_limit_min = 0;
    config.pos_limit_max = 0;
    
    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
        int addr = CONFIG_START_ADDR + (motor_id - 1) * sizeof(MotorConfig);
        EEPROM.get(addr, config);
        
        // Validate dirs
        if (config.motor_dir != 1 && config.motor_dir != -1) config.motor_dir = 1;
        if (config.encoder_dir != 1 && config.encoder_dir != -1) config.encoder_dir = 1;
    }
    return config;
}
