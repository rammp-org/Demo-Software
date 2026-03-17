#include "ConfigStorage.h"

// Out-of-class definitions for static const members (required for linker)
const uint16_t ConfigStorage::MAGIC_NUMBER;
const int ConfigStorage::MAGIC_ADDR;
const int ConfigStorage::DIR_START_ADDR;
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
    int8_t defaults[NUM_MOTORS] = {1, 1, 1, 1, 1, 1};
    saveAllDirections(defaults);
}

void ConfigStorage::saveMotorDirection(int motor_id, int8_t direction) {
    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
        int addr = DIR_START_ADDR + motor_id - 1;
        EEPROM.update(addr, (uint8_t)direction);
    }
}

int8_t ConfigStorage::loadMotorDirection(int motor_id) {
    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
        int addr = DIR_START_ADDR + motor_id - 1;
        int8_t dir = (int8_t)EEPROM.read(addr);
        // Validate: must be 1 or -1
        if (dir != 1 && dir != -1) {
            dir = 1;  // Default to normal
        }
        return dir;
    }
    return 1;  // Default
}

void ConfigStorage::saveAllDirections(int8_t* dirs) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        EEPROM.update(DIR_START_ADDR + i, (uint8_t)dirs[i]);
    }
    EEPROM.put(MAGIC_ADDR, MAGIC_NUMBER);
}

void ConfigStorage::loadAllDirections(int8_t* dirs) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        dirs[i] = loadMotorDirection(i + 1);
    }
}
