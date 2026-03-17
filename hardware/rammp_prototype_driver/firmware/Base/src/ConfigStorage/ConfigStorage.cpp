#include "ConfigStorage.h"

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
    EEPROM.put(MAGIC_ADDR, MAGIC_NUMBER);
}

void ConfigStorage::saveMotorDirection(int motor_id, int8_t direction) {
    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
        EEPROM.write(DIR_START_ADDR + motor_id - 1, direction);
    }
}

int8_t ConfigStorage::loadMotorDirection(int motor_id) {
    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
        int8_t dir = EEPROM.read(DIR_START_ADDR + motor_id - 1);
        // Validate: must be 1 or -1
        if (dir != 1 && dir != -1) {
            dir = 1;  // Default to normal
        }
        return dir;
    }
    return 1;  // Default
}

void ConfigStorage::saveAllDirections(int8_t dirs[NUM_MOTORS]) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        EEPROM.write(DIR_START_ADDR + i, dirs[i]);
    }
    EEPROM.put(MAGIC_ADDR, MAGIC_NUMBER);
}

void ConfigStorage::loadAllDirections(int8_t dirs[NUM_MOTORS]) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        dirs[i] = loadMotorDirection(i + 1);
    }
}
