#include <IMU_Class.h>

IMU_Class::IMU_Class(Adafruit_BNO055 &bno_sense) : bno_sensor(bno_sense) {};

void IMU_Class::initialize_BNO055_sensor()
{
    /* Initialise BNO */
    if (!bno_sensor.begin())
    {
        // There was a problem detecting the BNO055 ... check your connections
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }
}

void IMU_Class::retrieve_readings()
{
    // Display orientation sensor values:
    // - VECTOR_ACCELEsROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Vector<3> euler = bno_sensor.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accel = bno_sensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    ax = accel.x();
    ay = accel.y();
    az = accel.z();
    // IMU.pitch =  euler.z() - 2.0- IMU.pitch_offset;   //offset to slightly tilt back the chair. Change offset in SL speed change
    pitch = euler.z() - 2.0 - 3.0;
    roll = euler.y() + 1.5;
    yaw = euler.x();

    pitchf = pitchf + K * (pitch - pitchf);
    rollf = rollf + K * (roll - rollf);

    pitchrd = 1 * pitchf / DG;
    rollrd = -1 * rollf / DG;
}