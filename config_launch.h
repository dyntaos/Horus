#ifndef CONFIG_H
#define CONFIG_H

#undef ENABLE_SERIAL_DEBUGGING
#define SERIAL_BAUD_RATE                38400
#define DATA_UPLOAD_DELAY               5000


#define LED_PIN                         13
#define DETONATION_PIN                  11
#define BUTTON_PIN                      12

//How long the button must be depressed during boot (actual time will be slightly longer, as this does not include initialization time) to clear the EEPROM (ms)
#define EEPROM_RESET_TIME               5000

// Meters/Sec^2 (Upward)
#define ACCEL_IGNITION_START            -12

// Time period acceleration must be sustained for to move to thrust state (milliseconds)
#define IGNITION_SUSTAIN_T              300

// Apogee detection acceleration range (meters/second^2)
#define APOGEE_ACCEL_RANGE              -12

//Expect deployment at 7400 ms to 7480 ms after ignitionT (assuming ignition is detected precisely)
//This long after ignitionT, flightState will automatically move to tumble
#define MAX_THRUST_TIME                 7700

//Minimum altitude polling time between polls (milliseconds)
#define ALT_POLL_TIME                   50

//Log the altitude to EEPROM every multiple of ALT_LOG_INTERVAL after ignitionT until EEPROM is full
#define ALT_LOG_INTERVAL                200

//How often the min and max altitudes will be written to EEPROM, but only if they have changed
#define ALT_MIN_MAX_LOG_INTERVAL        150

//Altitude in meters to deploy the parachute
#define DEPLOY_ALTITUDE                 140

//Duration to sustain the detonation charge for to deploy the parachute
#define DETONATION_DURATION_MS          2000

#define TOUCHDOWN_ALT_VARIANCE          1.0
#define TOUCHDOWN_ALT_DURATION          1.0
#define TOUCHDOWN_ACCEL_VARIANCE        0.6
#define GRAVITY                         9.8

/* ACCELEROMETER_FULL_RANGE_SCALE
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 */
#define ACCELEROMETER_FULL_RANGE_SCALE  3
#define ACCELEROMETER_MAX_VALUE         16.0*GRAVITY

// -32768 to +32767
#define ACCELEROMETER_FULL_RANGE        32767
#define ACCELEROMETER_MULTIPLIER        ACCELEROMETER_MAX_VALUE/ACCELEROMETER_FULL_RANGE

#define getAccelX()                     mpu6050.getAccelerationX()*ACCELEROMETER_MULTIPLIER
#define getAccelY()                     mpu6050.getAccelerationY()*ACCELEROMETER_MULTIPLIER
#define getAccelZ()                     -mpu6050.getAccelerationZ()*ACCELEROMETER_MULTIPLIER

//TODO: Axis depends on orientation of sensor in rocket
#define getVerticalAccel()              getAccelZ()

#ifdef ENABLE_SERIAL_DEBUGGING
    #define DEBUG_POLL_TIME                   1000
    uint32_t debugPollT = 0;
#endif

#endif
