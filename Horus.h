#ifndef ROCKET_H
#define ROCKET_H

//============================================================
//== Comment/uncomment this to toggle test/launch settings! ==
//============================================================

#define TEST_MODE

//============================================================

#include <Wire.h>

#include "SFE_BMP180.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#include "EEPROM_Packing.h"


#ifdef TEST_MODE
    #include "config_test.h"
#else
    #include "config_launch.h"
#endif

#define DATA_UPLOAD_DELAY               5000


/********************************
 *    Sensor Objects & Data     *
 ********************************/

SFE_BMP180 pressure;
double baseline;

MPU6050 mpu6050;

/************************
 *     Flight Data      *
 ************************/

uint32_t ignitionT;
uint32_t apogeeT;
uint32_t deploymentT;
uint32_t touchdownT;
double apogeeAlt = 0.0;
double maxAlt = 0.0;
double writtenMaxAlt = 0.0;
double minAlt = 0.0;
double writtenMinAlt = 0.0;

/************************
 *     Misc Timers      *
 ************************/

uint32_t altPollT = 0;
uint32_t auxPollT = 0;

uint8_t altIncrementalLogTime = 0;

/************************
 *    Flight States     *
 ************************/

enum e_flightState {
    sysError, dataUpload, erase, preLaunch, ignition, thrust, tumble, parachute, touchdown, complete
};
e_flightState flightState = preLaunch;
bool detonationOn = false;
double alt = 0.0;
uint32_t touchdownDetectT = 0;
int16_t touchdownDetectAlt = 0;

/************************
 *    LED Settings      *
 ************************/

enum e_ledState {
    on, off, flash_on, flash_off
};
e_ledState ledState = off;
unsigned long ledTime = 0;
unsigned int ledFlashOnTime = 250;
unsigned int ledFlashOffTime = 250;
bool ledOn = false;

/************************
 *   Program logging    *
 ************************/

bool log_thrust = false;
bool log_tumble = false;
bool log_parachute = false;
bool log_touchdown = false;
bool log_syserror = false;

bool log_apogee_timeout = false;
bool log_finaleepromwrite = false;

bool log_pressure_error = false;
bool log_default_case_error = false;

/************************/

#endif
