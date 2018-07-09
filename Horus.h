#ifndef ROCKET_H
#define ROCKET_H


//== Change this to select test or launch settings! ==

#define TEST_MODE

//====================================================


#include <Wire.h>
#include <EEPROM.h>

#include "SFE_BMP180.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#ifdef TEST_MODE
#include "config_test.h"
#else
#include "config_launch.h"
#endif



#define EEPROM_APOGEE_T_ADDR            0
#define EEPROM_DEPLOYMENT_T_ADDR        4
#define EEPROM_TOUCHDOWN_T_ADDR         8
#define EEPROM_MAX_ALT_ADDR             10
#define EEPROM_LOG_IGNITION             12
#define EEPROM_LOG_THRUST               13
#define EEPROM_LOG_TUMBLE               14
#define EEPROM_LOG_PARACHUTE            15
#define EEPROM_LOG_TOUCHDOWN            16
#define EEPROM_LOG_SYSERROR             17
#define EEPROM_LOG_APOGEE_TIMEOUT       18
#define EEPROM_LOG_FINALEEPROMWRITE     19
#define EEPROM_PRESSURE_ERROR           20
#define EEPROM_DEFAULT_CASE_ERROR       21
#define EEPROM_HASDATA                  22

#define EEPROM_MAX_LENGTH               23
#define EEPROM_RESET_DELAY              10000

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
uint32_t altPollT = 0;
int16_t maxAlt = 0;



/************************
 *    Flight States     *
 ************************/

enum e_flightState{sysError, dataUpload, erase, preLaunch, ignition, thrust, tumble, parachute, touchdown, complete};
e_flightState flightState = preLaunch;
bool detonationOn = false;
int16_t alt = 0;
uint32_t touchdownDetectT = 0;
int16_t touchdownDetectAlt = 0;




/************************
 *    LED Settings      *
 ************************/

enum e_ledState{on, off, flash_on, flash_off};
e_ledState ledState = off;
unsigned long ledTime = 0;
unsigned int ledFlashOnTime = 250;
unsigned int ledFlashOffTime = 250;
bool ledOn = false;




/************************
 *   Program logging    *
 ************************/
bool log_ignition = false;
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
