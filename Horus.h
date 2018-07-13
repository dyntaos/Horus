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
#include <EEPROM.h>


#ifdef TEST_MODE
#include "config_test.h"
#else
#include "config_launch.h"
#endif

#define DATA_UPLOAD_DELAY               5000

#ifdef ENABLE_SERIAL_DEBUGGING

#define PrintSensorData(fState)         do { if (millis() - debugPollT >= DEBUG_POLL_TIME) { \
                                              Serial.print("Flight State: "); \
                                              Serial.print(fState); \
                                              Serial.print("\tAltitude: "); \
                                              Serial.print(alt); \
                                              Serial.print("\tAcceleration Scalar: "); \
                                              Serial.print(getAccelerationScalar()); \
                                              Serial.print("\t\tX: "); \
                                              Serial.print(getAccelX()); \
                                              Serial.print("\t Y: "); \
                                              Serial.print(getAccelY()); \
                                              Serial.print("\t Z: "); \
                                              Serial.print(getAccelZ()); \
                                              Serial.print("\tSwitch Input: "); \
                                              Serial.println(digitalRead(SWITCH_PIN)?"HIGH":"LOW"); \
                                              debugPollT = millis(); \
                                        } } while(0)

#define SerialDebugMsg(msg)             Serial.println(msg)

#else
#define PrintSensorData(fState)
#define SerialDebugMsg(msg)
#endif

#define logIncrementalAltitude()        do { if (millis() >= altIncrementalLogTime) { \
                                                altIncrementalLogTime = millis(); \
                                                packEEPROM(alt); \
                                        } } while (0)

#define writeAltToEEPROM()              do { if (minAlt != writtenMinAlt) { \
                                                EEPROM.put(EEPROM_MIN_ALT, minAlt); \
                                                writtenMinAlt = minAlt; \
                                            } \
                                            if (maxAlt != writtenMaxAlt) { \
                                                EEPROM.put(EEPROM_MAX_ALT, maxAlt); \
                                                writtenMaxAlt = maxAlt; \
                                        } } while(0)

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
