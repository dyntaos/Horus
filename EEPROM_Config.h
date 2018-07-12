#ifndef EEPROM_CONFIG_H
#define EEPROM_CONFIG_H


// 4-byte addresses
#define EEPROM_APOGEE_TIME              0
#define EEPROM_DEPLOYMENT_TIME          4
#define EEPROM_TOUCHDOWN_TIME           8
#define EEPROM_APOGEE_ALT               12
#define EEPROM_MAX_ALT                  16
#define EEPROM_MIN_ALT                  20

// 1-byte addresses
#define EEPROM_LOG_THRUST               24
#define EEPROM_LOG_TUMBLE               25
#define EEPROM_LOG_PARACHUTE            26
#define EEPROM_LOG_TOUCHDOWN            27
#define EEPROM_LOG_SYSERROR             28
#define EEPROM_LOG_APOGEE_TIMEOUT       29
#define EEPROM_LOG_FINALEEPROMWRITE     30
#define EEPROM_PRESSURE_ERROR           31
#define EEPROM_DEFAULT_CASE_ERROR       32
#define EEPROM_HASDATA                  33

// EEPROM Values (Not addresses)
#define EEPROM_MAX_LENGTH               1023
#define EEPROM_RESET_DELAY              10000
#define EEPROM_ALT_LOG_START            34
#define EEPROM_ALT_LOG_END              1023


#endif
