#ifndef EEPROM_PACKING_H
#define EEPROM_PACKING_H

#include <EEPROM.h>
#include <stdint.h>

// 4-byte addresses
#define EEPROM_APOGEE_TIME              0
#define EEPROM_DEPLOYMENT_TIME          4
#define EEPROM_TOUCHDOWN_TIME           8
#define EEPROM_APOGEE_ALT               12
#define EEPROM_MAX_ALT                  16
#define EEPROM_MIN_ALT                  20
#define EEPROM_ALT_BASELINE             24
#define EEPROM_BOOT_MS                  28

// Bit addresses (relative to EEPROM_BASE_FLAG_ADDR)
#define EEPROM_LOG_THRUST               0
#define EEPROM_LOG_TUMBLE               1
#define EEPROM_LOG_PARACHUTE            2
#define EEPROM_LOG_TOUCHDOWN            3
#define EEPROM_LOG_SYSERROR             4
#define EEPROM_LOG_APOGEE_TIMEOUT       5
#define EEPROM_LOG_FINALEEPROMWRITE     6
#define EEPROM_PRESSURE_ERROR           7
#define EEPROM_DEFAULT_CASE_ERROR       8
#define EEPROM_HASDATA                  9

// EEPROM Values (Not addresses)
#define EEPROM_BASE_FLAG_ADDR           30
#define EEPROM_MAX_LENGTH               1023
#define EEPROM_RESET_DELAY              10000
#define EEPROM_ALT_LOG_START            32
#define EEPROM_ALT_LOG_END              1023


bool getFlag(uint8_t n);
void setFlag(uint8_t n, bool val);
void packEEPROM(uint16_t data);
void resetPack();
void finalizePackEEPROM();
uint16_t unpackEEPROM(uint16_t n);

#endif
