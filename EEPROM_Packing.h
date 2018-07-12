#ifndef EEPROM_PACKING_H
#define EEPROM_PACKING_H

#include <EEPROM.h>
#include <stdint.h>

#define EEPROM_ALT_LOG_START            34
#define EEPROM_ALT_LOG_END              1023

//extern uint16_t buffer;
//extern uint8_t *ptr; //Change to address size of machine

//extern uint8_t remainderBits;
//extern uint16_t curAddr;
//extern bool dataNotWritten;

//extern static const uint16_t bitMask;

void packEEPROM(uint16_t data);
void finalizePackEEPROM();
uint16_t unpackEEPROM(uint16_t n);

#endif
