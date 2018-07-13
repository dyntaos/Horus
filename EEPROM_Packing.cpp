//This file contains functions to store data in the EEPROM in 10-bit chunks (2^10 - 1 = 1023)
//This increases the amount of data we can pack in the EEPROM

#include "EEPROM_Packing.h"

uint16_t buffer;
uint8_t *ptr = 0;

uint8_t remainderBits = 0;
uint16_t curAddr = EEPROM_ALT_LOG_START;
bool dataNotWritten = false;

static const uint16_t bitMask = 0b0000001111111111;

bool getFlag(uint8_t n) {
    uint8_t EEPROM_Addr = (n / 8) + EEPROM_BASE_FLAG_ADDR;   //Get EEPROM address
    return (bool) (EEPROM_Addr >> (n - ((n / 8) * 8))) & 1U; //Extract flag from byte and return as boolean
}

void setFlag(uint8_t n, bool val) {
    uint8_t EEPROM_Addr = (n / 8) + EEPROM_BASE_FLAG_ADDR;
    uint8_t temp = EEPROM.read(EEPROM_Addr);
    temp |= val << (n - ((n / 8) * 8));
    EEPROM.write(EEPROM_Addr, temp);
}

/**
 * Pack a 10-bit number into a byte array in the EEPROM
 * These routines leave the unused 6-bits of the word for
 * other values (no wasted bits). To avoid unnecessary
 * EEPROM writes, there may be data that has been buffered
 * to write the next call to packEEPROM(). When you are done
 * calling packEEPROM(), you should call finalizePackEEPROM() once.
 */
void packEEPROM(uint16_t data) {
    if (ptr == 0) {
        ptr = (uint8_t*) &buffer;
        ptr++;
    }
    if ((remainderBits == 6 && curAddr == EEPROM_ALT_LOG_END) || (curAddr < EEPROM_ALT_LOG_END)) {
        data &= bitMask;
        data <<= (6 - remainderBits);
        buffer |= data;
        EEPROM.write(curAddr, *ptr);
        curAddr++;
        buffer <<= 8;
        remainderBits += 2;
        dataNotWritten = true;
        if (remainderBits >= 8) {
            EEPROM.write(curAddr, *ptr);
            curAddr++;
            buffer <<= 8;
            remainderBits = 0;
            dataNotWritten = false;
        } else if (remainderBits != 0) {
            if (!((remainderBits == 6 && curAddr == EEPROM_ALT_LOG_END) || (curAddr < EEPROM_ALT_LOG_END))) {
                EEPROM.write(curAddr, *ptr);
                curAddr++;
                buffer <<= 8;
                remainderBits = 0;
                dataNotWritten = false;
            }
        }
    }
}

void resetPack() {
    curAddr = EEPROM_ALT_LOG_START;
    remainderBits = 0;
    dataNotWritten = false;
    buffer = 0;
}

/**
 * If there is any unwritten data, write it to EEPROM
 * This should only be done when no more data is to be saved
 * to EEPROM to avoid unnecessary wear on EEPROM. If this
 * is called and more data needs to be written, it will work.
 */
void finalizePackEEPROM() {
    if (dataNotWritten && remainderBits != 0) {
        EEPROM.write(curAddr, *ptr);
        dataNotWritten = false;
    }
}

/**
 * TODO
 */
uint16_t unpackEEPROM(uint16_t n) {
    uint16_t firstIndex = (((n - EEPROM_ALT_LOG_START) * 10) / 8) + EEPROM_ALT_LOG_START;
    uint8_t firstBit = (((n - EEPROM_ALT_LOG_START) * 10) % 8);
    
    uint16_t value = 0;
    uint8_t *lo = (uint8_t*) &value;
    uint8_t *hi = lo + 1;
    
    *hi = EEPROM.read(firstIndex);
    *lo = EEPROM.read(firstIndex + 1);
    *hi &= (0b11111111 >> firstBit);
    *lo &= (0b1111111100000000 >> (2 + firstBit));
    value >>= (6 - firstBit);
    return value;
}

