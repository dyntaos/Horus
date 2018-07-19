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

#endif
