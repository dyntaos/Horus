# Horus
Horus - Arduino model rocket recovery and data recording system


**Currently this project is a work in progress and is not fully operational yet:exclamation:**


This code was created to use a GY-87 10DOF sensor module and is currently being tested and developed on an Arduino Uno R3, to be later adapted for the Arduino Pro Mini for "launch" mode.

More documentation will be written when the code is fully operational.

**Contact me if you want further information on the project that is not addressed here :speech_balloon:**


## The intended purpose of this system is to perform 2 main functions:

- Record flight data:
  - Maximum altitude
  - Time to apogee, parachute deployment, touchdown
- Delay the deployment of the rocket until a lower altitude, to prevent winds from carrying the rocket excessively far from the launch site. At apogee, the engines recovery charge will initiate a separation of the body tubes (tumble recovery), then later the parachute will be deployed at a preset altitude, performed by the Arduino via MOSFET triggered black-powder charge.


## Why?
This rocket I am designing this for is projected to fly just under 1km in altitude and if its parachute deploys at 1km, with even light winds, I will likely never recover the rocket, let alone see it again.


## LED Codes
LED output pin is set by "LED_PIN" in `config_launch.h` & `config_test.h`

- Steady On:                      PreLaunch mode - System is operational and ready to be launched
- Flashing 200ms on, 1000ms off:  Sensor Error
- Flashing 50ms on, 50ms off:     Clearing previous flight data (EEPROM)
- Flashing 50ms on, 200ms off:    DataUpload mode - Retrieval of previous flight data (will not record new data until previous flight data is cleared)
- Flashing 1000ms on, 1000ms off: Flight fully completed


## Problems compiling?
Please make sure you have version 1.8.5 or newer of the Arduino software. A library used by this code was changed in 1.6.2 (EEPROM.h) and does not compile with the earlier version. I do not know if it compiles with the 1.6.2 version having said this. It is only tested with 1.8.5.