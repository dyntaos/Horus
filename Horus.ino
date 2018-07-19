#include "Horus.h"

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
int16_t apogeeAlt = 0;
int16_t maxAlt = 0;
int16_t writtenMaxAlt = 0;
int16_t minAlt = 0;
int16_t writtenMinAlt = 0;

/************************
 *     Misc Timers      *
 ************************/

uint32_t altPollT = 0;
uint32_t auxPollT = 0;

uint32_t altIncrementalLogTime = 0;
uint32_t altMinMaxIncrementalLogTime = 0;

/************************
 *    Flight States     *
 ************************/

enum e_flightState {
    sysError, dataUpload, erase, preLaunch, ignition, thrust, tumble, parachute, touchdown, complete
};
e_flightState flightState = preLaunch;
bool detonationOn = false;
int16_t alt = 0;
uint32_t touchdownDetectT = 0;
int16_t touchdownDetectAlt = 0;

/************************
 *    LED Settings      *
 ************************/

enum e_ledState {on, off, flash_on, flash_off};
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



#ifdef ENABLE_SERIAL_DEBUGGING
    inline void PrintSensorData(const __FlashStringHelper* fState) {
        if (millis() - debugPollT >= DEBUG_POLL_TIME) {
            Serial.print("Flight State: ");
            Serial.print(fState);
            Serial.print("\tAltitude: ");
            Serial.print(alt);
            Serial.print("\tAcceleration Scalar: ");
            Serial.print(getAccelerationScalar());
            Serial.print("\t\tX: ");
            Serial.print(getAccelX());
            Serial.print("\t Y: ");
            Serial.print(getAccelY());
            Serial.print("\t Z: ");
            Serial.print(getAccelZ());
            Serial.print("\tSwitch Input: ");
            Serial.println(digitalRead(RESET_PIN)?"HIGH":"LOW");
            debugPollT = millis();
        }
    }

    #define SerialDebugMsg(msg)             Serial.println(msg)

#else
    inline void PrintSensorData(const __FlashStringHelper* fState) {}
    #define SerialDebugMsg(msg)
#endif

inline void logIncrementalAltitude() {
    if (millis() >= altIncrementalLogTime) {
        altIncrementalLogTime = millis() + ALT_LOG_INTERVAL;
        packEEPROM(alt);
    }
}

inline void writeAltMinMax() {
  if (millis() >= altMinMaxIncrementalLogTime){
    if (minAlt != writtenMinAlt) {
        EEPROM.put(EEPROM_MIN_ALT, minAlt);
        writtenMinAlt = minAlt;
    }
    if (maxAlt != writtenMaxAlt) {
        EEPROM.put(EEPROM_MAX_ALT, maxAlt);
        writtenMaxAlt = maxAlt;
    }
    altMinMaxIncrementalLogTime = millis() + ALT_MIN_MAX_LOG_INTERVAL;
  }
}


void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    Wire.begin();

    pinMode(LED_PIN, OUTPUT);
    pinMode(DETONATION_PIN, OUTPUT);
    pinMode(RESET_PIN, INPUT);
    digitalWrite(DETONATION_PIN, LOW);

    dataExport();

    /* Initialize the MPU-6050 accelerometer */
    mpu6050.initialize();
    mpu6050.setFullScaleAccelRange(ACCELEROMETER_FULL_RANGE_SCALE);

    if (mpu6050.testConnection()) {
        SerialDebugMsg(F("MPU-6050 accelerometer initialized..."));
    } else {
        flightState = sysError;
        ledState = flash_on;
        ledFlashOnTime = 200;
        ledFlashOffTime = 1000;
        SerialDebugMsg(F("MPU-6050 accelerometer initialization error!"));
    }
    
    /* Initialize the BMP180 altimeter */
    if (pressure.begin()) {
        SerialDebugMsg(F("BMP180 altimeter initialized..."));
        baseline = getPressure();
    } else {
        flightState = sysError;

        ledFlashOnTime = 200;
        ledFlashOffTime = 1000;
        SerialDebugMsg(F("BMP180 altimeter initialization error!"));
    }
    
    ledTime = millis();
    altPollT = millis();
    auxPollT = millis();

#ifdef ENABLE_SERIAL_DEBUGGING
    debugPollT = ledTime;
#endif

    if (digitalRead(RESET_PIN) == HIGH) {
        //CLEAR EEPROM
        SerialDebugMsg(F("Clearing EEPROM..."));
        flightState = erase;
        ledFlashOffTime = 50;
        ledFlashOnTime = 50;
        ledState = flash_on;
        clearEEPROM();
        auxPollT = millis();
        SerialDebugMsg(F("EEPROM successfully cleared..."));
    }
    
    if (!getFlag(EEPROM_HASDATA)) {
        ledFlashOffTime = 200;
        ledFlashOnTime = 50;
        ledState = flash_on;
        flightState = dataUpload;
        auxPollT = millis();
    }
    
    if (flightState == preLaunch) {
        ledState = on;
        setFlag(EEPROM_HASDATA, true);
    }
    
#ifndef ENABLE_SERIAL_DEBUGGING
    Serial.end();
#endif
}

void loop() {
    int16_t vAccel;
    uint32_t t;
    uint16_t accelScalar;

    if (ledState != off)
        flashLED();

    t = millis();

    if ((t - altPollT) > ALT_POLL_TIME) {
        altPollT = t;
        alt = getAltitude();
        if (alt < minAlt) {
            minAlt = alt;
        }
        if (alt > maxAlt) {
            maxAlt = alt;
        }
    }

    switch (flightState) {

        case sysError:
            PrintSensorData(F("SysError"));
            log_syserror = true;
            setFlag(EEPROM_LOG_SYSERROR, true);
            log_finaleepromwrite = true;
            break;

        case dataUpload:
            PrintSensorData(F("DataUpload"));
            if (millis() - auxPollT >= DATA_UPLOAD_DELAY) {

#ifndef ENABLE_SERIAL_DEBUGGING
                Serial.begin(SERIAL_BAUD_RATE);
#endif

                dataExport();

#ifndef ENABLE_SERIAL_DEBUGGING
                Serial.end();
#endif

                auxPollT = millis();
            }

            break;

        case erase:
            PrintSensorData(F("Erase"));
            if (millis() - auxPollT >= EEPROM_RESET_DELAY) {
                ledFlashOffTime = 250;
                ledFlashOnTime = 250;
                ledState = on;
                flightState = preLaunch;

                SerialDebugMsg(F("Resuming launch subroutines..."));
            }
            break;

        case preLaunch:
            PrintSensorData(F("PreLaunch"));
            vAccel = getVerticalAccel();

            if (vAccel <= ACCEL_IGNITION_START) {
                ledState = off;
                flightState = ignition;
                ignitionT = millis();
                altIncrementalLogTime = ignitionT + ALT_LOG_INTERVAL;
                altMinMaxIncrementalLogTime = ignitionT + ALT_MIN_MAX_LOG_INTERVAL;
            }
            break;

        case ignition:
            PrintSensorData(F("Ignition"));
            
            logIncrementalAltitude();
            writeAltMinMax();
            
            vAccel = getVerticalAccel();
            
            if (vAccel > ACCEL_IGNITION_START) {
                flightState = preLaunch;
                ledState = on;
                ignitionT = 0;
                minAlt = 0;
                maxAlt = 0;
                resetPack();
            } else if ((millis() - ignitionT) >= IGNITION_SUSTAIN_T) {
                flightState = thrust;
            }
            break;

        case thrust:
            PrintSensorData(F("Thrust"));
            
            logIncrementalAltitude();
            writeAltMinMax();
            
            accelScalar = getAccelerationScalar();
            log_thrust = true;
            setFlag(EEPROM_LOG_THRUST, true);
            t = millis();

            if (alt > apogeeAlt) { //Floating point rounding errors are irrelevant and insignificant
                apogeeAlt = alt;
                apogeeT = t - ignitionT;
            }

            vAccel = getVerticalAccel();
            if (vAccel >= APOGEE_ACCEL_RANGE) {
                flightState = tumble;
                apogeeT = t - ignitionT;
            }

            //If max ignition + thrust time has elapsed (safety measure to ensure parachute is more likely to deploy if an error occurs)
            if ((t - ignitionT) >= MAX_THRUST_TIME) {
                log_apogee_timeout = true;
                setFlag(EEPROM_LOG_APOGEE_TIMEOUT, true);
                flightState = tumble;
                apogeeT = t - ignitionT;
            }
            break;

        case tumble:
            PrintSensorData(F("Tumble"));
            
            logIncrementalAltitude();
            writeAltMinMax();
            
            log_tumble = true;
            setFlag(EEPROM_LOG_TUMBLE, true);
            EEPROM.put(EEPROM_APOGEE_ALT, apogeeAlt);
            EEPROM.put(EEPROM_APOGEE_TIME, apogeeT);
            t = millis();

            if (alt <= DEPLOY_ALTITUDE) {
                //DEPLOY PARACHUTE
                flightState = parachute;
                t = millis();
                detonationOn = true;
                digitalWrite(DETONATION_PIN, HIGH);
                deploymentT = t - ignitionT;
                touchdownDetectT = t;
                touchdownDetectAlt = alt;
            }

            break;

        case parachute:
            PrintSensorData(F("Parachute"));
            
            logIncrementalAltitude();
            writeAltMinMax();
            
            log_parachute = true;
            setFlag(EEPROM_LOG_PARACHUTE, true);
            EEPROM.put(EEPROM_DEPLOYMENT_TIME, deploymentT);
            t = millis();

            if (detonationOn) {
                if ((deploymentT + DETONATION_DURATION_MS) >= t) {
                    detonationOn = false;
                    digitalWrite(DETONATION_PIN, LOW);
                }
            }

            accelScalar = getAccelerationScalar();

            if (((touchdownDetectAlt + TOUCHDOWN_ALT_VARIANCE > alt) || (touchdownDetectAlt - TOUCHDOWN_ALT_VARIANCE < alt))) {
                //Max touchdown variance exceeded, reset the timer
                touchdownDetectT = t;
                touchdownDetectAlt = alt;
            } else if ((touchdownDetectT + TOUCHDOWN_ALT_DURATION >= t) && (accelScalar >= GRAVITY - TOUCHDOWN_ACCEL_VARIANCE)
                    && (accelScalar <= GRAVITY + TOUCHDOWN_ACCEL_VARIANCE)) {
                //Touchdown!
                flightState = touchdown;
                touchdownT = t;
            }

            break;

        case touchdown:
            PrintSensorData(F("Touchdown"));
            
            logIncrementalAltitude();
            writeAltMinMax();
            
            log_touchdown = true;
            setFlag(EEPROM_LOG_TOUCHDOWN, true);
            EEPROM.put(EEPROM_TOUCHDOWN_TIME, touchdownT);

            ledFlashOffTime = 1000;
            ledFlashOnTime = 1000;
            ledState = flash_on;

            log_finaleepromwrite = true;
            setFlag(EEPROM_LOG_FINALEEPROMWRITE, true);
            flightState = complete;
            break;

        case complete:
            PrintSensorData(F("Complete"));
            
            logIncrementalAltitude();
            writeAltMinMax();
            
            //Sit idle
            break;

        default:
            PrintSensorData(F("-Default Case-"));
            log_default_case_error = true;
            setFlag(EEPROM_DEFAULT_CASE_ERROR, true);

            //Attempt to detect previous flightState and restore flightState
            if (log_touchdown)
                flightState = complete;
            else if (log_parachute)
                flightState = touchdown;
            else if (log_tumble)
                flightState = parachute;
            else if (log_thrust)
                flightState = tumble;
            else
                flightState = preLaunch;

            break;

    }
}

void flashLED() {
    if (ledState == flash_on || ledState == flash_off) {
        uint32_t t = millis() - ledTime;
        if (ledState == flash_on && t >= ledFlashOnTime) {
            //LED On
            ledState = flash_off;
            digitalWrite(LED_PIN, LOW);
            ledTime = millis();
        } else if (ledState == flash_off && t >= ledFlashOffTime) {
            //LED OFF
            ledState = flash_on;
            digitalWrite(LED_PIN, HIGH);
            ledTime = millis();
        }
    } else if (ledState == on && !ledOn) {
        digitalWrite(LED_PIN, HIGH);
        ledOn = true;
    } else if (ledState == off && ledOn) {
        digitalWrite(LED_PIN, LOW);
        ledOn = false;
    }
}

void dataExport() {

    Serial.println(F("Uploaded Data:"));
    Serial.println(F("\tLogs:"));

    Serial.print(F("\t\tSystem error state:\t"));
    Serial.println(getFlag(EEPROM_LOG_SYSERROR) ? F("TRUE") : F("FALSE"));

    Serial.print(F("\t\tPressure error:\t\t"));
    Serial.println(getFlag(EEPROM_PRESSURE_ERROR) ? F("TRUE") : F("FALSE"));

    Serial.print(F("\t\tDefault case error:\t"));
    Serial.println(getFlag(EEPROM_DEFAULT_CASE_ERROR) ? F("TRUE") : F("FALSE"));

    Serial.print(F("\t\tThrust state:\t\t"));
    Serial.println(getFlag(EEPROM_LOG_THRUST) ? F("TRUE") : F("FALSE"));

    Serial.print(F("\t\tApogee timeout:\t\t"));
    Serial.println(getFlag(EEPROM_LOG_APOGEE_TIMEOUT) ? F("TRUE") : F("FALSE"));

    Serial.print(F("\t\tTumble state:\t\t"));
    Serial.println(getFlag(EEPROM_LOG_TUMBLE) ? F("TRUE") : F("FALSE"));

    Serial.print(F("\t\tParachute state:\t"));
    Serial.println(getFlag(EEPROM_LOG_PARACHUTE) ? F("TRUE") : F("FALSE"));

    Serial.print(F("\t\tTouchdown state:\t"));
    Serial.println(getFlag(EEPROM_LOG_TOUCHDOWN) ? F("TRUE") : F("FALSE"));

    Serial.print(F("\t\tFinal EEPROM write:\t"));
    Serial.println(getFlag(EEPROM_LOG_FINALEEPROMWRITE) ? F("TRUE") : F("FALSE"));

    Serial.println(F("\n\tFlight Info:"));
    double tempDbl;

    Serial.print(F("\t\tApogee altitude:\t"));
    EEPROM.get(EEPROM_APOGEE_ALT, tempDbl);
    Serial.println(tempDbl);

    Serial.print(F("\t\tMax altitude:\t"));
    EEPROM.get(EEPROM_MAX_ALT, tempDbl);
    Serial.println(tempDbl);

    Serial.print(F("\t\tMin altitude:\t"));
    EEPROM.get(EEPROM_MIN_ALT, tempDbl);
    Serial.println(tempDbl);

    uint32_t tempLong;

    Serial.print(F("\t\tTime to apogee:\t"));
    EEPROM.get(EEPROM_APOGEE_TIME, tempLong);
    Serial.print(tempLong / 1000.0);
    Serial.println(F(" sec"));

    Serial.print(F("\t\tTime to deployment:\t"));
    EEPROM.get(EEPROM_DEPLOYMENT_TIME, tempLong);
    Serial.print(tempLong / 1000.0);
    Serial.println(F(" sec"));

    Serial.print(F("\t\tTime to touchdown:\t"));
    EEPROM.get(EEPROM_TOUCHDOWN_TIME, tempLong);
    Serial.print(tempLong / 1000.0);
    Serial.println(F(" sec"));

    Serial.println();
}

void clearEEPROM() {
    for (int i = 0; i <= EEPROM_MAX_LENGTH; i++) {
        EEPROM.write(i, 0);
    }
}


double getAccelerationScalar() {
    int16_t ax, ay, az;
    double dax, day, daz;
    mpu6050.getAcceleration(&ax, &ay, &az);
    dax = ax * ACCELEROMETER_MULTIPLIER;
    day = ay * ACCELEROMETER_MULTIPLIER;
    daz = az * ACCELEROMETER_MULTIPLIER;
    return sqrt((dax * dax) + (day * day) + (daz * daz));
}

double getAltitude() {
    //Value in meters, relative to baseline
    double P;
    P = getPressure();
    return pressure.altitude(P, baseline);
}

double getPressure() {
    char status;
    double T, P;

    // You must first get a temperature measurement to perform a pressure reading.

    // Start a temperature measurement:
    // If request is successful, the number of ms to wait is returned.
    // If request is unsuccessful, 0 is returned.

    status = pressure.startTemperature();
    if (status != 0) {
        // Wait for the measurement to complete:

        delay(status);

        // Retrieve the completed temperature measurement:
        // Note that the measurement is stored in the variable T.
        // Use '&T' to provide the address of T to the function.
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getTemperature(T);
        if (status != 0) {
            // Start a pressure measurement:
            // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
            // If request is successful, the number of ms to wait is returned.
            // If request is unsuccessful, 0 is returned.

            status = pressure.startPressure(3);
            if (status != 0) {
                // Wait for the measurement to complete:
                delay(status);

                // Retrieve the completed pressure measurement:
                // Note that the measurement is stored in the variable P.
                // Use '&P' to provide the address of P.
                // Note also that the function requires the previous temperature measurement (T).
                // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
                // Function returns 1 if successful, 0 if failure.

                status = pressure.getPressure(P, T);
                if (status != 0) {
                    return (P);
                } else {
                    SerialDebugMsg(F("error retrieving pressure measurement(1)\n"));
                    if (!log_pressure_error) setFlag(EEPROM_PRESSURE_ERROR, true);
                    log_pressure_error = true;
                }
            } else {
                SerialDebugMsg(F("error starting pressure measurement(2)\n"));
                if (!log_pressure_error) setFlag(EEPROM_PRESSURE_ERROR, true);
                log_pressure_error = true;
            }
        } else {
            SerialDebugMsg(F("error retrieving temperature measurement(1)\n"));
            if (!log_pressure_error) setFlag(EEPROM_PRESSURE_ERROR, true);
            log_pressure_error = true;
        }
    } else {
        SerialDebugMsg(F("error starting temperature measurement(2)\n"));
        if (!log_pressure_error) setFlag(EEPROM_PRESSURE_ERROR, true);
        log_pressure_error = true;
    }
    return 0.0;
}
