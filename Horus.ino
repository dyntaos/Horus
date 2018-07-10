#include "Horus.h"


void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Wire.begin();
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(DETONATION_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
  digitalWrite(DETONATION_PIN, LOW);
  
  dataExport();
  
  /* Initialize the MPU-6050 accelerometer */
  mpu6050.initialize();
  mpu6050.setFullScaleAccelRange(ACCELEROMETER_FULL_RANGE_SCALE);
  
  if (mpu6050.testConnection()){
    SerialDebugMsg(F("MPU-6050 accelerometer initialized..."));
  }else {
    flightState = sysError;
    ledState = flash_on;
    ledFlashOnTime = 200;
    ledFlashOffTime = 1000;
    SerialDebugMsg(F("MPU-6050 accelerometer initialization error!"));
  }

  /* Initialize the BMP180 altimeter */
  if (pressure.begin()){
    SerialDebugMsg(F("BMP180 altimeter initialized..."));
    baseline = getPressure();
  }else{
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

  if (digitalRead(SWITCH_PIN) == HIGH){
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
  
  if (EEPROM.read(EEPROM_HASDATA) != 0){
      ledFlashOffTime = 200;
      ledFlashOnTime = 50;
      ledState = flash_on;
      flightState = dataUpload;
      auxPollT = millis();
  }
  
  if (flightState == preLaunch){
    ledState = on;
    EEPROM.write(EEPROM_HASDATA, 1);
  }

  #ifndef ENABLE_SERIAL_DEBUGGING
  Serial.end();
  #endif
}



void loop() {
  int16_t vAccel;
  uint32_t t;
  uint16_t accelScalar;
  
  if (ledState != off) flashLED();

  t = millis();
  
  if ((t - altPollT) > ALT_POLL_TIME){
    altPollT = t;
    alt = getAltitude();
    if (alt < minAlt){
      minAlt = alt;
    }
    if (alt > maxAlt){
      maxAlt = alt;
    }
  }
  
  switch (flightState){
    
    case sysError:
      PrintSensorData(F("SysError"))
      log_syserror = true;
      EEPROM.write(EEPROM_LOG_SYSERROR, 1);
      log_finaleepromwrite = true;
      break;
      
    case dataUpload:
      PrintSensorData(F("DataUpload"))
      if (millis() - auxPollT >= DATA_UPLOAD_DELAY){
        
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
      PrintSensorData(F("Erase"))
      if (millis() - auxPollT >= EEPROM_RESET_DELAY){
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

      if (vAccel <= ACCEL_IGNITION_START){
        ledState = off;
        flightState = ignition;
        ignitionT = millis();
      }
      break;

    case ignition:
      PrintSensorData(F("Ignition"));
      vAccel = getVerticalAccel();
      if (vAccel > ACCEL_IGNITION_START){
        flightState = preLaunch;
        ledState = on;
        ignitionT = 0;
      }else if ((millis() - ignitionT) >= IGNITION_SUSTAIN_T){
        flightState = thrust;
      }
      break;
    
    case thrust:
      PrintSensorData(F("Thrust"));
      accelScalar = getAccelerationScalar();
      log_thrust = true;
      EEPROM.write(EEPROM_LOG_THRUST, 1);
      t = millis();
      
      if (alt > apogeeAlt){ //Floating point rounding errors are irrelavent and insignificant
        apogeeAlt = alt;
        apogeeT = t - ignitionT;
      }

      vAccel = getVerticalAccel();
      if (vAccel >= APOGEE_ACCEL_RANGE){
        flightState = tumble;
        apogeeT = t - ignitionT;
        writeAltToEEPROM();
      }
      
      //If max ignition + thrust time has elapsed (safety measure to ensure parachute is more likely to deploy if an error occurs)
      if ((t - ignitionT) >= MAX_THRUST_TIME){
        log_apogee_timeout = true;
        EEPROM.write(EEPROM_LOG_APOGEE_TIMEOUT, 1);
        flightState = tumble;
        apogeeT = t - ignitionT;
        writeAltToEEPROM();
      }
      break;

    case tumble:
      PrintSensorData(F("Tumble"));
      log_tumble = true;
      EEPROM.write(EEPROM_LOG_TUMBLE, 1);
      EEPROM.put(EEPROM_APOGEE_ALT, apogeeAlt);
      EEPROM.put(EEPROM_APOGEE_TIME, apogeeT);
      t = millis();
      
      if (alt <= DEPLOY_ALTITUDE){
        //DEPLOY PARACHUTE
        flightState = parachute;
        t = millis();
        detonationOn = true;
        digitalWrite(DETONATION_PIN, HIGH);
        deploymentT = t - ignitionT;
        touchdownDetectT = t;
        touchdownDetectAlt = alt;
        writeAltToEEPROM();
      }
      
      break;

    case parachute:
      PrintSensorData(F("Parachute"));
      log_parachute = true;
      EEPROM.write(EEPROM_LOG_PARACHUTE, 1);
      EEPROM.put(EEPROM_DEPLOYMENT_TIME, deploymentT);
      t = millis();
      
      if (detonationOn){
        if ((deploymentT + DETONATION_DURATION_MS) >= t){
          detonationOn = false;
          digitalWrite(DETONATION_PIN, LOW);
        }
      }

      accelScalar = getAccelerationScalar();
      
      if (((touchdownDetectAlt + TOUCHDOWN_ALT_VARIANCE > alt) || (touchdownDetectAlt - TOUCHDOWN_ALT_VARIANCE < alt))){
        //Max touchdown variance exceeded, reset the timer
        touchdownDetectT = t;
        touchdownDetectAlt = alt;
      }else if ( (touchdownDetectT + TOUCHDOWN_ALT_DURATION >= t) && (accelScalar >= GRAVITY - TOUCHDOWN_ACCEL_VARIANCE) && (accelScalar <= GRAVITY + TOUCHDOWN_ACCEL_VARIANCE) ){
        //Touchdown!
        flightState = touchdown;
        touchdownT = t;
      }
      
      break;


    case touchdown:
      PrintSensorData(F("Touchdown"));
      log_touchdown = true;
      EEPROM.write(EEPROM_LOG_TOUCHDOWN, 1);
      EEPROM.put(EEPROM_TOUCHDOWN_TIME, touchdownT);
      
      ledFlashOffTime = 1000;
      ledFlashOnTime = 1000;
      ledState = flash_on;
      
      log_finaleepromwrite = true;
      EEPROM.write(EEPROM_LOG_FINALEEPROMWRITE, 1);
      flightState = complete;
      writeAltToEEPROM();
      break;

    case complete:
      PrintSensorData(F("Complete"));
      //Sit idle
      break;

    default:
      PrintSensorData(F("-Default Case-"))
      log_default_case_error = true;
      EEPROM.write(EEPROM_DEFAULT_CASE_ERROR, 1);
      
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


void flashLED(){
  if (ledState == flash_on || ledState == flash_off){
    uint32_t t = millis() - ledTime;
    if (ledState == flash_on && t >= ledFlashOnTime){
      //LED On
      ledState = flash_off;
      digitalWrite(LED_PIN, LOW);
      ledTime = millis();
    }else if (ledState == flash_off && t >= ledFlashOffTime){
      //LED OFF
      ledState = flash_on;
      digitalWrite(LED_PIN, HIGH);
      ledTime = millis();
    }
  }else if(ledState == on && !ledOn){
    digitalWrite(LED_PIN, HIGH);
    ledOn = true;
  }else if(ledState == off && ledOn){
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }
}


void dataExport(){

  Serial.println(F("Uploaded Data:"));
  Serial.println(F("\tLogs:"));
  
  Serial.print(F("\t\tSystem error state:\t"));
  Serial.println(EEPROM.read(EEPROM_LOG_SYSERROR)?F("TRUE"):F("FALSE"));
  
  Serial.print(F("\t\tPressure error:\t\t"));
  Serial.println(EEPROM.read(EEPROM_PRESSURE_ERROR)?F("TRUE"):F("FALSE"));
  
  Serial.print(F("\t\tDefault case error:\t"));
  Serial.println(EEPROM.read(EEPROM_DEFAULT_CASE_ERROR)?F("TRUE"):F("FALSE"));
  
  Serial.print(F("\t\tThrust state:\t\t"));
  Serial.println(EEPROM.read(EEPROM_LOG_THRUST)?F("TRUE"):F("FALSE"));
  
  Serial.print(F("\t\tApogee timeout:\t\t"));
  Serial.println(EEPROM.read(EEPROM_LOG_APOGEE_TIMEOUT)?F("TRUE"):F("FALSE"));
  
  Serial.print(F("\t\tTumble state:\t\t"));
  Serial.println(EEPROM.read(EEPROM_LOG_TUMBLE)?F("TRUE"):F("FALSE"));
  
  Serial.print(F("\t\tParachute state:\t"));
  Serial.println(EEPROM.read(EEPROM_LOG_PARACHUTE)?F("TRUE"):F("FALSE"));
  
  Serial.print(F("\t\tTouchdown state:\t"));
  Serial.println(EEPROM.read(EEPROM_LOG_TOUCHDOWN)?F("TRUE"):F("FALSE"));
  
  Serial.print(F("\t\tFinal EEPROM write:\t"));
  Serial.println(EEPROM.read(EEPROM_LOG_FINALEEPROMWRITE)?F("TRUE"):F("FALSE"));
  
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
  Serial.print(tempLong/1000.0);
  Serial.println(F(" sec"));

  Serial.print(F("\t\tTime to deployment:\t"));
  EEPROM.get(EEPROM_DEPLOYMENT_TIME, tempLong);
  Serial.print(tempLong/1000.0);
  Serial.println(F(" sec"));

  Serial.print(F("\t\tTime to touchdown:\t"));
  EEPROM.get(EEPROM_TOUCHDOWN_TIME, tempLong);
  Serial.print(tempLong/1000.0);
  Serial.println(F(" sec"));
  
  Serial.println();
}


void clearEEPROM(){
    for (int i = 0; i <= EEPROM_MAX_LENGTH; i++) {
        EEPROM.write(i, 0);
    }
}


void writeAltToEEPROM(){
  if (minAlt != writtenMinAlt){
    EEPROM.put(EEPROM_MIN_ALT, minAlt);
    writtenMinAlt = minAlt;
  }
  if (maxAlt != writtenMaxAlt){
    EEPROM.put(EEPROM_MAX_ALT, maxAlt);
    writtenMaxAlt = maxAlt;
  }
}


double getAccelerationScalar(){
  int16_t ax, ay, az;
  double dax, day, daz;
  mpu6050.getAcceleration(&ax, &ay, &az);
  dax = ax*ACCELEROMETER_MULTIPLIER;
  day = ay*ACCELEROMETER_MULTIPLIER;
  daz = az*ACCELEROMETER_MULTIPLIER;
  return sqrt((dax*dax) + (day*day) + (daz*daz));
}


double getAltitude(){
  //Value in meters, relative to baseline
  double P;
  P = getPressure();
  return pressure.altitude(P,baseline);
}


double getPressure(){
  char status;
  double T,P;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0){
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0){
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0){
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0){
          return(P);
        }else{
          SerialDebugMsg(F("error retrieving pressure measurement(1)\n"));
          log_pressure_error = true;
          EEPROM.write(EEPROM_PRESSURE_ERROR, 1);
        }
      }else{
        SerialDebugMsg(F("error starting pressure measurement(2)\n"));
        log_pressure_error = true;
        EEPROM.write(EEPROM_PRESSURE_ERROR, 1);
      }
    }else{
      SerialDebugMsg(F("error retrieving temperature measurement(1)\n"));
      log_pressure_error = true;
      EEPROM.write(EEPROM_PRESSURE_ERROR, 1);
    }
  }else{
    SerialDebugMsg(F("error starting temperature measurement(2)\n"));
    log_pressure_error = true;
    EEPROM.write(EEPROM_PRESSURE_ERROR, 1);
  }
  return 0.0;
}
