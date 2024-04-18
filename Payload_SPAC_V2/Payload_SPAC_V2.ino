/* 
V1 14/02/24
BMP sensors, SD card, and Startup trigger setup

V2 17/02/24
Start conditions, Logic statements

V3 19/02/24
Data saving, Test script

V4 23/02/24
Height to Byte function, Byte to Height function, start, min, max height settings

V5 13/04/24
JRK set to wire2, Multifile, PID integration. Functions for chamber pressure

V6 18/04/24
Nick got a hold of it
*/


/*
To do
Test SD card
Erranious value detection
*/

#include "Defines.h"
#include "JrkG2.h"

unsigned long timeInterval = standbyTime;  // Initial interval between sensor readings (ms)
unsigned long previousTime = 0;            // Initial time variable (ms)
bool experimentPrimed = false;             // Actuator off until true
bool sdAv = true;                          // Is there an SD card

unsigned long currentTime;  // Variable for time (ms)
float previousAltitude;     // Sets variable for previous altitude (m)
float altitude = 0;         // Reads altitude (check sensor) (m)
unsigned long deltaTime;    // Calculates time difference (s)
float actualVelocity;       // Velocity read by PCB pressure sensor (m/s)
float pressureRocket;       // Reads altitude (check which sensor) (hPa)
float temperatureRocket;    // Reads chamber's temperature (C)
//float densityRocket;      // Calculates air density in rocket (kg/m^3)
float pressureChamber;      // Reads chamber's pressure (hPa)
float temperatureAtmos;     // Reads the Atmos temp (c)
float temperatureChamber;   // Reads chamber's temperature (C)
float actuatorHeight;       // Reads acuator's height (mm)

float hChamber;   // Height for volume in chamber (m)
float V;          // Volume (m^3)
int n;            // Moles
float R = 8.314;  // Ideal gas constant  (J/K/mol)
float T;          // Temperature (K)

JrkG2I2C jrk;           // Motor Controller
Adafruit_BMP280 Atmos;  // First BMP280 sensor at address 0x76
Adafruit_BMP280 Chamb;  // Second BMP280 sensor at address 0x77

const int chipSelect = BUILTIN_SDCARD;  // SD card CS pin
File dataFile;                          // Sets a data file
String currentFileName;                 // For multiple files

// Creates a new csv on each boot
String getNextFileName() {
  int fileNumber = 0;
  while (true) {
    String fileName = "Sensor_Data" + String(fileNumber) + ".csv";
    if (!SD.exists(fileName.c_str())) {
      return fileName;
    }
    fileNumber++;
  }
}

// Function to work in mm for actuator
int heightToByte(float mm1) {
  int byte1 = actuatorScale * mm1;
  if (mm1 > maxHeight) {
    mm1 = maxHeight;
  }
  if (mm1 < minHeight) {
    mm1 = minHeight;
  }
  return byte1;
}

// Function to convert feedback to a height
float byteToHeight(int byte2) {
  int mm2 = byte2 / actuatorScale;
  return mm2;
}

// Function to calculate initial moles
float moles() {
  n=pressureChamber*V/r*temperatureChamber;
  return(n);
}

// Function to calculate height in chamber
float nextHeight() {
  float num = n * R * temperatureChamber;
  float den = pressureRocket * A;
  hChamber = num / den;

  return hChamber;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();   // BMP sensors
  Wire2.begin();  // Motor controller

  // Sets test LEDs and Mosfet pins as outputs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(motorGate, OUTPUT);
  pinMode(valveGate, OUTPUT);

  digitalWrite(motorGate, LOW);   // Turns actuator off, LED 3 on
  digitalWrite(valveGate, HIGH);  // Opens air valve, LED 4 on

  delay(1000);

  // Set PID Values
  jrk.setProportionalCoefficient(proportionalCoef, proportionalExpo);
  jrk.setIntegralCoefficient(integralCoef, integralExpo);
  jrk.setDerivativeCoefficient(derivativeCoef, derivativeExpo);
  jrk.setPIDPeriod(PIDControlPeriod);
  jrk.setIntegralLimit(integralLimit);

  jrk.setTarget(heightToByte(startHeight));
  delay(1000);

  if (!SD.begin(chipSelect)) {        // Initial SD card check
    digitalWrite(BUILTIN_LED, HIGH);  // If the SD isn't detected the built in LED will light up
    sdAv = false;
  }

  currentFileName = getNextFileName();
  if (sdAv) {
    dataFile = SD.open(currentFileName.c_str(), FILE_WRITE);  // Initialize the dataFile object
    if (dataFile) {
      dataFile.println("Timestamp (ms),Altitude (m),Measured Velocity (m/s),Atmos_P (hPa),Chamb_P (hPa),Atmos_T (C),Chamb_T (C), Actuator Height (mm)");  //Header Row
      dataFile.close();
    } else {
      digitalWrite(BUILTIN_LED, HIGH);
    }
  }

  int status;
  // Check and initialize the first Chamb80 sensor at address 0x76
  status = Atmos.begin(0x77);
  if (!status) {
    digitalWrite(LED1, HIGH);  // If the Atmos isn't detected LED 1 on
    char buff[20];
    sprintf(buff, "Atmos BP Error: %i", status);
    Serial.println(buff);
    while (1) delay(10);
  }
  Atmos.setSampling(Atmos.MODE_NORMAL, Atmos.SAMPLING_X16, Atmos.SAMPLING_X16, Atmos.FILTER_X16, Atmos.STANDBY_MS_1);

  // Check and initialize the second Chamb80 sensor at address 0x77
  status = Chamb.begin(0x76);
  if (!status) {
    digitalWrite(LED2, HIGH);  // If the Chamb isn't detected LED 2 on
    char buff[20];
    sprintf(buff, "Atmos BP Error: %i", status);
    Serial.println(buff);
    while (1) delay(10);
  }
  Chamb.setSampling(Chamb.MODE_NORMAL, Chamb.SAMPLING_X16, Chamb.SAMPLING_X16, Chamb.FILTER_X16, Chamb.STANDBY_MS_1);

  attachInterrupt(digitalPinToInterrupt(testPin), Test, HIGH);  // Assigns test function to test pin
  previousTime = millis();
  previousAltitude = Atmos.readAltitude();
}

void loop() {
  SerialCMDHandle();
  currentTime = millis();                              
  if (currentTime - previousTime >= timeInterval) {    
    altitude = Atmos.readAltitude();                   
    pressureRocket = Atmos.readPressure();             
    pressureChamber = Chamb.readPressure();            
    temperatureAtmos = Atmos.readTemperature();
    temperatureChamber = Chamb.readTemperature();      
    actuatorHeight = byteToHeight(jrk.getFeedback());  

    unsigned long deltaTime = (currentTime - previousTime) / 100;             // Calculates time difference (seconds)
  }
  actualVelocity = (altitude - previousAltitude) / deltaTime;               // Calculates velocity (m/s)
  if (altitude >= targetAltitude) {  //If over height to confirm launch
    timeInterval = activeTime;
  }actualVelocity = (altitude - previousAltitude) / deltaTime;               // Calculates velocity (m/s)
  if (experimentPrimed) {  // until conditions met don't run experiement
    V = (absMax - actuatorHeight) * A;
    Controller();
  }
  else {
    if ((altitude >= targetAltitude) && (actualVelocity < 0)) {
      // Conditions are met, set the flag
      experimentPrimed = true;
    }
  }

  previousTime = currentTime;  // Iterates for next loop
  previousAltitude = altitude;
  DataSave();  // Saves data
}

void Controller() {
  digitalWrite(valveGate, LOW);  // Closes valve, LED 4 on
  jrk.setTarget(heightToByte(nextHeight()));

  return;
}

void DataSave() {
  if (sdAv) {
    dataFile = SD.open(currentFileName.c_str(), FILE_WRITE);
    if (dataFile) {
      char buff[255];
      sprintf(buff,"%g , %g , %g , %g , %g , %g , %g , %g \n",currentTime,altitude,actualVelocity,pressureRocket,pressureChamber,temperatureAtmos,temperatureChamber,actuatorHeight);
      dataFile.print(buff);  // Pressure in chamber (hPa)
      dataFile.close();  // Close the file
    }
  }
}

void Test() {
  cli();

  digitalWrite(BUILTIN_LED, HIGH);  // BUILTIN LED 1 on
  digitalWrite(LED1, HIGH);         // LED 1 on
  digitalWrite(LED2, HIGH);         // LED 2 on
  digitalWrite(motorGate, HIGH);    // Turns actuator on, LED 3 on
  digitalWrite(valveGate, HIGH);    // Closes valve, LED 4 on

  // Tests actuator
  jrk.setTarget(heightToByte(100));
  _delay_us(2000000);
  jrk.setTarget(heightToByte(0));
  _delay_us(2000000);
  jrk.setTarget(heightToByte(startHeight));
  _delay_us(2000000);

  // Ends test
  digitalWrite(BUILTIN_LED, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(valveGate, LOW);
  digitalWrite(motorGate, LOW);

  _delay_us(2000000);

  sei();
}

void clearArray(char *Array, uint8_t len) {
  while (len != 0) {
    Array[len - 1] = 0;
    len--;
  }
  return;
}

void SerialCMDHandle() {
  static uint8_t index;
  static char buffer[100];
  bool command = false;

  if (Serial.available()) {
    buffer[index] = Serial.read();
    Serial.print(buffer[index]);
    index++;
    if (index == 100) {
      index = 0;
      clearArray(buffer, 100);
    }
  }

  uint8_t i = 0;
  while (i < 100) {
    if (buffer[i] == '\r' || buffer[i] == '\n') {
      command = true;
    }
    i++;
  }

  if (command) {
    switch (buffer[0]) {
      case 'M':  //MOSFET
        switch (buffer[1]) {
          case 'V':
            {  //VALVE
              char byte = buffer[3];
              if (byte == '0') {
                digitalWrite(valveGate, LOW);
                Serial.println("Valve Off");
              } else if (byte == '1') {
                digitalWrite(valveGate, HIGH);
                Serial.println("Valve On");
              }
              break;
            }
          case 'M':
            {  //VALVE
              char byte = buffer[3];
              if (byte == '0') {
                digitalWrite(motorGate, LOW);
                Serial.println("Motor Off");
              } else if (byte == '1') {
                digitalWrite(motorGate, HIGH);
                Serial.println("Motor On");
              }
            }
            break;
        }
      case 'S':  //Sensors
        switch (buffer[1]) {
          case 'A':
            {  //Atmospheric
              char buff[100];
              sprintf(buff, "Alt: %f, Press: %f, Temp %f \n", Atmos.readAltitude(), Atmos.readPressure(), Atmos.readTemperature());
              Serial.print(buff);
            }
            break;
          case 'C':
            {  //Chamber
              char buff[100];
              sprintf(buff, "Alt: %f, Press: %f, Temp %f \n", Chamb.readAltitude(), Chamb.readPressure(), Chamb.readTemperature());
              Serial.print(buff);
            }
            break;
          case 'M':
            {
              char buff[100];
              sprintf(buff, "Height: %f \n", byteToHeight(jrk.getFeedback()));
              Serial.print(buff);
            }
            break;
        }
      case 'A':  //Actuator
        switch (buffer[1]) {
          case 'S':
            {  //Set input of mm*10
              char numBuff[3] = { buffer[3], buffer[4], buffer[5] };
              float set = strtol(numBuff, NULL, 10) / 10;
              char buff[50];
              sprintf(buff, "Going to %f", set);
              Serial.println(buff);
              jrk.setTarget(heightToByte(set));
            }
            break;
        }
      case 'T':
        {
          Autotune();
        }
        break;
    }
  }
  clearArray(buffer, 100);
  index = 0;
  return;
}

void Autotune() {
  PIDAutotuner tuner = PIDAutotuner();
  tuner.setTargetInputValue(tuneVal);
  tuner.setLoopInterval(PIDControlPeriod);
  tuner.setOutputRange(minHeight, maxHeight);
  tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);
  tuner.startTuningLoop(micros());

  long microseconds;
  while (!tuner.isFinished()) {
    microseconds = micros();
    double input = byteToHeight(jrk.getFeedback());
    double output = tuner.tunePID(input, microseconds);
    jrk.setTarget(heightToByte(output));
    while (micros() - microseconds < PIDControlPeriod) delayMicroseconds(1);
  }
  char buff[50];
  sprintf(buff, "Kp %f, Ki %f, Kd %f", tuner.getKp(), tuner.getKi(), tuner.getKd());
  Serial.println(buff);
}