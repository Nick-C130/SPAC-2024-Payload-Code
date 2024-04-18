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
*/


/*
To do
Test SD card
PID autotune
Erranious value detection
*/

#include "Defines.h"
#include "JrkG2.h"
#include "Autotune.h"

unsigned long timeInterval = 1000;  // Initial interval between sensor readings (ms)
unsigned long previousTime = 0;     // Initial time variable (ms)
const float targetAltitude = 2500;  // Extra condition to run main script. Underestimate of expected apogee (m)
bool experimentPrimed = false;      // Actuator off until true
bool sdAv = true;

unsigned long currentTime;      // Variable for time (ms)
static float previousAltitude;  // Sets variable for previous altitude (m)
float altitude;                 // Reads altitude (check sensor) (m)
unsigned long deltaTime;        // Calculates time difference (s)
float actualVelocity;           // Velocity read by PCB pressure sensor (m/s)
float pressureRocket;           // Reads altitude (check which sensor) (hPa)
float temperatureRocket;        // Reads chamber's temperature (C)
//float densityRocket;             // Calculates air density in rocket (kg/m^3)
float pressureChamber;     // Reads chamber's pressure (hPa)
float temperatureChamber;  // Reads chamber's temperature (C)
float actuatorHeight;      // Reads acuator's height (mm)

static float Pi = M_PI;    // Pi
float r = 0.02;            // Pressure chamber radius (m)
float A = Pi * pow(r, 2);  // Area (m^2)
float hChamber;            // Height for volume in chamber (m)
float V;                   // Volume (m^3)
int n;                     // Moles
float R = 8.314;           // Ideal gas constant  (J/K/mol)
float T;                   // Temperature (K)

// PID Values
uint16_t proportionalCoef = 10;
uint16_t proportionalExpo = 0;
uint16_t integralCoef = 819;
uint16_t integralExpo = 13;
uint16_t derivativeCoef = 5;
uint16_t derivativeExpo = 0;
uint16_t PIDPeriod = 30;
uint16_t integralLimit = 6000;

JrkG2I2C jrk;           // Motor Controller
Adafruit_BMP280 Atmos;  // First Chamb80 sensor at address 0x76
Adafruit_BMP280 Chamb;  // Second Chamb80 sensor at address 0x77

const int chipSelect = BUILTIN_SDCARD;  // Initialises SD card
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
  int byte1 = grad * mm1;
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
  int mm2 = byte2 / grad;
  return mm2;
}

// Function to calculate initial moles
int moles(float V, float temperatureChamber, float hChamber, float pressureChamber) {
  float num = A * hChamber * pressureChamber;
  float den = R * temperatureChamber;
  n = (int)(num / den);
  return n;
}

//Density outside chamber
float denRocket(float temperatureRocket, float pressureRocket) {
  float densityRocket = pressureRocket / (R * temperatureRocket);
  return densityRocket;
}

// Function to calculate height in chamber
float nextHeight(float V, float temperatureChamber, int n, float pressureRocket) {
  float num = (float)n * R * temperatureChamber;
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

  digitalWrite(motorGate, LOW);  // Turns actuator on, LED 3 on
  digitalWrite(valveGate, LOW);  // Opens air valve, LED 4 on

  delay(1000);

  // Set PID Values
  jrk.setProportionalCoefficient(proportionalCoef, proportionalExpo);
  jrk.setIntegralCoefficient(integralCoef, integralExpo);
  jrk.setDerivativeCoefficient(derivativeCoef, derivativeExpo);
  jrk.setPIDPeriod(PIDPeriod);
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
      dataFile.println("Timestamp (ms),Altitude (m),Measured Velocity (m/s),Atmos_P (hPa),Chamb_P (hPa),Chamb_T (C), Actuator Height (mm)");  //Header Row
      dataFile.close();
    } else {
      digitalWrite(BUILTIN_LED, HIGH);
    }
  }

  unsigned int status1;
  unsigned int status2;

  // Check and initialize the first Chamb80 sensor at address 0x76
  status1 = Atmos.begin(0x77);
  if (!status1) {
    digitalWrite(LED1, HIGH);  // If the Atmos isn't detected LED 1 on
    while (1) delay(10);
  }
  Atmos.setSampling(Atmos.MODE_NORMAL,Atmos.SAMPLING_X16,Atmos.SAMPLING_X16,Atmos.FILTER_X16,Atmos.STANDBY_MS_1);

  // Check and initialize the second Chamb80 sensor at address 0x77
  status2 = Chamb.begin(0x76);
  if (!status2) {
    digitalWrite(LED2, HIGH);  // If the Chamb isn't detected LED 2 on
    while (1) delay(10);
  }
  Chamb.setSampling(Chamb.MODE_NORMAL,Chamb.SAMPLING_X16,Chamb.SAMPLING_X16,Chamb.FILTER_X16,Chamb.STANDBY_MS_1);

  delay(1000);

  pressureChamber = Chamb.readPressure();            // Reads chamber pressure (check sensor)
  temperatureChamber = Chamb.readTemperature();      // Reads chamber pressure (check sensor)
  actuatorHeight = byteToHeight(jrk.getFeedback());  // Reads actuator height

  digitalWrite(motorGate, LOW);  // Turns actuator off, LED 3 off
  digitalWrite(valveGate, LOW);  // Opens air valve, LED 4 off

  attachInterrupt(digitalPinToInterrupt(testPin), Test, HIGH);  // Assigns test function to test pin
}

void loop() {
  SerialCMDHandle();
  currentTime = millis();                              // Reads current time since boot
  if (currentTime - previousTime >= timeInterval) {    // Loop to read data
    altitude = Atmos.readAltitude();                   // Reads altitude (check sensor)
    pressureRocket = Atmos.readPressure();             // Reads rocket pressure (check sensor)
    pressureChamber = Chamb.readPressure();            // Reads chamber pressure (check sensor)
    temperatureChamber = Chamb.readTemperature();      // Reads chamber pressure (check sensor)
    actuatorHeight = byteToHeight(jrk.getFeedback());  // Reads actuator height

    unsigned long deltaTime = (currentTime - previousTime) / 100;             // Calculates time difference (seconds)
    float actualVelocity = (altitude - previousAltitude) / (float)deltaTime;  // Calculates velocity (m/s)
  }
  if (altitude >= targetAltitude) {
    timeInterval = 10;
  }
  if (!experimentPrimed) {  // until conditions met don't run experiement
    if (altitude >= targetAltitude && actualVelocity < 0) {
      // Conditions are met, set the flag, change time interval
      experimentPrimed = true;
    }
  } else {
    V = (absMax - actuatorHeight) * A;
    Controller();
  }

  static float previousTime = currentTime;  // Iterates for next loop
  static float previousAltitude = altitude;
  DataSave();  // Saves data
}

void Controller() {
  digitalWrite(valveGate, LOW);  // Opens Closes valve, LED 4 on
  jrk.setTarget(heightToByte(nextHeight(V, temperatureChamber, n, pressureRocket)));

  return;
}

void DataSave() {
  dataFile = SD.open(currentFileName.c_str(), FILE_WRITE);
  if (dataFile) {
    dataFile.print(currentTime);
    dataFile.print(",");  // Time since power (milliseconds)
    dataFile.print(altitude);
    dataFile.print(",");  // Altitude (m)
    dataFile.print(actualVelocity);
    dataFile.print(",");  // Rocket velocity (m/s)
    dataFile.print(pressureRocket);
    dataFile.print(",");  // Pressure at PCB (hPa)
    dataFile.print(pressureChamber);
    dataFile.print(",");  // Pressure in chamber (hPa)
    dataFile.print(temperatureChamber);
    dataFile.print(",");  // Temperature in chamber (C)
    dataFile.print(actuatorHeight);
    dataFile.print(",");  // Pressure in chamber (hPa)
    dataFile.println();

    dataFile.close();  // Close the file
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
              char numBuff[3] = {buffer[3],buffer[4],buffer[5]};
              float set = strtol(numBuff,NULL,10)/10;
              char buff[50];
              sprintf(buff,"Going to %f", set);
              Serial.println(buff);
              jrk.setTarget(heightToByte(set));
            }
            break;
        }
    }
    clearArray(buffer, 100);
    index = 0;
  }
  return;
}

void clearArray(char *Array, uint8_t len) {
  while (len != 0) {
    Array[len - 1] = 0;
    len--;
  }
  return;
}