/*
To do
Erranious value detection
AP Command doesn't always update
*/

#include "Defines.h"
#include "JrkG2.h"

bool experimentPrimed = false;  // Actuator off until true
bool dataSavePrimed = false;

bool sdAv = true;               // Is there an SD card

unsigned long currentTime;          // Variable for time (ms)
unsigned long previousTime = 0;     // Initial time variable (ms)
unsigned long timeInterval = 1000;  // Initial interval between sensor readings (ms)
unsigned long deltaTime;            // Calculates time difference (s)

float actualVelocity;  // Velocity as calculated by barometer (m/s)

float altitude = 0;        // Reads altitude (check sensor) (m)
float previousAltitude;    // Sets variable for previous altitude (m)
float pressureAtmos;       // Reads chamber's pressure (Pa)
float pressureChamber;     // Reads chamber's pressure (Pa)
float temperatureAtmos;    // Reads the Atmos temp (C)
float temperatureChamber;  // Reads chamber's temperature (C)

sensors_event_t a, g, temp;  // Struct for MPU data

float accelX;  // Reads accelaration in X axis
float accelY;  // Reads accelaration in Y axis
float accelZ;  // Reads accelaration in Z axis

// float gyroX;  // Reads rotation in X axis
// float gyroY;  // Reads rotation in Y axis
// float gyroZ;  // Reads rotation in Z axis

float actuatorHeight;   // Reads acuator's height (mm)
float actuatorVoltage;  // Reads actuator's input volatage (mV)
float actuatorCurrent;  // Reads actuator's current draw (mA)

float hChamber;   // Height for volume in chamber (m)
float hPiston;    // Height the piston needs to be to achieve hChamber
float V;          // Volume (m^3)
int n;            // Moles
float R = 8.314;  // Ideal gas constant  (J/K/mol)
float T;          // Temperature (K)

JrkG2I2C jrk;           // Motor Controller
Adafruit_BMP280 Atmos;  // First BMP280 sensor at address 0x76
Adafruit_BMP280 Chamb;  // Second BMP280 sensor at address 0x77
Adafruit_MPU6050 MPU;   // IMU sensor at address 0x68

const int chipSelect = BUILTIN_SDCARD;  // SD card CS pin
File dataFile;                          // Sets a data file
String currentDataFileName;             // File for Data
String currentEventFileName;            // For for Event Logs

// Struct for reading/writing PID values
struct PIDVals {
  int kp;
  int kpe;
  int ki;
  int kie;
  int kd;
  int kde;
};

// Creates a new csv on each boot
String getNextFileName(String name, String type) {
  int fileNumber = 0;
  while (true) {
    String fileName = name + String(fileNumber) + type;
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
  n = pressureChamber * V / r * (temperatureChamber + 273.15);
  return (n);
}

// Function to calculate height in chamber
float nextHeight() {
  float num = n * R * (temperatureChamber + 273.15);
  float den = pressureAtmos * A;
  hChamber = num / den;
  hPiston = absMax - hChamber;

  return hPiston;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();   // BMP + IMU sensors
  Wire2.begin();  // Motor controller

  // Sets test LEDs and MOSFET pins as outputs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(motorGate, OUTPUT);
  pinMode(valveGate, OUTPUT);

  digitalWrite(motorGate, LOW);  // Turns actuator off, LED 3 on
  digitalWrite(valveGate, LOW);  // Opens air valve, LED 4 on

  delay(1000);

  // Set PID Values
  PIDVals send;
  EEPROM.get(0, send);
  if (send.kp == -1) {
    send = { proportionalCoef, proportionalExpo, integralCoef, integralExpo, derivativeCoef, derivativeExpo };
  }
  sendPID(send);

  jrk.setPIDPeriod(PIDControlPeriod);
  jrk.setIntegralLimit(integralLimit);

  jrk.setTarget(heightToByte(startHeight));
  delay(1000);

  if (!SD.begin(chipSelect)) {        // Initial SD card check
    digitalWrite(BUILTIN_LED, HIGH);  // If the SD isn't detected the built in LED will light up
    sdAv = false;
  }

  currentDataFileName = getNextFileName("Sensor Data", ".csv");
  if (sdAv) {
    dataFile = SD.open(currentDataFileName.c_str(), FILE_WRITE);  // Initialize the dataFile object
    if (dataFile) {
      dataFile.println("Timestamp (ms),Altitude (m),Measured Velocity (m/s),Atmos_P (hPa),Chamb_P (hPa),Atmos_T (C),Chamb_T (C), AccelZ (g), Actuator Height (mm),Battery V (mV), Actuator Current (mA)");  //Header Row
      dataFile.close();
    } else {
      digitalWrite(BUILTIN_LED, HIGH);
    }
  }

  if (sdAv) {
    dataFile = SD.open(currentDataFileName.c_str(), FILE_WRITE);
    MPU.getEvent(&a, &g, &temp);
    if (dataFile) {
      char buff[255];
      sprintf(buff, "%xl , %.2f , %.2f , %f , %f , %.2f, %.2f , %.2f , %.1f ,  \n", 0, Atmos.readAltitude(), 0, Atmos.readPressure(), Chamb.readPressure(), Atmos.readTemperature(), Chamb.readTemperature(), a.acceleration.z, byteToHeight(jrk.getScaledFeedback()), jrk.getVinVoltage(), jrk.getCurrent());
      dataFile.print(buff);  // Pressure in chamber (hPa)
      dataFile.close();      // Close the file
    }
  }

  currentEventFileName = getNextFileName("Events", ".txt");
  if (sdAv) {
    dataFile = SD.open(currentEventFileName.c_str(), FILE_WRITE);  // Initialize the dataFile object
    if (dataFile) {
      dataFile.println("Powered On");  //Header Row
      dataFile.close();
    } else {
      digitalWrite(BUILTIN_LED, HIGH);
    }
  }

  delay(2000);
  int status;
  // Check and initialize the first Chamb80 sensor at address 0x76
  status = Atmos.begin(0x77);
  if (status != 1) {
    digitalWrite(LED1, HIGH);  // If the Atmos isn't detected LED 1 on
    char buff[20];
    sprintf(buff, "Atmos BP Error: %i", status);
    Serial.println(buff);
    //while (1) delay(10);
  }
  Atmos.setSampling(Atmos.MODE_NORMAL, Atmos.SAMPLING_X16, Atmos.SAMPLING_X16, Atmos.FILTER_OFF, Atmos.STANDBY_MS_1);

  // Check and initialize the second Chamb80 sensor at address 0x77
  status = Chamb.begin(0x76);
  if (status != 1) {
    digitalWrite(LED2, HIGH);  // If the Chamb isn't detected LED 2 on
    char buff[20];
    sprintf(buff, "Chamber BP Error: %i", status);
    Serial.println(buff);
    //while (1) delay(10);
  }
  Chamb.setSampling(Chamb.MODE_NORMAL, Chamb.SAMPLING_X16, Chamb.SAMPLING_X16, Chamb.FILTER_OFF, Chamb.STANDBY_MS_1);

  status = MPU.begin(0x68);
  if (status != 1) {
    digitalWrite(LED3, HIGH);  // If the MPU isn't detected LED 3 on
    char buff[20];
    sprintf(buff, "MPU Error: %i", status);
    Serial.println(buff);
    //while (1) delay(10);
  }
  MPU.setAccelerometerRange(MPU6050_RANGE_16_G);
  // MPU.setGyroRange(MPU6050_RANGE_500_DEG);
  // MPU.setFilterBandwidth(MPU6050_BAND_21_HZ);


  attachInterrupt(digitalPinToInterrupt(testPin), Test, HIGH);  // Assigns test function to test pin
  previousTime = millis();
  previousAltitude = Atmos.readAltitude();

  EventLog("Setup Completed");
}

void loop() {
  SerialCMDHandle();  // Allows serial inputs for commands
  currentTime = millis();
  altitude = Atmos.readAltitude();
  pressureAtmos = Atmos.readPressure();
  pressureChamber = Chamb.readPressure();
  temperatureAtmos = Atmos.readTemperature();
  temperatureChamber = Chamb.readTemperature();
  actuatorHeight = byteToHeight(jrk.getScaledFeedback());
  actuatorVoltage = jrk.getVinVoltage();
  actuatorCurrent = jrk.getCurrent();

  MPU.getEvent(&a, &g, &temp);
  accelZ = a.acceleration.z;

  if ((accelZ > launchAccel) || (accelZ < -launchAccel)) { 
    dataSavePrimed = true;
    EventLog("Data Saving Commenced");
  }

  if (currentTime - previousTime >= timeInterval) {
    deltaTime = (currentTime - previousTime) / 100;  // Calculates time difference (seconds)

    previousTime = currentTime;  // Iterates for next loop
    previousAltitude = altitude;
  }

  actualVelocity = (altitude - previousAltitude) / deltaTime;  // Calculates velocity (m/s)

  if (experimentPrimed) {  // until conditions met don't run experiement
    timeInterval = activeTime;

    V = (absMax - actuatorHeight) * A;
    Controller();
  } else {
    if ((altitude >= targetAltitude) && (actualVelocity < targetVelocity)) {
      EventLog("Experiment Primed");
      experimentPrimed = true;
      n = moles();
    }
  }

  DataSave();  // Saves data
}

void Controller() {
  digitalWrite(valveGate, HIGH);  // Closes valve, LED 4 on
  if ((pressureChamber > (pressureAtmos + maxPressureDiffernce)) || (pressureChamber < (pressureAtmos + maxPressureDiffernce))) {
    jrk.setTarget(heightToByte(nextHeight()));
  }

  return;
}

void DataSave() {
  if (dataSavePrimed) {
    if (sdAv) {
      dataFile = SD.open(currentDataFileName.c_str(), FILE_WRITE);
      if (dataFile) {
        char buff[255];
        sprintf(buff, "%xl , %.2f , %.2f , %f , %f , %.2f, %.2f , %.2f , %.1f ,  \n", currentTime, altitude, actualVelocity, pressureAtmos, pressureChamber, temperatureAtmos, temperatureChamber, accelZ, actuatorHeight, actuatorVoltage, actuatorCurrent);
        dataFile.print(buff);  // Pressure in chamber (hPa)
        dataFile.close();      // Close the file
      }
    }
  }
  return;
}

void EventLog(String event) {
  if (sdAv) {
    dataFile = SD.open(currentEventFileName.c_str(), FILE_WRITE);
    if (dataFile) {
      dataFile.println(event);  //Header Row
      dataFile.close();         // Close the file
    }
  }
  return;
}

void Test() {
  cli();

  EventLog("Test Ran");

  digitalWrite(BUILTIN_LED, HIGH);  // BUILTIN LED 1 on
  digitalWrite(LED1, HIGH);         // LED 1 on
  digitalWrite(LED2, HIGH);         // LED 2 on
  digitalWrite(LED3, HIGH);         // LED 3 on
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
  digitalWrite(LED3, LOW);
  digitalWrite(valveGate, LOW);
  digitalWrite(motorGate, LOW);

  _delay_us(2000000);

  sei();
  return;
}

void clearArray(char *Array, uint8_t len) {
  while (len != 0) {
    Array[len - 1] = 0;
    len--;
  }
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
    double input = byteToHeight(jrk.getScaledFeedback());
    double output = tuner.tunePID(input, microseconds);
    jrk.setTarget(heightToByte(output));
    while (micros() - microseconds < PIDControlPeriod) delayMicroseconds(1);
  }

  int kpe = getExpo(tuner.getKp());
  int kp = tuner.getKp() * pow(2, kpe);
  int kie = getExpo(tuner.getKi());
  int ki = tuner.getKi() * pow(2, kie);
  int kde = getExpo(tuner.getKd());
  int kd = tuner.getKd() * pow(2, kde);

  PIDVals Tuned = { kp, kpe, ki, kie, kd, kde };
  EEPROM.put(0, Tuned);
  sendPID(Tuned);

  char buff[50];
  sprintf(buff, "Kp %f, Ki %f, Kd %f", tuner.getKp(), tuner.getKi(), tuner.getKd());
  Serial.println(buff);
  EventLog("PID Tuned");
  return;
}

int getExpo(float value) {
  int e = 1;
  int multi;
  bool finished = false;
  while (!finished) {
    multi = value * pow(2, e);
    if (multi > 1023) {
      e++;
    }
    if (multi == 0) {
      e--;
    }
    if (e == 18) {
      finished = true;
    }
    if (e == 0) {
      finished = true;
    }
    if (multi > 0) {
      if (multi < 1023) {
        finished = true;
        break;
      }
    }
  }

  return e;
}

void sendPID(struct PIDVals setVals) {

  jrk.setProportionalCoefficient(setVals.kp, setVals.kpe);
  jrk.setIntegralCoefficient(setVals.ki, setVals.kie);
  jrk.setDerivativeCoefficient(setVals.kd, setVals.kde);
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
      Serial.println("Buffer Overflow");
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
            }
            break;
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
              break;
            }
        }
      case 'S':  //Sensors
        switch (buffer[1]) {
          case 'A':
            {  //Atmospheric
              char buff[100];
              sprintf(buff, "Alt: %f m| Press: %f Pa| Temp %f C \n", Atmos.readAltitude(), Atmos.readPressure(), Atmos.readTemperature());
              Serial.print(buff);
            }
            break;
          case 'C':
            {  //Chamber
              char buff[100];
              sprintf(buff, "Alt: %f m| Press: %f Pa| Temp %f C \n", Chamb.readAltitude(), Chamb.readPressure(), Chamb.readTemperature());
              Serial.print(buff);
            }
            break;
          case 'M':
            {  //Motor Controller
              char buff[100];
              sprintf(buff, "Pos: %f mm| Vin: %u mV| Current: %u mA \n", byteToHeight(jrk.getScaledFeedback()), jrk.getVinVoltage(), jrk.getRawCurrent());
              Serial.print(buff);
            }
            break;
          case 'I':
            {  //MPU6050
              MPU.getEvent(&a, &g, &temp);
              char buff[100];
              sprintf(buff, "Ax: %f | Ay: %f | Az %f \n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
              Serial.print(buff);

              sprintf(buff, "Gx: %f | Gy: %f | Gz %f \n", g.gyro.x, g.gyro.y, g.gyro.z);
              Serial.print(buff);
            }
            break;
          case 'R':
            {  //Reset
              Atmos.reset();
              Chamb.reset();
              MPU.reset();
              Serial.print("Sensors Reset");
            }
            break;
          case 'O':
            {  //All Data
              {
                char buff[255];

                // First line
                sprintf(buff, "Current Time: %lu ms| Delta Time: %lu s| Altitude: %.2f m", currentTime, deltaTime, altitude);
                Serial.println(buff);

                // Second line
                sprintf(buff, "Velocity: %.2f m/s| Pressure Atmos: %.2f Pa| Pressure Chamber: %.2f Pa", actualVelocity, pressureAtmos, pressureChamber);
                Serial.println(buff);

                // Third line
                sprintf(buff, "Temperature Atmos: %.1f C| Temperature Chamber: %.1f C| Actuator Height: %.1f mm", temperatureAtmos, temperatureChamber, actuatorHeight);
                Serial.println(buff);

                MPU.getEvent(&a, &g, &temp);

                // Fourth line
                sprintf(buff, "Ax: %f | Ay: %f | Az %f \n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
                Serial.print(buff);

                // Fifth line
                sprintf(buff, "Gx: %f | Gy: %f | Gz %f \n", g.gyro.x, g.gyro.y, g.gyro.z);
                Serial.print(buff);

                // Sixth line
                sprintf(buff, "Vin: %u mV| Current: %u mA \n", jrk.getVinVoltage(), jrk.getRawCurrent());
                Serial.print(buff);
              }
            }
            break;
          case 'P':
            {  //PID values
              char cbuff[100];
              sprintf(cbuff, "KP: %i KPE: %i KI: %i KIE: %i KD: %i KDE: %i\n", jrk.getProportionalMultiplier(), jrk.getProportionalExponent(), jrk.getIntegralMultiplier(), jrk.getIntegralExponent(), jrk.getDerivativeMultiplier(), jrk.getDerivativeExponent());
              Serial.print(cbuff);
            }
            break;
          case 'V':
            {  // Velocity
              char cbuff[100];
              sprintf(cbuff, "Altitude: %.2f m| Velocity: %.2f m/s \n", altitude, actualVelocity);
              Serial.print(cbuff);
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
              sprintf(buff, "Going to %.1f mm", set);
              Serial.println(buff);
              jrk.setTarget(heightToByte(set));
            }
            break;
          case 'T':
            {  //Autotune
              Autotune();
            }
            break;
          case 'P':
            {  // Swap between pretuned and autotuned PID values
              char byte = buffer[3];
              PIDVals send;
              if (byte == '0') {
                send = { proportionalCoef, proportionalExpo, integralCoef, integralExpo, derivativeCoef, derivativeExpo };
              } else if (byte == '1') {
                EEPROM.get(0, send);
              }
              sendPID(send);
              char cbuff[100];
              sprintf(cbuff, "KP: %i KPE: %i KI: %i KIE: %i KD: %i KDE: %i\n", jrk.getProportionalMultiplier(), jrk.getProportionalExponent(), jrk.getIntegralMultiplier(), jrk.getIntegralExponent(), jrk.getDerivativeMultiplier(), jrk.getDerivativeExponent());
              Serial.print(cbuff);
            }
            break;
        }
    }
    clearArray(buffer, 100);
    index = 0;
  }
  return;
}