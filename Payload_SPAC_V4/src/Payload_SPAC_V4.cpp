#include "Defines.h"
#include "JrkG2.h"

bool experimentPrimed = false; // Actuator off until true
bool dataSavePrimed = false;   // Data starts saving on launch

unsigned long currentTime;         // Variable for time (ms)
unsigned long previousTime = 0;    // Initial time variable (ms)
unsigned long timeInterval = 1000; // Initial interval between sensor readings (ms)
unsigned long deltaTime;           // Calculates time difference (s)

float actualVelocity; // Velocity as calculated by barometer (m/s)

float altitude;           // Reads altitude (check sensor) (m)
float previousAltitude;   // Sets variable for previous altitude (m)
float deltaAltitude;      // Change in height for velocity calculation (m)
float pressureAtmos;      // Reads chamber's pressure (Pa)
float pressureChamber;    // Reads chamber's pressure (Pa)
float pressureDifference; // Difference between Atmos and Chamber (Pa)
float temperatureAtmos;   // Reads the Atmos temp (C)
float temperatureChamber; // Reads chamber's temperature (C)

sensors_event_t a, g, temp; // Struct for MPU data

float accelZ; // Reads accelaration in Z axis

float actuatorHeight;  // Reads acuator's height (mm)
float actuatorVoltage; // Reads actuator's input volatage (mV)
float actuatorCurrent; // Reads actuator's current draw (mA)

double chamberPressure;
double cmdVel;
double atmosPressure;

PID ActuatorPID(&chamberPressure, &cmdVel, &atmosPressure, Kp, Ki, Kd, DIRECT);

JrkG2I2C jrk;          // Motor Controller
Adafruit_BMP280 Atmos; // First BMP280 sensor at address 0x76
Adafruit_BMP280 Chamb; // Second BMP280 sensor at address 0x77
Adafruit_MPU6050 MPU;  // IMU sensor at address 0x68

bool sdAv = true;            // Is there an SD card
File dataFile;               // Sets a data file
String currentDataFileName;  // File for Data
String currentEventFileName; // For for Event Logs

uint8_t mode = INIT;

bool sitl = false;
char cbuff[100];

struct PIDVals
{
  double kp;
  double ki;
  double kd;
  uint8_t checksum;
};

String getNextFileName(String name, String type);
void DataSave();
void EventLog(String event);
void clearArray(char *Array, uint8_t len);
void SerialCMDHandle();
bool recieveSITL();
void readData();
bool setSpeed(int speed);
void Autotune();
uint8_t compute_checksum(double f1, double f2, double f3);

int main()
{
  while (1)
  {
    SerialCMDHandle();
    currentTime = millis();
    if (currentTime - previousTime >= timeInterval)
    {
      if (sitl)
      {
        recieveSITL();
      } // If doing SITL testing recieve SITL data
      else
      {
        readData();
      } // Otherwise poll sensors

      actualVelocity = (altitude - previousAltitude) / (currentTime - previousTime);

      switch (mode)
      {
      case INIT: // Initialising
      {
        Serial.begin(115200);
        Wire.begin();  // BMP + IMU sensors
        Wire2.begin(); // Motor controller

        // Sets test LED pins as outputs
        pinMode(LED1, OUTPUT);
        pinMode(LED2, OUTPUT);
        pinMode(LED3, OUTPUT);
        pinMode(LED4, OUTPUT);

        jrk.setTarget(startHeight);
        delay(1000);

        if (!SD.begin(BUILTIN_SDCARD))
        {                                  // Initial SD card check
          digitalWrite(BUILTIN_LED, HIGH); // If the SD isn't detected the built in LED will light up
          digitalWrite(LED4, HIGH);
          sdAv = false;
        }

        currentDataFileName = getNextFileName("Sensor Data", ".csv");
        if (sdAv)
        {
          dataFile = SD.open(currentDataFileName.c_str(), FILE_WRITE); // Initialize the dataFile object
          if (dataFile)
          {
            dataFile.println("Timestamp (ms),Altitude (m),Measured Velocity (m/s),Atmos_P (hPa),Chamb_P (hPa),Atmos_T (C),Chamb_T (C), AccelZ (g), Actuator Height (mm),Battery V (mV), Actuator Current (mA)"); // Header Row
            dataFile.close();
          }
          else
          {
            digitalWrite(BUILTIN_LED, HIGH);
            digitalWrite(LED4, HIGH);
          }
        }

        currentEventFileName = getNextFileName("Events", ".txt");
        if (sdAv)
        {
          dataFile = SD.open(currentEventFileName.c_str(), FILE_WRITE); // Initialize the dataFile object
          if (dataFile)
          {
            EventLog("Powered On");
          }
          else
          {
            digitalWrite(BUILTIN_LED, HIGH);
            digitalWrite(LED4, HIGH);
          }
        }

        delay(2000);
        int status;
        // Check and initialize the first Chamb80 sensor at address 0x76
        status = Atmos.begin(0x77);
        if (status != 1)
        {
          digitalWrite(LED1, HIGH); // If the Atmos isn't detected LED 1 on
          digitalWrite(LED4, HIGH);
          char buff[20];
          sprintf(buff, "Atmos BP Error: %i", status);
          Serial.println(buff);
        }
        Atmos.setSampling(Atmos.MODE_NORMAL, Atmos.SAMPLING_X16, Atmos.SAMPLING_X16, Atmos.FILTER_OFF, Atmos.STANDBY_MS_1);

        // Check and initialize the second Chamb80 sensor at address 0x77
        status = Chamb.begin(0x76);
        if (status != 1)
        {
          digitalWrite(LED2, HIGH); // If the Chamb isn't detected LED 2 on
          digitalWrite(LED4, HIGH);
          char buff[20];
          sprintf(buff, "Chamber BP Error: %i", status);
          Serial.println(buff);
        }
        Chamb.setSampling(Chamb.MODE_NORMAL, Chamb.SAMPLING_X16, Chamb.SAMPLING_X16, Chamb.FILTER_OFF, Chamb.STANDBY_MS_1);

        status = MPU.begin(0x68);
        if (status != 1)
        {
          digitalWrite(LED3, HIGH); // If the MPU isn't detected LED 3 on
          digitalWrite(LED4, HIGH);
          char buff[20];
          sprintf(buff, "MPU Error: %i", status);
          Serial.println(buff);
        }
        MPU.setAccelerometerRange(MPU6050_RANGE_16_G);
        MPU.setGyroRange(MPU6050_RANGE_500_DEG);
        MPU.setFilterBandwidth(MPU6050_BAND_21_HZ);

        previousTime = millis();
        altitude = Atmos.readAltitude();
        previousAltitude = altitude;

        PIDVals eepromVals;
        EEPROM.get(0, eepromVals);
        uint8_t checksum = compute_checksum(eepromVals.kp, eepromVals.ki, eepromVals.kd);
        if ((eepromVals.kp != -1) && (checksum = eepromVals.checksum))
        {
          ActuatorPID.SetTunings(eepromVals.kp, eepromVals.ki, eepromVals.kd);
        }

        mode = READY;
        EventLog("Switching to ready mode");
      }
      break;
      case READY:
      {
        if (accelZ >= launchAccel && actualVelocity >= minVel)
        { // Detect launch based on both acceleration and velocity
          mode = ASCENT;
          timeInterval = expRunTime;
          EventLog("Liftoff detected");
        }
      }
      break;
      case ASCENT:
      {
        if (altitude > armAlt && actualVelocity <= minVel)
        {
          mode = RUNNING;
          EventLog("Apogee");
        } // Detect apogee with velocity and safe this with a minimum altitude to start at
      }
      break;
      case RUNNING:
      {
        ActuatorPID.Compute();
        setSpeed(cmdVel);
        if (altitude < groundAltitude && actualVelocity <= minVel)
        {
          mode = DONE;
          EventLog("Landing");
        } // Detect landing with alt and velocity
      }
      break;
      case DONE:
      {
        delay(1);
      }
      break;
      }
      previousAltitude = altitude;
      previousTime = currentTime;
    }
  }
  return (1);
}

bool setSpeed(int speed)
{
  if(speed > maxSpeed){
    speed = maxSpeed;
  }
  if(speed < -maxSpeed){
    speed = -maxSpeed;
  }
  if (actuatorHeight >= maxHeight && speed > 0)
  {
    EventLog("Max height hit");
    jrk.forceDutyCycle(0);
    return 0;
  }
  if (actuatorHeight <= minHeight && speed < 0)
  {
    jrk.forceDutyCycle(0);
    EventLog("Min height hit");
    return 0;
  }
  jrk.forceDutyCycle(speed);
  return (1);
}

String getNextFileName(String name, String type)
{
  int fileNumber = 0;
  while (true)
  {
    String fileName = name + String(fileNumber) + type;
    if (!SD.exists(fileName.c_str()))
    {
      return fileName;
    }
    fileNumber++;
  }
}

void DataSave()
{
  dataFile = SD.open(currentDataFileName.c_str(), FILE_WRITE);
  if (dataFile)
  {
    char buff[255];
    sprintf(buff, "%lu, %.2f, %.2f, %f, %f, %.2f, %.2f, %.2f, %f, %f, %f\n",
            currentTime, altitude, actualVelocity, pressureAtmos, pressureChamber, temperatureAtmos, temperatureChamber, accelZ, actuatorHeight, actuatorVoltage, actuatorCurrent);
    dataFile.print(buff); // Pressure in chamber (hPa)
    dataFile.close();     // Close the file
  }
  return;
}

void EventLog(String event)
{
  if (sdAv)
  {
    dataFile = SD.open(currentEventFileName.c_str(), FILE_WRITE);
    if (dataFile)
    {
      dataFile.print(millis());
      dataFile.print(",");
      dataFile.println(event); // Header Row
      dataFile.close();        // Close the file
    }
  }
  return;
}

void readData()
{
  altitude = Atmos.readAltitude();
  pressureAtmos = Atmos.readPressure();
  pressureChamber = Chamb.readPressure();
  pressureDifference = pressureAtmos - pressureChamber;
  temperatureAtmos = Atmos.readTemperature();
  temperatureChamber = Chamb.readTemperature();
  actuatorHeight = jrk.getScaledFeedback();
  actuatorVoltage = jrk.getVinVoltage();
  actuatorCurrent = jrk.getCurrent();
  MPU.getEvent(&a, &g, &temp);
  accelZ = a.acceleration.z;
}

bool recieveSITL()
{
  pressureChamber = Chamb.readPressure();
  temperatureChamber = Chamb.readTemperature();
  sprintf(cbuff, "RD, %f, %f", pressureChamber, temperatureChamber);
  Serial.print(cbuff); // Send RD to python script to get it to send data
  char incomming[SITLLength];
  if (Serial.readBytes(incomming, SITLLength) == SITLLength)
  {
    altitude = (incomming[0] << 3 * 8) | (incomming[1] << 2 * 8) | (incomming[2] << 1 * 8) | incomming[3];
    pressureAtmos = (incomming[4] << 3 * 8) | (incomming[5] << 2 * 8) | (incomming[6] << 1 * 8) | incomming[7];
    pressureChamber = (incomming[8] << 3 * 8) | (incomming[9] << 2 * 8) | (incomming[10] << 1 * 8) | incomming[11];
    temperatureAtmos = (incomming[12] << 3 * 8) | (incomming[13] << 2 * 8) | (incomming[14] << 1 * 8) | incomming[15];
    temperatureChamber = (incomming[16] << 3 * 8) | (incomming[17] << 2 * 8) | (incomming[18] << 1 * 8) | incomming[19];
    accelZ = (incomming[20] << 3 * 8) | (incomming[21] << 2 * 8) | (incomming[22] << 1 * 8) | incomming[23];

    pressureDifference = pressureAtmos - pressureChamber;
    actuatorHeight = jrk.getScaledFeedback();
    actuatorVoltage = jrk.getVinVoltage();
    actuatorCurrent = jrk.getCurrent();
    return (1);
  }
  else
  {
    Serial.println("Error: No data recieved");
    return (0);
  }
}

void clearArray(char *Array, uint8_t len)
{
  while (len != 0)
  {
    Array[len - 1] = 0;
    len--;
  }
  return;
}

void SerialCMDHandle()
{
  static uint8_t index;
  static char buffer[100];
  bool command = false;

  if (Serial.available())
  {
    buffer[index] = Serial.read();
    Serial.print(buffer[index]);
    index++;
    if (index == 100)
    {
      index = 0;
      Serial.println("Buffer Overflow");
      clearArray(buffer, 100);
    }
  }

  uint8_t i = 0;
  while (i < 100)
  {
    if (buffer[i] == '\r' || buffer[i] == '\n')
    {
      command = true;
    }
    i++;
  }

  if (command)
  {
    switch (buffer[0])
    {
    case 'S': // Sensors
      switch (buffer[1])
      {
      case 'A':
      { // Atmospheric
        char buff[100];
        sprintf(buff, "Alt: %f m| Press: %f Pa| Temp %f C \n", Atmos.readAltitude(), Atmos.readPressure(), Atmos.readTemperature());
        Serial.print(buff);
      }
      break;
      case 'C':
      { // Chamber
        char buff[100];
        sprintf(buff, "Alt: %f m| Press: %f Pa| Temp %f C \n", Chamb.readAltitude(), Chamb.readPressure(), Chamb.readTemperature());
        Serial.print(buff);
      }
      break;
      case 'M':
      { // Motor Controller
        char buff[100];
        sprintf(buff, "Pos: %f mm| Vin: %u mV| Current: %u mA \n", jrk.getScaledFeedback(), jrk.getVinVoltage(), jrk.getRawCurrent());
        Serial.print(buff);
      }
      break;
      case 'I':
      { // MPU6050
        MPU.getEvent(&a, &g, &temp);
        char buff[100];
        sprintf(buff, "Ax: %f | Ay: %f | Az %f \n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
        Serial.print(buff);

        sprintf(buff, "Gx: %f | Gy: %f | Gz %f \n", g.gyro.x, g.gyro.y, g.gyro.z);
        Serial.print(buff);
      }
      break;
      case 'R':
      { // Reset
        Atmos.reset();
        Chamb.reset();
        MPU.reset();
        Serial.print("Sensors Reset");
      }
      break;
      case 'O':
      { // All Data
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
      break;
      case 'V':
      { // Velocity
        char cbuff[100];
        sprintf(cbuff, "Altitude: %.2f m| Velocity: %.2f m/s \n", altitude, actualVelocity);
        Serial.print(cbuff);
      }
      break;
      case 'E':
      {
        char cbuff[100];
        sprintf(cbuff, "State machine: %i | Pressure Difference %.2f Pa \n", mode, pressureDifference);
        Serial.print(cbuff);
      }
      }
    case 'A': // Actuator
      switch (buffer[1])
      {
      case 'S':
      { // Set input of mm*10
        char numBuff[3] = {buffer[3], buffer[4], buffer[5]};
        float set = strtol(numBuff, NULL, 10) / 10;
        char buff[50];
        sprintf(buff, "Going to %.1f mm", set);
        Serial.println(buff);
        jrk.setTarget(set);
      }
      break;
      case 'T':
      {
        Autotune();
      }
      break;
      }
    }
    clearArray(buffer, 100);
    index = 0;
  }
  return;
}

void Autotune()
{
  PIDAutotuner tuner = PIDAutotuner();
  tuner.setTargetInputValue(tunePress);
  tuner.setLoopInterval(expRunTime);
  tuner.setOutputRange(-maxSpeed, maxSpeed);
  tuner.setZNMode(PIDAutotuner::znModeBasicPID);
  tuner.startTuningLoop();

  long microseconds;
  while (!tuner.isFinished())
  {
    microseconds = micros();
    double input = Chamb.readPressure();
    double output = tuner.tunePID(input);
    setSpeed(output);
    while (micros() - microseconds < expRunTime)
      delayMicroseconds(1);
  }

  uint8_t checksum = compute_checksum(tuner.getKp(), tuner.getKi(), tuner.getKd());
  PIDVals Tuned = {tuner.getKp(), tuner.getKi(), tuner.getKd(), checksum};
  EEPROM.put(0, Tuned);
  char buff[50];
  sprintf(buff, "Kp %f, Ki %f, Kd %f", tuner.getKp(), tuner.getKi(), tuner.getKd());
  Serial.println(buff);
  EventLog("PID Tuned");
  return;
}

uint8_t compute_checksum(double f1, double f2, double f3)
{
  // Array to hold the four float values
  double doubles[4] = {f1, f2, f3};

  // Pointer to access the bytes of the float array
  uint8_t *bytes = (uint8_t *)doubles;

  // Compute the checksum by summing all bytes
  uint32_t checksum = 0;
  for (uint8_t i = 0; i < 3 * sizeof(double); i++)
  {
    checksum += bytes[i];
  }

  // Reduce the checksum to fit into a uint8_t
  return (uint8_t)(checksum & 0xFF);
}