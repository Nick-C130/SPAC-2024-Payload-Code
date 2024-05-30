#include "Defines.h"
#include "JrkG2.h"

bool experimentPrimed = false;  // Actuator off until true
bool dataSavePrimed = false;    // Data starts saving on launch

unsigned long currentTime;          // Variable for time (ms)
unsigned long previousTime = 0;     // Initial time variable (ms)
unsigned long timeInterval = 1000;  // Initial interval between sensor readings (ms)
unsigned long deltaTime;            // Calculates time difference (s)

float actualVelocity;  // Velocity as calculated by barometer (m/s)

float altitude = 0;        // Reads altitude (check sensor) (m)
float previousAltitude;    // Sets variable for previous altitude (m)
float deltaAltitude;       // Change in height for velocity calculation (m)
float pressureAtmos;       // Reads chamber's pressure (Pa)
float pressureChamber;     // Reads chamber's pressure (Pa)
float pressureDifference;  // Difference between Atmos and Chamber (Pa)
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

PID ActuatorPID(&pressureInput, &setHeight, &pressureAtmosInput, EXPP, EXPI, EXPD, DIRECT);

JrkG2I2C jrk;           // Motor Controller
Adafruit_BMP280 Atmos;  // First BMP280 sensor at address 0x76
Adafruit_BMP280 Chamb;  // Second BMP280 sensor at address 0x77
Adafruit_MPU6050 MPU;   // IMU sensor at address 0x68

bool sdAv = true;                       // Is there an SD card
File dataFile;                          // Sets a data file
String currentDataFileName;             // File for Data
String currentEventFileName;            // For for Event Logs

bool sitl = false;
char cbuff[100];

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

void setup() {
  Serial.begin(115200);
  Wire.begin();   // BMP + IMU sensors
  Wire2.begin();  // Motor controller

  // Sets test LED pins as outputs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  jrk.setPIDPeriod(PIDControlPeriod);
  jrk.setIntegralLimit(integralLimit);

  jrk.setTarget(startHeight);
  delay(1000);

  if (!SD.begin(BUILTIN_SDCARD)) {        // Initial SD card check
    digitalWrite(BUILTIN_LED, HIGH);  // If the SD isn't detected the built in LED will light up
    digitalWrite(LED4, HIGH);
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
      digitalWrite(LED4, HIGH);
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
      digitalWrite(LED4, HIGH);
    }
  }

  delay(2000);
  int status;
  // Check and initialize the first Chamb80 sensor at address 0x76
  status = Atmos.begin(0x77);
  if (status != 1) {
    digitalWrite(LED1, HIGH);  // If the Atmos isn't detected LED 1 on
    digitalWrite(LED4, HIGH);
    char buff[20];
    sprintf(buff, "Atmos BP Error: %i", status);
    Serial.println(buff);
  }
  Atmos.setSampling(Atmos.MODE_NORMAL, Atmos.SAMPLING_X16, Atmos.SAMPLING_X16, Atmos.FILTER_OFF, Atmos.STANDBY_MS_1);

  // Check and initialize the second Chamb80 sensor at address 0x77
  status = Chamb.begin(0x76);
  if (status != 1) {
    digitalWrite(LED2, HIGH);  // If the Chamb isn't detected LED 2 on
    digitalWrite(LED4, HIGH);
    char buff[20];
    sprintf(buff, "Chamber BP Error: %i", status);
    Serial.println(buff);
  }
  Chamb.setSampling(Chamb.MODE_NORMAL, Chamb.SAMPLING_X16, Chamb.SAMPLING_X16, Chamb.FILTER_OFF, Chamb.STANDBY_MS_1);

  status = MPU.begin(0x68);
  if (status != 1) {
    digitalWrite(LED3, HIGH);  // If the MPU isn't detected LED 3 on
    digitalWrite(LED4, HIGH);
    char buff[20];
    sprintf(buff, "MPU Error: %i", status);
    Serial.println(buff);
  }
  MPU.setAccelerometerRange(MPU6050_RANGE_16_G);
  MPU.setGyroRange(MPU6050_RANGE_500_DEG);
  MPU.setFilterBandwidth(MPU6050_BAND_21_HZ);


  previousTime = millis();
  previousAltitude = Atmos.readAltitude();

  EventLog("Setup Completed");
}

void loop() {

}