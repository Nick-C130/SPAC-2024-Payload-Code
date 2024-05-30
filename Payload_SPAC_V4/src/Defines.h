#ifndef defines_h
#define defines_h

//************** Library includes *************//
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>  //https://adafruit.github.io/Adafruit_MPU6050/html/class_adafruit___m_p_u6050.html
#include <SD.h>
#include <cmath>
#include <util/delay.h>
#include <pidautotuner.h>  //https://github.com/jackw01/arduino-pid-autotuner
#include <EEPROM.h>
#include <PID_v1.h>

//************** Alias defines *************//
#define AccWire Wire2  // Changes Wire for JRK to Wire2

//************** Pin defines *************//
#define LED1 0          // Status LED1
#define LED2 1          // Status LED2
#define LED3 4          // Status LED3
#define LED4 5          // Status LED4
#define motorGate 2     // Motor gate pin
#define valveGate 3     // Valve gate pin
#define BUILTIN_LED 13  // Built in LED
#define testPin 23      // Trigger on pin 23

//************** Pressure Sensor defines *************//
#define AtmosTemp 0
#define AtmosPress 1
#define AtmosAlt 2
#define ChambTemp 3
#define ChambPress 4
#define ChambAlt 5

#define maxTempDifference 4
#define maxPressureDiffernce 100

//************** IMU Sensor defines *************//

// #define AccelX
// #define AccelY
// #define AccelZ

// #define GyroX
// #define GyroY
// #define GyroZ


//************** Actuator defines *************//
#define startHeight 0       // Start height of the actuator (mm)
#define maxHeight 70         // Max height of the actuator before it reaches BMP (mm)
#define minHeight 0          // Min height before actuator leaves the chamber (mm)
#define actuatorScale 40.95  // Scaling factor for feedback calcs

//************** Experiment defines *************//
#define armAlt 2500.0           // Extra condition to run main script. Underestimate of expected apogee (m)
#define groundAltitude 400     // Velocity to be under to end experiment
#define groundVelocity 0.2     // Velocity to end experiment
#define launchAccel 20         // Condition for data saving to begin


//************** PID defines *************//
#define tuneVal 40
#define proportionalCoef 10
#define proportionalExpo 0
#define integralCoef 819
#define integralExpo 13
#define derivativeCoef 5
#define derivativeExpo 0
#define PIDControlPeriod 30
#define integralLimit 6000


//************** STIL defines *************//
#define SITLLength 24

#endif