#ifndef defines_h
#define defines_h

//************** Library includes *************//
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <cmath>
#include <util/delay.h>
#include <pidautotuner.h>   //https://github.com/jackw01/arduino-pid-autotuner

//************** Alias defines *************//
#define AccWire Wire2                       // Changes Wire for JRK to Wire2

//************** Pin defines *************//
#define LED1 0                              // Status LED1
#define LED2 1                              // Status LED2
#define motorGate 2                         // Motor gate pin
#define valveGate 3                         // Valve gate pin
#define BUILTIN_LED 13                      // Built in LED
#define testPin 23                          // Trigger on pin 2

//************** Sensor defines *************//
#define AtmosTemp 0
#define AtmosPress 1
#define AtmosAlt 2
#define ChambTemp 3
#define ChambPress 4
#define ChambAlt 5

#define maxTempDifference 4

//************** Actuator defines *************//
#define startHeight 14                       // Start height of the actuator (mm)
#define maxHeight 70                        // Max height of the actuator before it reaches BMP (mm)
#define minHeight 3                         // Min height before actuator leaves the chamber (mm)
#define absMax 90                           // Chamber max from actuator (mm)
#define actuatorScale 40.95
#define maxTarget maxHeight * actuatorScale
#define minTarget minHeight * actuatorScale

//************** Experiment defines *************//
#define activeTime 10
#define standbyTime 1000
#define targetAltitude 2500.0               // Extra condition to run main script. Underestimate of expected apogee (m)
#define r 0.02                              // Pressure chamber radius (m)
#define A M_PI * pow(r, 2)                  // Area (m^2)

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

//************** PID defines *************//
#define SITLLength 20
#endif