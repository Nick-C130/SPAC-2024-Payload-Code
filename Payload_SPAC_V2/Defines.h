#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <cmath>
#include <util/delay.h>

#define AccWire Wire2                       // Changes Wire for JRK to Wire2

#define LED1 0                              // Status LED1
#define LED2 1                              // Status LED2
#define motorGate 2                         // Motor gate pin
#define valveGate 3                         // Valve gate pin
#define BUILTIN_LED 13                      // Built in LED
#define testPin 23                          // Trigger on pin 2

#define startHeight 9   // Start height of the actuator (mm)
#define maxHeight 70    // Max height of the actuator before it reaches BMP (mm)
#define minHeight 10    // Min height before actuator leaves the chamber (mm)
#define absMax 90       // Chamber max from actuator (mm)
#define grad 40.95
#define maxTarget maxHeight * grad
#define minTarget minHeight * grad