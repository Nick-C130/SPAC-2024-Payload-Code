//************** MOSFETs *************//
// MV x- Opens/closes air valve
// MM x- Turns on/off motor controller

//************** Sensors *************//
// SA - Atmospheric pressure sensor data
// SC - Chamber pressure sensor data
// SM - Motor controller data - POS VOLTAGE AND CURRENT
// SI - MPU data
// SR - Sensor reset, will need to power cycle
// SO - Other Data (inlcuding some of the above)
// SP - Gets PID values
// SV - Altitude and velocity
// SE - Experiment data

//************** Actuator *************//
// AS xxx- Sets actuator height to xxx/10mm
// AT - Runs PID autotuner
// AP x -- 0 sets default values, 1 sets tunned values
// AX x - 0 turns experiment off, 1 primes experiment
// AB - Runs test script