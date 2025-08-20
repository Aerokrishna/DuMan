// //libraries
#include <Arduino.h>
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <sys/time.h>
#include "Servo.h"

// pinmap
#include "pinmap/PinMap_base.h"
// // constants
#include "constants/home_angles.h"
#include "constants/constants.cpp"


// // read write
// #include "read_write/encoders.cpp"
#include "read_write/servo.cpp"
