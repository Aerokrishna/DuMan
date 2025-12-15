// //libraries
#include <Arduino.h>
#include <vector>

#include "Servo.h"
#include "serial_parser.hpp"
#include "blitz_timer.cpp"
#include "Ultrasonic.h"

#include <PID.h>
// pinmap
#include "pinmap/PinMap_base.h"
// // constants
#include "constants/constants.cpp"
#include "constants/motor_joints.cpp"
#include "constants/servo_joints.cpp"

#include "functions/store_data.cpp"
#include "functions/joint_angles.cpp"
#include "functions/timer_cb.cpp"


#include "read_write/encoders.cpp"
