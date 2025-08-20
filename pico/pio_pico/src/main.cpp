#include <iostream>
#include <vector>
#include <cstring>  // for std::memcpy
#include <stdexcept>
#include "include_all.cpp"
#include "serial_parser.hpp"

void get_servo_angles(int ik_angles[NUM_JOINTS]){
    // right arm
    targetAngles[5] = ik_angles[5] + 45;
    targetAngles[6] = 135 - ik_angles[6];
    targetAngles[7] = ik_angles[7] + 135;
    targetAngles[8] = 90 - ik_angles[8];    // wrist
    targetAngles[9] = 90 - ik_angles[9]; // wrist yaw

    targetAngles[0] = ik_angles[0] + 135;
    targetAngles[1] = ik_angles[1] + 45;
    targetAngles[2] = 45 - ik_angles[2];
    targetAngles[3] = 90 + ik_angles[3];
    targetAngles[4] = -ik_angles[4];
}

void get_data(){
    std::vector<uint8_t> payload = receive_data();
    uint8_t id = payload[0];

    if (!payload.empty()){
        Serial.println(id);

        if (id == ANGLES){
            Angles angles = parse_struct<Angles>(payload);
            
            Serial.print(" hip ");
            Serial.println(angles.hip);

            int ik_angles[NUM_JOINTS];

            ik_angles[0] = angles.hip_l;
            ik_angles[1] = angles.shoulder_l;
            ik_angles[2] = angles.elbow_l;
            ik_angles[3] = angles.wrist_l;
            ik_angles[4] = angles.wrist_yaw_l;
            ik_angles[5] = angles.hip;
            ik_angles[6] = angles.shoulder;
            ik_angles[7] = angles.elbow;
            ik_angles[8] = angles.wrist;
            ik_angles[9] = angles.wrist_yaw;
            get_servo_angles(ik_angles);
            for (int i = 0; i<NUM_JOINTS; i++){
                Serial.println(targetAngles[i]);
            }
        }
    }
}

void setup(){
    Serial.begin(115200);
    // while (!Serial);

    for (int i = 0; i<NUM_JOINTS; i++){
        servos[i].attach(servoPins[i], 500, 2500);
        // servos[i].write(current_angle[i])
    }
    // myservo.attach(4, 500, 2500);
}

void loop() {
    // for (int i = 0; i<NUM_JOINTS; i++){
    //     servos[i].write(currentAngles[i]);
    // }
    
    get_data();
    setServoAngles(targetAngles);
    
}   

