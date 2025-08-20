#include <iostream>
#include <vector>
#include <cstring>  // for std::memcpy
#include <stdexcept>
#include "include_all.cpp"
#include "serial_parser.hpp"
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
    for (int i = 0; i<NUM_JOINTS; i++){
        servos[i].write(currentAngles[i]);
    }
    
    // setServoAngles(targetAngles);
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
        }
        
    }
}