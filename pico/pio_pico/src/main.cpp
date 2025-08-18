#include <iostream>
#include <vector>
#include <cstring>  // for std::memcpy
#include <stdexcept>
#include "include_all.cpp"
#include "serial_parser.hpp"


void setup(){
    Serial.begin(115200);
    while (!Serial); // wait for serial monitor to open
}
void loop() {
    // static InterfaceID interface = -1;
    // Example: serialize struct
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
    // delay(1);
}
