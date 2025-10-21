// takes raw byte array, matches the interface and parses the data
void store_data(std::vector<uint8_t> payload) {

    if (!payload.empty()) {

        // find id
        uint8_t id = payload[0];
    
        // parse the struct based on the id
        if (id == JOINT_ANGLES) {
            
            // function to parse the struct
            joint_angles = parse_struct<JointAngles>(payload); // count.a, count.b, count.c, count.d, based on your interface
            motors[0].target_angle = joint_angles.hip;
            motors[1].target_angle = joint_angles.shoulder;
            motors[2].target_angle = joint_angles.elbow;
            
            servos[0].target_angle = joint_angles.wrist1;
            servos[1].target_angle = joint_angles.wrist2;
            servos[2].target_angle = joint_angles.wrist3;

            // joint_angles.id = 2;
    
        }

        if (id == JOINT_VEL) {
            
            // function to parse the struct
            joint_angles = parse_struct<JointAngles>(payload); // count.a, count.b, count.c, count.d, based on your interface
            motors[0].control = joint_angles.hip;
            motors[1].control = joint_angles.shoulder;
            motors[2].control = joint_angles.elbow;
            
            // servos[0].target_angle = joint_angles.wrist1;
            // servos[1].target_angle = joint_angles.wrist2;
            // servos[2].target_angle = joint_angles.wrist3;

            new_vel_data = millis(); // starts when new data received
            vel_cmd = true;

            // joint_angles.id = 2;
            // send_data(pack_data<JointAngles>(joint_angles));

        }
    }
}
