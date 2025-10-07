// takes raw byte array, matches the interface and parses the data
void store_data(std::vector<uint8_t> payload) {

    if (!payload.empty()) {

        // find id
        uint8_t id = payload[0];
    
        // parse the struct based on the id
        if (id == 1) {
            
            // function to parse the struct
            joint_angles = parse_struct<JointAngles>(payload); // count.a, count.b, count.c, count.d, based on your interface
            motors[0].target_angle = joint_angles.right_hip;
            motors[1].target_angle = joint_angles.right_shoulder;
            motors[2].target_angle = joint_angles.right_elbow;
          
            servos[0].target_angle = joint_angles.right_wrist1;
            servos[1].target_angle = joint_angles.right_wrist2;
            servos[2].target_angle = joint_angles.right_wrist3;
            
            joint_angles_feedback = joint_angles;
            joint_angles_feedback.id = 2;
            
            send_data(pack_data<JointAngles>(joint_angles_feedback));
        }
    }
}

