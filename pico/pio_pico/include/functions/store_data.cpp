// takes raw byte array, matches the interface and parses the data
void store_data(std::vector<uint8_t> payload) {

    if (!payload.empty()) {

        // find id
        uint8_t id = payload[0];
    
        // parse the struct based on the id
        if (id == JOINT_ANGLES) {
            
            // function to parse the struct
            joint_angles = parse_struct<JointAngles>(payload); // count.a, count.b, count.c, count.d, based on your interface
            motor_joints[0].target_angle = joint_angles.right_hip;
            motor_joints[1].target_angle = joint_angles.right_shoulder;
            motor_joints[2].target_angle = joint_angles.right_elbow;
            motor_joints[3].target_angle = joint_angles.left_hip;
            motor_joints[4].target_angle = joint_angles.left_shoulder;
            motor_joints[5].target_angle = joint_angles.left_elbow;

            servo_joints[0].target_angle = joint_angles.right_wrist1;
            servo_joints[1].target_angle = joint_angles.right_wrist2;
            servo_joints[2].target_angle = joint_angles.right_wrist3;
            servo_joints[3].target_angle = joint_angles.left_wrist1;
            servo_joints[4].target_angle = joint_angles.left_wrist2;
            servo_joints[5].target_angle = joint_angles.left_wrist3;

        }
    }
}
