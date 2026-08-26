void timer_cb(){

    joint_angles_feedback.hip = motors[0].current_angle;
    joint_angles_feedback.shoulder = motors[1].current_angle;
    joint_angles_feedback.elbow = motors[2].current_angle;

    joint_angles_feedback.wrist1 = motors[0].control;
    joint_angles_feedback.wrist2 = motors[1].control;
    joint_angles_feedback.wrist3 = motors[2].control;

    joint_angles_feedback.id = JOINT_ANGLES_FEEDBACK;
    
    send_data(pack_data<JointAngles>(joint_angles_feedback));

    if (ungrip==true){
        if (ungrip_cnt < 50){
            ungrip=false;
            ungrip_cnt++;

            gripper_servo.write(open_angle);
        }
        else{
            gripper_servo.write(90);
        }
    }
}

// void us_timer_cb(){

//     gripper_object.distance = ultrasonic.MeasureInCentimeters(30000); ;
//     gripper_object.id = USSENSOR;
//     send_data(pack_data<USSensor>(gripper_object));

// }

BlitzTimer t1(timer_cb, 10);
// BlitzTimer t2(us_timer_cb, 50);

