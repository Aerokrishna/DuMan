void timer_cb(){

    joint_angles_feedback.hip = motors[0].control;
    joint_angles_feedback.shoulder = motors[1].control;
    joint_angles_feedback.elbow = motors[2].control;

    joint_angles_feedback.wrist1 = motors[0].current_angle;
    joint_angles_feedback.wrist2 = motors[1].current_angle;
    joint_angles_feedback.wrist3 = motors[2].current_angle;

    joint_angles_feedback.id = 2;
    
    send_data(pack_data<JointAngles>(joint_angles_feedback));

}

BlitzTimer t1(timer_cb, 10);
