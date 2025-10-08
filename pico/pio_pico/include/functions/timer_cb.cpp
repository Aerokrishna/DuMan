void timer_cb(){

    joint_angles_feedback.right_shoulder = motors[0].current_angle;
    joint_angles_feedback.right_elbow = motors[0].control;
    joint_angles_feedback.right_wrist1 = motors[0].error;

    joint_angles_feedback.id = 2;
    
    send_data(pack_data<JointAngles>(joint_angles_feedback));

}

BlitzTimer t1(timer_cb, 10);
