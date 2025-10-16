void timer_cb(){

    joint_angles_feedback.right_hip = motors[0].control;
    joint_angles_feedback.right_shoulder = motors[1].control;
    joint_angles_feedback.right_elbow = motors[2].control;

    joint_angles_feedback.id = 2;
    
    send_data(pack_data<JointAngles>(joint_angles_feedback));

}

BlitzTimer t1(timer_cb, 10);
