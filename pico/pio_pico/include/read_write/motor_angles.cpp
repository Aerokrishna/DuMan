void setMotorAngles(){

    for (int i = 0; i <= NUM_JOINTS/2; i++) {

    unsigned long current_time = millis();

    // compute error
    float error = motor_joints[i].target_angle - motor_joints[i].current_angle;

    // get pid based on the gains
    float control = motor_joints[i].motor_pid.get_pid(error, 0.0, current_time);

    // write it to the motor
    float dir = 1;
    if (control < 0){
        dir = 0;

    }
    
    digitalWrite(motor_joints[i].dir_pin, dir);
    analogWrite(motor_joints[i].pwm_pin, abs(control));

    }
    
}