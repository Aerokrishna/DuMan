void timer_cb(){

    if (new_data == true){

        profile.reset_profile(current_angle, cmd_pidvals.target_angle, cmd_pidvals.time_);
        motor_pid.update_gains(cmd_pidvals.Kp, cmd_pidvals.Ki, cmd_pidvals.Kd);
        new_data = false;

    }

    // send the message 
    // send_data(pack_data<Counter>(count_response));

    unsigned long current_time = millis();

    // gets the setpoint
    float setpoint = profile.get_setpoint(current_time);

    // compute error
    float error = setpoint - current_angle;

    // get pid based on the gains
    float control = motor_pid.get_pid(error, 0.0, current_time);

    // write it to the motor
    float dir = 1;
    if (control < 0){
        dir = 0;

    }
    
    digitalWrite(motor_dir, dir);
    analogWrite(motor_pwm_, abs(control));

    elapsed_time = elapsed_time + 0.01; // dt

    // send feedback
    // setpoint, current, elapsed time

    pid_feedback.id = PID_FEEDBACK;
    pid_feedback.setpoint = setpoint;
    pid_feedback.current = current_angle;
    pid_feedback.motor_pwm = control;
    pid_feedback.elapsed_time = elapsed_time;

    send_data(pack_data<PIDFeedback>(pid_feedback));

}

BlitzTimer t1(timer_cb, 10);
