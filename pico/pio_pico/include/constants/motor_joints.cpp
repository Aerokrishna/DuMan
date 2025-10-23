struct MotorJoint {
    uint dir_pin;
    uint pwm_pin;
    uint enc_A;
    uint enc_B;
    int encoder_val;
    float target_angle;
    float current_angle;
    float control;
    float error;
    PID pid;

    MotorJoint(uint dirPin, uint pwmPin, uint encA, uint encB, float kp, float ki, float kd, float imax)
        : dir_pin(dirPin),
          pwm_pin(pwmPin),
          enc_A(encA),
          enc_B(encB),
          encoder_val(0),
          target_angle(0),
          current_angle(0),
          control(0),
          error(0),
          pid(kp, ki, kd, imax) {}

    void controlMotor(unsigned long current_time) {

        if (vel_cmd == false){
            error = target_angle - current_angle;
            control = pid.get_pid(error, 0.0f, current_time);

            // debug_state("Joint Angle");
        }   
        else {

        }
        int pwm = constrain(control, -255, 255);
        // if (abs(pwm) < 10){pwm = 0;}
        
        if (control <= 0.0f) {
            if (abs(control) > 30.0f){
                control = -30.0;
            }
            digitalWrite(dir_pin, 0);
            analogWrite(pwm_pin, abs(pwm));
        }

        else {
            if (abs(control) > 30.0f){
                control = 30.0;
            }
            digitalWrite(dir_pin, 1);
            analogWrite(pwm_pin, abs(pwm));
        }
    }
};

MotorJoint motors[3] = {
    MotorJoint(hip_dir, hip_pwm, hip_encA, hip_encB, 8, 0.0, 0.0, 100), // dir pin, pwm pin, kp, ki, kd, imax
    MotorJoint(shoulder_dir, shoulder_pwm, shoulder_encA, shoulder_encB, 8, 0.0, 0.0, 100), // dir pin, pwm pin, kp, ki, kd, imax
    MotorJoint(elbow_dir, elbow_pwm, elbow_encA, elbow_encB, 3, 0.0, 0.0, 100) // dir pin, pwm pin, kp, ki, kd, imax
};

// MotorJoint motor(right_shoulder_dir, right_shoulder_pwm, right_shoulder_encA, right_shoulder_encB, 6.5, 0.0, 0.0, 100);