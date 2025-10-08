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
        error = target_angle - current_angle;
        control = pid.get_pid(error, 0.0f, current_time);

        int pwm = constrain(control, -255, 255);
        if (abs(pwm) < 10){pwm = 0;}
        
        if (control <= 0.0f) {
            digitalWrite(dir_pin, 0);
            analogWrite(pwm_pin, abs(pwm));
        }

        else {
            digitalWrite(dir_pin, 1);
            analogWrite(pwm_pin, abs(pwm));
        }
        // digitalWrite(dir_pin, 1);
        // analogWrite(pwm_pin, 100);
    }
};

MotorJoint motors[2] = {
    MotorJoint(right_hip_dir, right_hip_pwm, right_hip_encA, right_hip_encB, 6, 0.0, 0.0, 100), // dir pin, pwm pin, kp, ki, kd, imax
    MotorJoint(right_shoulder_dir, right_shoulder_pwm, right_shoulder_encA, right_shoulder_encB, 10, 0.0, 0.0, 100), // dir pin, pwm pin, kp, ki, kd, imax
 // MotorJoint(right_elbow_dir, right_elbow_pwm, right_elbow_encA, right_elbow_encB, 6, 0.01, 0.05, 100), // dir pin, pwm pin, kp, ki, kd, imax
};

// MotorJoint motor(right_shoulder_dir, right_shoulder_pwm, right_shoulder_encA, right_shoulder_encB, 6.5, 0.0, 0.0, 100);