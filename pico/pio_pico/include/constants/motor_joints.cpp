struct MotorJoint {
    int dir_pin;
    int pwm_pin;
    uint enc_A;
    uint enc_B;
    int encoder_val;
    float target_angle;
    float current_angle;
    PID pid;

    MotorJoint(int dirPin, int pwmPin, uint enc_A, uint enc_B, float kp, float ki, float kd, float imax)
        : dir_pin(dirPin),
          pwm_pin(pwmPin),
          encoder_val(0),
          target_angle(0),
          current_angle(0),
          pid(kp, ki, kd, imax) {}

    void controlMotor(unsigned long current_time) {
        float error = target_angle - current_angle;
        float control = pid.get_pid(error, 0.0f, current_time);

        int dir = (control < 0) ? LOW : HIGH;
        analogWrite(pwm_pin, abs(control));
        digitalWrite(dir_pin, dir);
    }
};

MotorJoint motors[3] = {
    MotorJoint(right_hip_dir, right_hip_pwm, right_hip_encA, right_hip_encB, 6, 0.01, 0.05, 100), // dir pin, pwm pin, kp, ki, kd, imax
    MotorJoint(right_shoulder_dir, right_shoulder_pwm, right_shoulder_encA, right_shoulder_encB, 6, 0.01, 0.05, 100), // dir pin, pwm pin, kp, ki, kd, imax
    MotorJoint(right_elbow_dir, right_elbow_pwm, right_elbow_encA, right_elbow_encB, 6, 0.01, 0.05, 100), // dir pin, pwm pin, kp, ki, kd, imax
};

