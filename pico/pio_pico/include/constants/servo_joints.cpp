struct ServoJoint {
    int servo_pin;
    int target_angle;
    Servo servo;

    ServoJoint(int pin) : servo_pin(pin), target_angle(0) {}

    void attachServo() {
        servo.attach(servo_pin, 500, 2500);
    }

    void setAngle() {
        servo.write(target_angle);
    }
};

// ServoJoint servos[3] = {
//     ServoJoint(right_wrist1_pin),
//     ServoJoint(right_wrist2_pin),
//     ServoJoint(right_wrist3_pin),

// };

