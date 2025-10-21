struct ServoJoint {
    int servo_pin;
    int target_angle;
    Servo servo;

    ServoJoint(int pin, int initial_angle) : servo_pin(pin), target_angle(initial_angle) {}

    void attachServo() {
        servo.attach(servo_pin, 500, 2500);
    }

    void setAngle() {
        servo.write(target_angle);
    }
};

ServoJoint servos[3] = {
    ServoJoint(wrist1_pin, 90),
    ServoJoint(wrist2_pin, 90),
    ServoJoint(wrist3_pin, 90)

};

