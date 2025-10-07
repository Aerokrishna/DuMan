struct ServoJoint {
    int servo_pin;
    int target_angle;
    Servo servo;

    ServoJoint(int pin) : servo_pin(pin), target_angle(0) {}

    void attachServo() {
        servo.attach(servo_pin);
    }

    void setAngle() {
        servo.write(target_angle);
    }
};

ServoJoint servos[3] = {
    ServoJoint(8),
    ServoJoint(9),
    ServoJoint(10)
};

