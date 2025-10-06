// Function to move servos to target angles
void setWristAngles() {
    for (int i = 0; i <= NUM_JOINTS/2; i++) {

        servo_joints[i].servo.write(servo_joints[i].target_angle);

    }
}
