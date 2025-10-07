void setMotorAngles() {
    unsigned long current_time = millis();
    for (int i = 0; i < 3; i++) {
        motors[i].controlMotor(current_time);
    }
}

void setWristAngles() {
    for (int i = 0; i < 3; i++) {
        servos[i].setAngle();
    }
}