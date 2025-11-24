#include "include_all.cpp"

void setup(){
    Serial.begin(115200);

    // for (int i = 0; i < 3; i++) {
    //     pinMode(motors[i].dir_pin, OUTPUT);
    //     pinMode(motors[i].pwm_pin, OUTPUT);

    //     pinMode(motors[i].enc_A, INPUT_PULLUP);
    //     pinMode(motors[i].enc_B, INPUT_PULLUP);

    //     servos[i].attachServo();

    // }

    pinMode(motors[0].dir_pin, OUTPUT);
    pinMode(motors[0].pwm_pin, OUTPUT);

    pinMode(motors[0].enc_A, INPUT_PULLUP);
    pinMode(motors[0].enc_B, INPUT_PULLUP);

    pinMode(motors[1].dir_pin, OUTPUT);
    pinMode(motors[1].pwm_pin, OUTPUT);

    pinMode(motors[1].enc_A, INPUT_PULLUP);
    pinMode(motors[1].enc_B, INPUT_PULLUP);

    pinMode(motors[2].dir_pin, OUTPUT);
    pinMode(motors[2].pwm_pin, OUTPUT);

    pinMode(motors[2].enc_A, INPUT_PULLUP);
    pinMode(motors[2].enc_B, INPUT_PULLUP);

    gpio_set_irq_enabled_with_callback(motors[0].enc_A, GPIO_IRQ_EDGE_RISE, true, updateEncoder);
    gpio_set_irq_enabled_with_callback(motors[1].enc_A, GPIO_IRQ_EDGE_RISE, true, updateEncoder);
    gpio_set_irq_enabled_with_callback(motors[2].enc_A, GPIO_IRQ_EDGE_RISE, true, updateEncoder);

    servos[0].attachServo();
    servos[1].attachServo();
    servos[2].attachServo();

    gripper_servo.attach(gripper_pin);
    // grip_state.grip_state = false;

}

void loop() {

    // recieves data in every loop 
    std::vector<uint8_t> payload = receive_data();

    // stores data, called every loop
    store_data(payload);
    
    // setWristAngles();
    // setMotorAngles();

    unsigned long current_time = millis();

    motors[0].controlMotor(current_time);
    motors[1].controlMotor(current_time);
    motors[2].controlMotor(current_time);

    // servos[0].setAngle();
    // servos[1].setAngle();
    // servos[2].setAngle();

//    set_gripper(grip_state.grip_state);

    if (vel_cmd == true && float(current_time - new_vel_data) > 1000.0f){
        vel_cmd = false;
        
        // // stay wherever you are
        // motors[0].target_angle = motors[0].current_angle;
        // motors[1].target_angle = motors[1].current_angle;
        // motors[2].target_angle = motors[2].current_angle;

    }
    
    t1.spin();

}      




