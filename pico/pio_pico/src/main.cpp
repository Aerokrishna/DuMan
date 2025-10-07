#include "include_all.cpp"

void setup(){

    for (int i = 0; i < 3; i++) {
        pinMode(motors[i].dir_pin, OUTPUT);
        pinMode(motors[i].pwm_pin, OUTPUT);

        servos[i].servo.attach(servos[i].servo_pin);
    }

    // // serial begin
    Serial.begin(115200);

    // pinMode(motor_pwm_, OUTPUT);
    // pinMode(motor_dir, OUTPUT);

    // pinMode(enc_A, INPUT_PULLUP);
    // pinMode(enc_B, INPUT_PULLUP);
    
    gpio_set_irq_enabled_with_callback(motors[0].enc_A, GPIO_IRQ_EDGE_RISE, true, updateEncoderRightHip);
}

void loop() {

    // // recieves data in every loop 
    // std::vector<uint8_t> payload = receive_data();

    // // stores data, called every loop
    // store_data(payload);
    
    // // spin the callbacks
    // t1.spin();

}      




