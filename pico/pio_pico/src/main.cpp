#include "include_all.cpp"

void setup(){

    // serial begin
    Serial.begin(115200);

    pinMode(motor_pwm_, OUTPUT);
    pinMode(motor_dir, OUTPUT);

    pinMode(enc_A, INPUT_PULLUP);
    pinMode(enc_B, INPUT_PULLUP);
    
    gpio_set_irq_enabled_with_callback(enc_A, GPIO_IRQ_EDGE_RISE, true, updateEncoder);
}

void loop() {

    // recieves data in every loop 
    std::vector<uint8_t> payload = receive_data();

    // stores data, called every loop
    store_data(payload);
    
    // spin the callbacks
    t1.spin();

}      




