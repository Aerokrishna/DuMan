#include "include_all.cpp"

void setup(){
    Serial.begin(115200);

    for (int i = 0; i < 3; i++) {
        // pinMode(motors[i].dir_pin, OUTPUT);
        // pinMode(motors[i].pwm_pin, OUTPUT);

        pinMode(motors[i].enc_A, INPUT_PULLUP);
        pinMode(motors[i].enc_B, INPUT_PULLUP);

        servos[i].attachServo();

    }

    attachInterrupt(digitalPinToInterrupt(motors[0].enc_A), updateEncoderRightHip, RISING);
    attachInterrupt(digitalPinToInterrupt(motors[1].enc_A), updateEncoderRightShoulder, RISING);
    attachInterrupt(digitalPinToInterrupt(motors[2].enc_A), updateEncoderRightElbow, RISING);

}

void loop() {

    // recieves data in every loop 
    std::vector<uint8_t> payload = receive_data();

    // stores data, called every loop
    store_data(payload);
    
    // setMotorAngles();
    // setWristAngles();

    
    // // spin the callbacks
    // t1.spin();

}      




