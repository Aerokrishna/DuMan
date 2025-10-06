enum PacketID : uint8_t {

    JOINT_ANGLES = 1,
    PID_CMD = 3,
    PID_FEEDBACK = 4,
};

#pragma pack(push, 1)
struct JointAngles {
    uint8_t id;
    float left_hip;
    float left_shoulder;
    float left_elbow;
    float left_wrist1;
    float left_wrist2;
    float left_wrist3;

    float right_hip;
    float right_shoulder;
    float right_elbow;
    float right_wrist1;
    float right_wrist2;
    float right_wrist3;
};  
#pragma pack(pop)

#pragma pack(push, 1)
struct PIDTest {
    uint8_t id;
    float Kp;
    float Ki;
    float Kd;
    float target_angle;
    float time_;
};  
#pragma pack(pop)

#pragma pack(push, 1)
struct PIDFeedback {
    uint8_t id;
    float setpoint;
    float current;
    int16_t motor_pwm;
    float elapsed_time;

};  
#pragma pack(pop)

size_t get_packet_size(uint8_t id) {
    switch (id) {
        case PID_CMD:    return sizeof(PIDTest);
        case PID_FEEDBACK:    return sizeof(PIDFeedback);
        case JOINT_ANGLES:    return sizeof(PIDFeedback);

        default:      return 0; // unknown
    }
}
