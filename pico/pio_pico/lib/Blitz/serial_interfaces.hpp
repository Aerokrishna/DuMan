enum PacketID : uint8_t {

    JOINT_ANGLES = 1,
    JOINT_ANGLES_FEEDBACK = 2,

    PID_CMD = 3,
    PID_FEEDBACK = 4,
};

#pragma pack(push, 1)
struct JointAngles {
    uint8_t id;
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

#pragma pack(push, 1)
struct Counter {
    uint8_t id;
    int16_t a;
    int16_t b;
    float c;
    float d;
};  
#pragma pack(pop)

size_t get_packet_size(uint8_t id) {
    switch (id) {
        case PID_CMD:    return sizeof(PIDTest);
        case PID_FEEDBACK:    return sizeof(PIDFeedback);
        case JOINT_ANGLES:    return sizeof(JointAngles);
        case JOINT_ANGLES_FEEDBACK:    return sizeof(JointAngles);

        default:      return 0; // unknown
    }
}
