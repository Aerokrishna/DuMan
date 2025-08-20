enum PacketID : uint8_t {
    CMD_VEL = 1,
    ODOM = 2,
    ANGLES = 3,

};

struct Odometry{
    uint8_t id;
    float x;
    float y;
    float yaw;
    float vibes;
};

struct CmdVel{
    uint8_t id;
    float vx;
    float vy;
    float vyaw;
};

struct Angles{
    uint8_t id;
    int16_t hip_l;
    int16_t shoulder_l;
    int16_t elbow_l;
    int16_t wrist_l;
    int16_t wrist_yaw_l;

    int16_t hip;
    int16_t shoulder;
    int16_t elbow;
    int16_t wrist;
    int16_t wrist_yaw;

};

size_t get_packet_size(uint8_t id) {
    switch (id) {
        case CMD_VEL: return sizeof(CmdVel);
        case ODOM:    return sizeof(Odometry);
        case ANGLES:    return sizeof(Angles);

        default:      return 0; // unknown
    }
}
