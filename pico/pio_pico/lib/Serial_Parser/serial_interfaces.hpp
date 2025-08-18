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
    float hip;
    float shoulder;
    float elbow;
    float wrist_yaw;
    float wrist;

};

size_t get_packet_size(uint8_t id) {
    switch (id) {
        case CMD_VEL: return sizeof(CmdVel);
        case ODOM:    return sizeof(Odometry);
        case ANGLES:    return sizeof(Angles);

        default:      return 0; // unknown
    }
}
