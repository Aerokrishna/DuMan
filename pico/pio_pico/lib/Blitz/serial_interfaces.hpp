enum PacketID : uint8_t {

    JOINT_ANGLES = 4,
    JOINT_ANGLES_FEEDBACK = 5,
    JOINT_VEL = 6,
    GRIP_STATE = 8

    // // left
    // JOINT_ANGLES = 1,
    // JOINT_ANGLES_FEEDBACK = 2,
    // JOINT_VEL = 3,
    // GRIP_STATE = 7
    
};

#pragma pack(push, 1)
struct JointAngles {
    uint8_t id;
    float hip;
    float shoulder;
    float elbow;
    float wrist1;
    float wrist2;
    float wrist3;
};  
#pragma pack(pop)

#pragma pack(push, 1)
struct GripState {
    uint8_t id;
    bool grip_state; 
};  
#pragma pack(pop)

size_t get_packet_size(uint8_t id) {
    switch (id) {
        case JOINT_ANGLES:    return sizeof(JointAngles);
        case JOINT_ANGLES_FEEDBACK:    return sizeof(JointAngles);
        case JOINT_VEL:    return sizeof(JointAngles);
        case GRIP_STATE:    return sizeof(GripState);

        default:      return 0; // unknown
    }
}
