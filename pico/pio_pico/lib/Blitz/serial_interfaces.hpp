enum PacketID : uint8_t {

    JOINT_ANGLES = 1,
    JOINT_ANGLES_FEEDBACK = 2,

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

size_t get_packet_size(uint8_t id) {
    switch (id) {
        case JOINT_ANGLES:    return sizeof(JointAngles);
        case JOINT_ANGLES_FEEDBACK:    return sizeof(JointAngles);
        default:      return 0; // unknown
    }
}
