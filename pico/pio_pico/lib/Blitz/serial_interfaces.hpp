enum PacketID : uint8_t {

    JOINT_ANGLES = 1,
    JOINT_ANGLES_FEEDBACK = 2,
    JOINT_VEL = 3,
    

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

size_t get_packet_size(uint8_t id) {
    switch (id) {
        case JOINT_ANGLES:    return sizeof(JointAngles);
        case JOINT_ANGLES_FEEDBACK:    return sizeof(JointAngles);
        case JOINT_VEL:    return sizeof(JointAngles);

        default:      return 0; // unknown
    }
}
