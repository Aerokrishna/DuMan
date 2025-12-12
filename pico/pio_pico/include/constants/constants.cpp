JointAngles joint_angles;
JointAngles joint_angles_feedback;
unsigned long new_vel_data;
bool vel_cmd;

GripState grip_state;
bool ungrip = true;
int ungrip_cnt = 0;
//right
int open_angle = 80;
int grip_angle = 110;

//left

// int open_angle = 100;
// int grip_angle = 70;

Servo gripper_servo;