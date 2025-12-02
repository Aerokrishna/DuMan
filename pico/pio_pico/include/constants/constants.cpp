JointAngles joint_angles;
JointAngles joint_angles_feedback;
unsigned long new_vel_data;
bool vel_cmd;

GripState grip_state;

//right
int open_angle = 35;
int grip_angle = 100;

//left

// int open_angle = 0;
// int grip_angle = 50;

Servo gripper_servo;