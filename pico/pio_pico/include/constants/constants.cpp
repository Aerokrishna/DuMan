JointAngles joint_angles;
JointAngles joint_angles_feedback;
unsigned long new_vel_data;
bool vel_cmd;

GripState grip_state;

//right
// int open_angle = 60;
// int grip_angle = 90;

//left

int open_angle = 40;
int grip_angle = 65;

Servo gripper_servo;