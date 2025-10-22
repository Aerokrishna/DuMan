JointAngles joint_angles;
JointAngles joint_angles_feedback;
unsigned long new_vel_data;
bool vel_cmd;

GripState grip_state;
int open_angle = 0;
int grip_angle = 90;
Servo gripper_servo;