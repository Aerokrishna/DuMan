
const int NUM_JOINTS = 12;
int encoder_val[NUM_JOINTS];
JointAngles joint_angles;

struct MotorAngles {
    float target_angle = 0.0;
    int encoder_val = 0;
    float current_angle = (encoder_val/1080.0f) * 360;
    int dir_pin;
    int pwm_pin;
    PID motor_pid;
    MotorAngles():motor_pid(0,0,0,0);
};

MotorAngles motor_joints[NUM_JOINTS/2];

// motor_angles[0]->motor_pid::PID(0,0,0,0);


struct ServoAngles {
    int target_angle = 0;
    int servo_pin;
    Servo servo;

};

ServoAngles servo_joints[(NUM_JOINTS/2)];
