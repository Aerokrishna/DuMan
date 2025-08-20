int enc_val_right = 0;
int enc_val_left = 0;
const int NUM_JOINTS = 10;

int servoPins[NUM_JOINTS] = {
  L_hip, L_shoulder, L_elbow, L_wrist, L_wrist_y,
  R_hip, R_shoulder, R_elbow, R_wrist, R_wrist_y
};

int currentAngles[NUM_JOINTS] = {
  home_L_hip, home_L_shoulder, home_L_elbow, home_L_wrist, home_L_wrist_y,
  home_R_hip, home_R_shoulder, home_R_elbow, home_R_wrist, home_R_wrist_y
};

int targetAngles[NUM_JOINTS] = {
  home_L_hip, home_L_shoulder, home_L_elbow, home_L_wrist, home_L_wrist_y,
  home_R_hip, home_R_shoulder, home_R_elbow, home_R_wrist, home_R_wrist_y
};

bool new_goal = false;
Servo servos[NUM_JOINTS];
