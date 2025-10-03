from typing import List
import numpy as np

MOVE_GROUP_ARM: str = "duman_right"
MOVE_GROUP_GRIPPER: str = "ee_right"

joint_min = np.array([-1.57, -0.1, -2.36, -3.14, -3.14, -3.14])
joint_max = np.array([0.78, 1.57, 1.57, 3.14, 3.14, 3.14])

def joint_names() -> List[str]:

    return ["right_hip",
            "right_shoulder",
            "right_elbow",
            "right_wrist1",
            "right_wrist2",
            "right_wrist3"
            ]

def base_link_name() -> str:
    return "base"

def end_effector_name() -> str:
    return "ee_right"

def joint_goal_valid(joint : np.ndarray) -> bool:
    return np.all((joint >= joint_min) & (joint <= joint_max))