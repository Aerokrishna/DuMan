from typing import List
import numpy as np

MOVE_GROUP_ARM: str = "duman_left"
MOVE_GROUP_GRIPPER: str = "ee_left"

joint_min = np.array([-0.78, -1.57, -1.57, -3.14, -3.14, -3.14])
joint_max = np.array([1.57, 0.1, 2.36, 3.14, 3.14, 3.14])

def joint_names() -> List[str]:

    return ["left_hip",
            "left_shoulder",
            "left_elbow",
            "left_wrist1",
            "left_wrist2",
            "left_wrist3"
            ]

def base_link_name() -> str:
    return "base"

def end_effector_name() -> str:
    return "ee_left"

def joint_goal_valid(joint : np.ndarray) -> bool:
    return np.all((joint >= joint_min) & (joint <= joint_max))