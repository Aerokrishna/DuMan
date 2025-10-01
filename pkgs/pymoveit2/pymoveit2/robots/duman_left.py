from typing import List

MOVE_GROUP_ARM: str = "duman_left"
MOVE_GROUP_GRIPPER: str = "ee_left"

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
