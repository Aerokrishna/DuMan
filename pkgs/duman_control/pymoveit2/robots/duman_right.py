from typing import List

MOVE_GROUP_ARM: str = "duman_right"
MOVE_GROUP_GRIPPER: str = "ee_right"

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