from typing import List

MOVE_GROUP_ARM: str = "duman_arm"
MOVE_GROUP_GRIPPER: str = "end_effector"

def joint_names() -> List[str]:

    return ["hip",
            "shoulder",
            "elbow",
            "wrist_yaw",
            "wrist",
            "end_effector_joint"
            ]

def base_link_name() -> str:
    return "base"


def end_effector_name() -> str:
    return "end_effector"
