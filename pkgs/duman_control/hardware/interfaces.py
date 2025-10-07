from blitz_helper import BlitzInterfaces
blitz_interfaces = {str : BlitzInterfaces}

blitz_interfaces = {
    "joint_angles_right": BlitzInterfaces(
        msg_id=1,
        struct_fmt="ffffff",
        from_mcu=False
    ),

    "joint_angles_right_feedback": BlitzInterfaces(
        msg_id=2,
        struct_fmt="ffffff",
        from_mcu=True
    ),
}
