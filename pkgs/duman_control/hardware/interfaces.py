from blitz_helper import BlitzInterfaces
blitz_interfaces = {str : BlitzInterfaces}

blitz_interfaces = {
    "joint_angles": BlitzInterfaces(
        msg_id=1,
        struct_fmt="ffffffffffff",
        from_mcu=False
    ),
}
