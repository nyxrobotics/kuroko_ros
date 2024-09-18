[ control info ]
control_cycle = 10  # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/ttyDynamixel | 2000000   | chest

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME             | BULK READ ITEMS
dynamixel | /dev/ttyDynamixel | 1   | XM430-W210     | 2.0      | chest                | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 4   | XM430-W210     | 2.0      | shoulder_r_pitch     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 16  | XM430-W210     | 2.0      | shoulder_r_roll      | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 18  | XM430-W210     | 2.0      | elbow_r_front        | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 17  | XM430-W210     | 2.0      | elbow_r_rear         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 5   | XM430-W210     | 2.0      | shoulder_l_pitch     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 19  | XM430-W210     | 2.0      | shoulder_l_roll      | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 21  | XM430-W210     | 2.0      | elbow_l_front        | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 20  | XM430-W210     | 2.0      | elbow_l_rear         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 2   | XM430-W210     | 2.0      | hip_r_roll           | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 6   | XM430-W210     | 2.0      | hip_r_pitch          | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 7   | XM540-W150     | 2.0      | thigh_r_active       | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 8   | XM540-W150     | 2.0      | shin_r_active        | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 9   | XM430-W210     | 2.0      | ankle_r_roll         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 10  | XM430-W210     | 2.0      | ankle_r_yaw          | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 3   | XM430-W210     | 2.0      | hip_l_roll           | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 11  | XM430-W210     | 2.0      | hip_l_pitch          | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 12  | XM540-W150     | 2.0      | thigh_l_active       | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 13  | XM540-W150     | 2.0      | shin_l_active        | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 14  | XM430-W210     | 2.0      | ankle_l_roll         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyDynamixel | 15  | XM430-W210     | 2.0      | ankle_l_yaw          | present_position, position_p_gain, position_i_gain, position_d_gain
