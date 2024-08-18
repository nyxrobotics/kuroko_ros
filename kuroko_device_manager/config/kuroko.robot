[ control info ]
control_cycle = 10  # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/ttyUSB0 | 57600     | waist

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME             | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1   | XM430-W210     | 2.0      | waist                | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 2   | XM430-W210     | 2.0      | hip_r_roll           | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 3   | XM430-W210     | 2.0      | hip_l_roll           | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 4   | XM430-W210     | 2.0      | shoulder_r_pitch     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 5   | XM430-W210     | 2.0      | shoulder_l_pitch     | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 6   | XM430-W210     | 2.0      | hip_r_pitch          | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 7   | XM540-W150     | 2.0      | thigh_r_front_active | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 8   | XM540-W150     | 2.0      | shin_r_active        | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 9   | XM430-W210     | 2.0      | ankle_r_roll         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 10  | XM430-W210     | 2.0      | ankle_r_yaw          | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 11  | XM430-W210     | 2.0      | hip_l_pitch          | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 12  | XM540-W150     | 2.0      | thigh_l_front_active | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 13  | XM540-W150     | 2.0      | shin_l_active        | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 14  | XM430-W210     | 2.0      | ankle_l_roll         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 15  | XM430-W210     | 2.0      | ankle_l_yaw          | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 16  | XM430-W210     | 2.0      | shoulder_r_roll      | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 17  | XM430-W210     | 2.0      | elbow_r_rear         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 18  | XM430-W210     | 2.0      | elbow_r_front        | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 19  | XM430-W210     | 2.0      | shoulder_l_roll      | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 20  | XM430-W210     | 2.0      | elbow_l_rear         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 21  | XM430-W210     | 2.0      | elbow_l_front        | present_position, position_p_gain, position_i_gain, position_d_gain
