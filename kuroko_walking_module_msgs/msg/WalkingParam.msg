####### walking init pose #######
float32 init_x_offset
float32 init_y_offset
float32 init_z_offset
float32 init_roll_offset
float32 init_pitch_offset
float32 init_yaw_offset
float32 init_hip_pitch_offset

####### time parameter #####
float32 period_time
float32 dsp_ratio
float32 step_fb_ratio

########## walking parameter ########
float32 x_move_amplitude
float32 y_move_amplitude
float32 z_move_amplitude
float32 angle_move_amplitude
bool move_aim_on

########## balance parameter ##########
bool balance_enable
float32 balance_hip_roll_gain
float32 balance_knee_gain
float32 balance_ankle_roll_gain
float32 balance_ankle_pitch_gain

########## swing parameter ##########
float32 y_swing_amplitude
float32 z_swing_amplitude
float32 chest_swing_amplitude
float32 shoulder_swing_amplitude
float32 hip_swing_amplitude
float32 hip_swing_phase

########## gain parameter ##########
int32 p_gain
int32 i_gain
int32 d_gain
