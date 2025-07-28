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
float32 step_forward_back_ratio

########## walking parameter ########
float32 x_step
float32 y_step
float32 foot_height
float32 yaw_step

########## balance parameter ##########
bool balance_enable
float32 balance_gyro_x_gain
float32 balance_gyro_y_gain
float32 balance_gyro_pitch_gain
float32 balance_gyro_roll_gain
float32 balance_gyro_zx_gain
float32 balance_gyro_zy_gain
float32 balance_acc_x_gain
float32 balance_acc_y_gain
float32 balance_acc_pitch_gain
float32 balance_acc_roll_gain
float32 balance_acc_zx_gain
float32 balance_acc_zy_gain

########## swing parameter ##########
float32 y_swing_amplitude
float32 z_swing_amplitude
float32 roll_swing_amplitude
float32 roll_swing_phase

float32 hip_swing_up_amplitude
float32 hip_swing_down_amplitude

float32 chest_swing_amplitude
float32 shoulder_swing_amplitude

########## gain parameter ##########
int32 p_gain
int32 i_gain
int32 d_gain
