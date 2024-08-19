#ifndef KUROKO_KINEMATICS_DEFINE_H_
#define KUROKO_KINEMATICS_DEFINE_H_

namespace motion_control
{
#define MAX_JOINT_ID (38)
#define ALL_JOINT_ID (38)

#define MAX_ARM_ID (3)
#define MAX_LEG_ID (6)
#define MAX_ITER (5)

#define ID_HEAD_END (20)
#define ID_COB (29)
#define ID_TORSO (29)

#define ID_R_ARM_START (3)
#define ID_R_ARM_END (6)
#define ID_L_ARM_START (7)
#define ID_L_ARM_END (10)

#define ID_R_LEG_START (11)
#define ID_R_LEG_END (24)
#define ID_L_LEG_START (25)
#define ID_L_LEG_END (38)

#define GRAVITY_ACCELERATION (9.8)
}  // namespace motion_control

#endif /* KUROKO_KINEMATICS_DEFINE_H_ */
