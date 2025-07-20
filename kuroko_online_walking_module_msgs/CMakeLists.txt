cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0048)
  cmake_policy(
    SET
    CMP0048
    NEW
  )
endif(POLICY CMP0048)
project(kuroko_online_walking_module_msgs)

find_package(
  catkin REQUIRED
  COMPONENTS std_msgs
             sensor_msgs
             geometry_msgs
             message_generation
)

add_message_files(
  FILES JointPose.msg
        KinematicsPose.msg
        FootStepCommand.msg
        FootStepArray.msg
        PreviewRequest.msg
        PreviewResponse.msg
        WalkingParam.msg
        Step2D.msg
        Step2DArray.msg
)

add_service_files(
  FILES GetJointPose.srv
        GetKinematicsPose.srv
        GetPreviewMatrix.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
)
