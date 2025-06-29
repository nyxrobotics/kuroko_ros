#!/usr/bin/env python
import time

import rospy
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty


class GazeboUnpauser:
    def __init__(self, expected_models):
        rospy.init_node('unpause_gazebo_physics')
        self.expected_models = expected_models
        self.model_names = []
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

    def model_states_callback(self, msg):
        self.model_names = msg.name

    def wait_for_models(self, timeout=60):
        rospy.loginfo("Waiting for models to spawn: %s", self.expected_models)
        start_time = time.time()
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            if all(model in self.model_names for model in self.expected_models):
                rospy.loginfo("All expected models have spawned.")
                return True
            rate.sleep()

        rospy.logerr("Timeout waiting for models: %s", self.expected_models)
        return False

    def unpause_physics(self):
        rospy.loginfo("Waiting for /gazebo/unpause_physics service...")
        try:
            rospy.wait_for_service('/gazebo/unpause_physics', timeout=10)
            unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            unpause()
            rospy.loginfo("Called /gazebo/unpause_physics.")
        except rospy.ROSException:
            rospy.logerr("Timeout waiting for service.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


if __name__ == '__main__':
    model_str = rospy.get_param("~expected_models", "")
    expected_models = [m.strip() for m in model_str.split(",") if m.strip()]
    unpauser = GazeboUnpauser(expected_models)

    if unpauser.wait_for_models(timeout=60):
        unpauser.unpause_physics()
