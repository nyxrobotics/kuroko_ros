#!/usr/bin/env python
import time

import rospy
from gazebo_msgs.srv import GetModelState
from std_srvs.srv import Empty


class GazeboUnpauser:
    def __init__(self, expected_models):
        self.expected_models = expected_models

    def wait_for_models(self, timeout=None):
        rospy.logwarn("Waiting for models to spawn: %s", self.expected_models)

        try:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=10)
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        except rospy.ROSException:
            rospy.logerr("Service /gazebo/get_model_state not available")
            return False

        start_time = time.time()

        while not rospy.is_shutdown():
            all_found = True

            for model in self.expected_models:
                try:
                    res = get_model_state(model, "")
                    if not res.success:
                        rospy.loginfo("Model '%s' not found yet", model)
                        all_found = False
                        break
                except rospy.ServiceException as e:
                    rospy.loginfo("Service call exception for model '%s': %s", model, str(e))
                    all_found = False
                    break

            if all_found:
                rospy.loginfo("All expected models have spawned.")
                return True

            if timeout is not None and (time.time() - start_time) > timeout:
                rospy.loginfo("Still waiting after %.1f seconds..." % timeout)
                start_time = time.time()

            time.sleep(0.5)

        rospy.logerr("ROS shutdown before all models spawned.")
        return False

    def unpause_physics(self):
        rospy.loginfo("Waiting for /gazebo/unpause_physics service...")
        try:
            rospy.wait_for_service('/gazebo/unpause_physics', timeout=10)
            unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            unpause()
            rospy.loginfo("Physics unpaused.")
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr("Failed to unpause physics: %s", e)


if __name__ == '__main__':
    rospy.init_node('unpause_physics')

    expected_str = rospy.get_param("~expected_models", "")
    wait_after_spawn = rospy.get_param("~wait_after_spawn", 1.0)
    expected_models = expected_str.split()

    rospy.loginfo("expected_models raw string: '%s'", expected_str)
    rospy.loginfo("expected_models list: %s", expected_models)

    if not expected_models:
        rospy.logwarn("No expected_models specified; proceeding immediately.")

    unpauser = GazeboUnpauser(expected_models)

    if unpauser.wait_for_models(timeout=60):
        rospy.loginfo("Waiting %.1f seconds after model spawn..." % wait_after_spawn)
        time.sleep(wait_after_spawn)
        unpauser.unpause_physics()
