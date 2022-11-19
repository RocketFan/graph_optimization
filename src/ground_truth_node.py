#!/usr/bin/env python
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates

class GroundTruthNode:
    UAV_NAME_PREFIX = "iris"

    def __init__(self):
        rospy.init_node('ground_truth_node', anonymous=True)
        self.sub_states = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.states_callback)
        self.pub_ground_truth = rospy.Publisher(
            "/ground_truth", ModelStates, queue_size=1)
        rospy.spin()

    def states_callback(self, data: ModelStates):
        names = data.name
        poses = data.pose
        twists = data.twist
        ground_truth_compressed = np.array([[n, p, t] for n, p, t in zip(
            names, poses, twists) if n.startswith(self.UAV_NAME_PREFIX)])
        self.publish_groud_truth(ground_truth_compressed)

    def publish_groud_truth(self, ground_truth_array):
        msg = ModelStates()
        msg.name = ground_truth_array[:, 0]
        msg.pose = ground_truth_array[:, 1]
        msg.twist = ground_truth_array[:, 2]
        self.pub_ground_truth.publish(msg)

if __name__ == '__main__':
    ground_truth_node = GroundTruthNode()
