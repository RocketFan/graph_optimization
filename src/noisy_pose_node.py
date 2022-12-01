import rospy
import random

from abc import ABC, abstractmethod
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates

class PoseNoise(ABC):
    def __init__(self, offset):
        self.offset = offset

    def applay(self, pose: Pose):
        position = pose.position
        position.x += self.get_noise()
        position.y += self.get_noise()
        position.z += self.get_noise()

    @abstractmethod
    def get_noise(self) -> int:
        pass

class PoseRandomNoise(PoseNoise):
    def __init__(self, max):
        self.max = max

    def get_noise(self) -> int:
        return random.random() * self.max

class NoisyPositionNode:
    def __init__(self):
        self.noise = PoseRandomNoise(3)

        self.sub_states = rospy.Subscriber(
            "/ground_truth", ModelStates, self.states_callback)
        self.pub_noisy_position = rospy.Publisher(
            "/noisy_position", ModelStates, queue_size=1)

    def run(self):
        rospy.spin()

    def states_callback(self, data: ModelStates):
        poses = data.pose

        for pose in poses:
            self.noise.applay(pose)

        self.publish_noisy_position(data)

    def publish_noisy_position(self, data: ModelStates):
        msg = ModelStates()
        msg.name = data.name
        msg.pose = data.pose
        self.pub_noisy_position.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('noisy_pose_node', anonymous=True)
        node = NoisyPositionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass