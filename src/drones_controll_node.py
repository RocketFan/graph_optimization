#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL, CommandTOLRequest

class Drone:
    def __init__(self, name):
        self.name = name
        self.arming_client = rospy.ServiceProxy(f'/{self.name}/mavros/cmd/arming', CommandBool)
        self.takeoff_client = rospy.ServiceProxy(f'/{self.name}/mavros/cmd/takeoff', CommandTOL)
        self.local_point_pub = rospy.Publisher(f'/{self.name}/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    def arm(self):
        res = self.arming_client(True)
        rospy.loginfo(f'Arm {self.name} sucess: {res.success}, result: {res.result}')

    def takeoff(self, altitude):
        req = CommandTOLRequest()
        req.altitude = altitude

        res = self.takeoff_client(req)
        rospy.loginfo(f'Takeoff {self.name} sucess: {res.success}, result: {res.result}')

    def fly_to_point(self, x, y, z):
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        self.local_point_pub.publish(msg)

    def print_state(self):
        rospy.loginfo(f'State of {self.name}', )

class DronesControllNode:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.drone = Drone('uav1')

    def run(self):
        self.takeoff_all()

        while not rospy.is_shutdown():
            self.drone.fly_to_point(0, 0, 5)
            self.rate.sleep()

    def takeoff_all(self):
        self.drone.arm()
        self.drone.takeoff(5)

if __name__ == '__main__':
    try:
        rospy.init_node('drone_controll_node', anonymous=True)
        node = DronesControllNode()
        node.run()
    except rospy.ROSInterruptException:
        pass