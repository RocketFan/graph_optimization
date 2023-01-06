#!/usr/bin/env python

import rospy
import numpy as np
import os

from enum import Enum, auto
from time import sleep

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandHome, CommandHomeRequest
from mavros_msgs.srv import ParamSet, ParamSetRequest
from mavros_msgs.msg import State
from nav_msgs.msg import Path


def extract_id(name: str):
    id = name.replace('uav', '')
    return int(id)


class WayPoint:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return f'[x: {self.x}, y: {self.y}, z: {self.z}]'

class Waypoints:
    def __init__(self, name):
        self.name = name
        self.id = extract_id(name)
        self.path = os.path.dirname(__file__) + '/../waypoints'
        self.points = []
        self.path_pub = rospy.Publisher(f'/{self.name}/path', Path, queue_size=10)

    def load(self, type: str):
        with open(f'{self.path}/{type}/{self.id}.txt', 'r') as f:
            for line in f.readlines():
                point = self.parse_point(line)
                self.points.append(point)
                rospy.loginfo(f'[{self.name}] Load point: {point}')

    def parse_point(self, line: str) -> Point:
        line_array = line.split(' ')
        point = WayPoint(float(line_array[0]),
                      float(line_array[1]),
                      float(line_array[2]))

        return point

    def publish(self):
        path = Path()
        path.header.frame_id = 'map'

        for waypoint in self.points:
            point = Point()
            point.x = waypoint.x
            point.y = waypoint.y
            point.z = waypoint.z
            pose = PoseStamped()
            pose.pose.position = point
            path.poses.append(pose)

        self.path_pub.publish(path)


class Drone:
    def __init__(self, name):
        self.name = name
        self.flying = False
        self.fly_timer = None

        mavros_prefix = f'/{self.name}/mavros'
        self.arming_client = rospy.ServiceProxy(
            f'{mavros_prefix}/cmd/arming', CommandBool)
        self.set_home_client = rospy.ServiceProxy(
            f'{mavros_prefix}/cmd/set_home', CommandHome)
        self.set_mode_client = rospy.ServiceProxy(
            f'{mavros_prefix}/set_mode', SetMode)
        self.set_param_client = rospy.ServiceProxy(
            f'{mavros_prefix}/param/set', ParamSet)

        self.local_point_pub = rospy.Publisher(
            f'{mavros_prefix}/setpoint_position/local', PoseStamped, queue_size=10)

        self.global_position_sub = rospy.Subscriber(
            f'{mavros_prefix}/global_position/global', NavSatFix, self.global_position_callback)
        self.local_position_sub = rospy.Subscriber(
            f'{mavros_prefix}/local_position/pose', PoseStamped, self.local_position_callback)
        self.state_sub = rospy.Subscriber(
            f'{mavros_prefix}/state', State, self.state_callback)

        self.global_position_msg: NavSatFix = None
        self.local_position_msg: PoseStamped = None
        self.state_msg: State = None

    def is_ready(self):
        return (self.global_position_msg != None
                and self.local_position_msg != None
                and self.state_msg != None)

    def is_connected(self):
        return self.state_msg.connected

    def is_armed(self):
        return self.state_msg.armed

    def is_offboard_set(self):
        return self.state_msg.mode == "OFFBOARD"

    def is_flying(self):
        return self.flying

    def is_reached(self, x, y, z):
        local_position = self.local_position_msg.pose.position
        position = np.array(
            [local_position.x, local_position.y, local_position.z])
        desired_position = np.array([x, y, z])
        diff_position = np.linalg.norm(desired_position - position)

        return abs(diff_position) < 0.15

    def global_position_callback(self, msg: NavSatFix):
        self.global_position_msg = msg

    def local_position_callback(self, msg: PoseStamped):
        self.local_position_msg = msg

    def state_callback(self, msg: State):
        self.state_msg = msg

    def init_home(self):
        req = CommandHomeRequest()
        req.current_gps = True
        res = self.set_home_client(req)
        rospy.loginfo(
            f'[{self.name}] Init home sucess: {res.success}, result: {res.result}')

    def stop(self):
        if self.fly_timer:
            self.fly_timer.shutdown()

    def init_offboard(self):
        self.fly_to_point(0, 0, 0)

    def arm(self):
        res = self.arming_client(True)
        rospy.loginfo(
            f'[{self.name}] Arm sucess: {res.success}, result: {res.result}')

    def takeoff(self, altitude):
        self.stop()
        self.flying = True
        self.fly_timer = rospy.Timer(rospy.Duration(
            0.05), lambda _: self.fly_to_point_callback(0, 0, altitude))

    def land(self, x, y):
        self.stop()
        self.flying = True
        self.fly_timer = rospy.Timer(rospy.Duration(
            0.05), lambda _: self.fly_to_point_callback(x, y, 2))
        self.land_timer = rospy.Timer(rospy.Duration(
            0.05), lambda _: self.land_callback())

    def land_callback(self):
        if not self.is_flying():
            self.stop()
            self.land_timer.shutdown()

    def fly_to_point(self, x, y, z):
        self.stop()
        self.flying = True
        self.fly_timer = rospy.Timer(rospy.Duration(
            0.05), lambda _: self.fly_to_point_callback(x, y, z))

    def fly_to_point_callback(self, x, y, z):
        if self.is_reached(x, y, z):
            self.flying = False

        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        self.local_point_pub.publish(msg)

    def set_offboard(self):
        req = SetModeRequest()
        req.custom_mode = "OFFBOARD"
        res = self.set_mode_client(req)
        rospy.loginfo(f'[{self.name}] Set OFFBOARD sucess: {res.mode_sent}')

    def set_param(self, param_id: str, value):
        req = ParamSetRequest()
        req.param_id = param_id

        if isinstance(value, int):
            req.value.integer = value
        else:
            req.value.real = value

        res = self.set_param_client(req)
        rospy.loginfo(
            f'[{self.name}] Set param {param_id} sucess: {res.success}, value: {value}')

    def print_mode(self):
        rospy.loginfo(f'MODE: {self.state_msg.mode}')


class MissionState(Enum):
    NONE = auto()
    INIT = auto()
    ARM = auto()
    SET_MODE = auto()
    TAKEOFF = auto()
    FLY = auto()
    LAND = auto()


class DronesControllNode:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.drone_name = rospy.get_param('~uav_name')
        rospy.logwarn(os.path.abspath(__file__))
        # self.drone_name = rospy.get_namespace()
        self.drone = Drone(self.drone_name)
        self.state = MissionState.INIT
        self.waypoints = Waypoints(self.drone_name)
        self.waypoints_index = 0

    def run(self):
        self.waypoints.load('random')
        self.wait_for_msgs()
        self.wait_for_connection()

        while not rospy.is_shutdown():
            self.run_state()
            self.waypoints.publish()
            self.rate.sleep()

    def wait_for_msgs(self):
        while not rospy.is_shutdown():
            if self.drone.is_ready():
                break
            else:
                self.rate.sleep()

    def wait_for_connection(self):
        while not rospy.is_shutdown():
            if self.drone.is_connected():
                break
            else:
                self.rate.sleep()

    def run_state(self):
        if self.state == MissionState.INIT:
            self.init_drone()
            self.state = MissionState.ARM
        elif self.state == MissionState.ARM:
            self.drone.arm()
            self.state = MissionState.SET_MODE
        elif self.state == MissionState.SET_MODE:
            self.set_mode()
        elif self.state == MissionState.TAKEOFF:
            self.takeoff(5)
        elif self.state == MissionState.FLY:
            self.fly_waypoints()
        elif self.state == MissionState.LAND:
            self.drone.land(0, 0)
            self.state = MissionState.NONE

    def init_drone(self):
        self.drone.init_home()
        self.drone.init_offboard()
        self.drone.set_param("COM_RCL_EXCEPT", 4)
        self.drone.set_param("MPC_XY_VEL_MAX", 1.0)
        self.drone.set_param("MPC_Z_VEL_MAX_UP", 1.0)
        self.state = MissionState.ARM

    def set_mode(self):
        if not self.drone.is_offboard_set():
            self.drone.set_offboard()
        else:
            self.state = MissionState.TAKEOFF

    def takeoff(self, altitude) -> MissionState:
        self.drone.takeoff(altitude)
        self.state = MissionState.NONE

        self.takeoff_timer = rospy.Timer(rospy.Duration(
            0.05), lambda _: self.takeoff_callback())

    def takeoff_callback(self):
        if not self.drone.is_flying():
            self.state = MissionState.FLY
            self.takeoff_timer.shutdown()

    def fly_waypoints(self):
        if not self.drone.is_flying():
            if self.waypoints_index < len(self.waypoints.points):
                self.fly_to_next_waypoint()
                self.waypoints_index += 1
            else:
                self.state = MissionState.LAND

    def fly_to_next_waypoint(self):
        point = self.waypoints.points[self.waypoints_index]
        rospy.loginfo(f'[{self.drone_name}] Fly to waypoint: {point}')
        self.drone.fly_to_point(point.x, point.y, point.z)

if __name__ == '__main__':
    try:
        rospy.init_node('drone_controll_node', anonymous=True)
        node = DronesControllNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
