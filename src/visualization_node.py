import rospy
import rostopic
import random

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

def extract_id(name: str):
    id = name.replace('uav', '')
    return int(id)

class PathMarker:
    def __init__(self, id: int, color: ColorRGBA):
        self.path = None
        self.id = id
        self.color = color

    def set_path(self, path: Path):
        self.path = path

    def get_msg(self) -> MarkerArray:
        markers = MarkerArray()

        if self.path:
            points_marker = self.create_marker(self.id, Marker.POINTS)
            lines_marker = self.create_marker(self.id * 1000, Marker.LINE_STRIP)

            for pose in self.path.poses:
                point: Point = pose.pose.position
                self.update_points_marker(points_marker, point)
                self.update_lines_marker(lines_marker, point)

            markers.markers.append(points_marker)
            markers.markers.append(lines_marker)

        return markers

    def create_marker(self, id, type):
        marker = Marker()
        marker.color = self.color
        marker.id = id
        marker.type = type
        marker.header.frame_id = self.path.header.frame_id

        return marker

    def update_points_marker(self, marker: Marker, point: Point):
        marker.points.append(point)
        marker.scale.x = 0.2
        marker.scale.y = 0.2

    def update_lines_marker(self, marker: Marker, point: Point):
        marker.points.append(point)
        marker.scale.x = 0.1

class UAVVisualization:
    def __init__(self, name):
        print('Created: ', name)
        self.name = name
        self.id = extract_id(name)
        self.color = self.draw_color()
        self.path_marker = PathMarker(self.id, self.color)

        self.path_sub = rospy.Subscriber(f'/{name}/path', Path, self.path_callback)
        self.path_pub = rospy.Publisher(f'/{name}/visualization/path1', MarkerArray, queue_size=10)

    def draw_color(self) -> ColorRGBA:
        color = ColorRGBA()
        color.r = random.random()
        color.g = random.random()
        color.b = random.random()
        color.a = 1

        return color

    def path_callback(self, path: Path):
        self.path_marker.set_path(path)

    def publish(self):
        msg = self.path_marker.get_msg()
        self.path_pub.publish(msg)

class VisualizationNode:
    def __init__(self):
        self.rate = rospy.Rate(30)
        self.uav_names = []
        self.uavs = []
        self.update_timer = rospy.Timer(rospy.Duration(1), lambda _: self.update())

    def run(self):
        while not rospy.is_shutdown():
            self.visualize()
            self.rate.sleep()

    def visualize(self):
        for uav in self.uavs:
            uav.publish()

    def update(self):
        new_uav_names = self.update_uav_names()
        self.add_uavs(new_uav_names)

    def update_uav_names(self):
        topics = rostopic.get_topic_list()[0]
        new_uav_names = []
        
        for topic in topics:
            topic_splitted = topic[0].split('/')
            uav_name = topic_splitted[1]

            if 'uav' in uav_name not in self.uav_names:
                new_uav_names.append(uav_name)
                self.uav_names.append(uav_name)

        return new_uav_names

    def add_uavs(self, new_uav_names):
        for uav_name in new_uav_names:
            uav = UAVVisualization(uav_name)
            self.uavs.append(uav)

if __name__ == '__main__':
    try:
        rospy.init_node('visualization_node', anonymous=True)
        node = VisualizationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass