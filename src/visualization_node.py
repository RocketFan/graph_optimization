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
    def __init__(self, id: int, color: ColorRGBA, width: float, with_points=False):
        self.path = None
        self.id = (id + 1) * 1000
        self.color = color
        self.width = width
        self.with_points = with_points

    def set_path(self, path: Path):
        self.path = path

    def get_msg(self) -> MarkerArray:
        markers = MarkerArray()

        if self.path:
            points_marker = self.create_marker(self.id, Marker.POINTS)
            lines_marker = self.create_marker(self.id + 1, Marker.LINE_STRIP)

            for pose in self.path.poses:
                point: Point = pose.pose.position
                self.update_points_marker(points_marker, point)
                self.update_lines_marker(lines_marker, point)

            if self.with_points:
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
        marker.scale.x = self.width * 2
        marker.scale.y = self.width * 2

    def update_lines_marker(self, marker: Marker, point: Point):
        marker.points.append(point)
        marker.scale.x = self.width

class UAVVisualization:
    def __init__(self, name):
        print('Created: ', name)
        self.name = name
        self.id = extract_id(name) * 1000
        
        random_color = self.draw_color()
        blue_color = self.get_color(0, 0, 1, 0.5)
        red_color = self.get_color(1, 0, 0, 0.5)
        self.waypoints_path_marker = PathMarker(self.id, random_color, 0.1, True)
        self.ground_truth_path_marker = PathMarker(self.id + 1, blue_color, 0.025)
        self.noisy_path_marker = PathMarker(self.id + 2, red_color, 0.025)

        self.waypoints_path_sub = rospy.Subscriber(f'/{name}/path', Path, self.waypoints_path_callback)
        self.waypoints_path_pub = rospy.Publisher(f'/{name}/visualization/path', MarkerArray, queue_size=10)
        self.ground_truth_path_sub = rospy.Subscriber(f'/{name}/ground_truth/path', Path, self.ground_truth_path_callback)
        self.ground_truth_path_pub = rospy.Publisher(f'/{name}/visualization/ground_truth/path', MarkerArray, queue_size=10)
        self.noisy_path_sub = rospy.Subscriber(f'/{name}/noisy/path', Path, self.noisy_path_callback)
        self.noisy_path_pub = rospy.Publisher(f'/{name}/visualization/noisy/path', MarkerArray, queue_size=10)

    def get_waypoints_path_markers(self) -> list:
        return self.waypoints_path_marker.get_msg().markers

    def get_ground_truth_path_markers(self) -> list:
        return self.ground_truth_path_marker.get_msg().markers

    def get_noisy_path_markers(self) -> list:
        return self.noisy_path_marker.get_msg().markers

    def draw_color(self) -> ColorRGBA:
        color = ColorRGBA()
        color.r = random.random()
        color.g = random.random()
        color.b = random.random()
        color.a = 1

        return color

    def get_color(self, r, g, b, a):
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a

        return color

    def waypoints_path_callback(self, path: Path):
        self.waypoints_path_marker.set_path(path)

    def ground_truth_path_callback(self, path: Path):
        self.ground_truth_path_marker.set_path(path)

    def noisy_path_callback(self, path: Path):
        self.noisy_path_marker.set_path(path)

    def publish(self):
        waypoints_path_msg = self.waypoints_path_marker.get_msg()
        self.waypoints_path_pub.publish(waypoints_path_msg)

        ground_truth_path_msg = self.ground_truth_path_marker.get_msg()
        self.ground_truth_path_pub.publish(ground_truth_path_msg)

        noisy_path_msg = self.noisy_path_marker.get_msg()
        self.noisy_path_pub.publish(noisy_path_msg)
        

class VisualizationNode:
    def __init__(self):
        self.rate = rospy.Rate(30)
        self.uav_names = []
        self.uavs = []
        self.update_timer = rospy.Timer(rospy.Duration(1), lambda _: self.update())

        self.waypoints_path_pub = rospy.Publisher('/visualization/path', MarkerArray, queue_size=10)
        self.ground_truth_path_pub = rospy.Publisher('/visualization/ground_truth/path', MarkerArray, queue_size=10)
        self.noisy_path_pub = rospy.Publisher('/visualization/noisy/path', MarkerArray, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            self.visualize()
            self.rate.sleep()

    def visualize(self):
        for uav in self.uavs:
            uav.publish()

        self.publish_waypoints_path()
        self.publish_ground_truth_path()
        self.publish_noisy_path()

    def publish_waypoints_path(self):
        markers_msg = MarkerArray()

        for uav in self.uavs:
            markers = uav.get_waypoints_path_markers()
            markers_msg.markers += markers

        self.waypoints_path_pub.publish(markers_msg)

    def publish_ground_truth_path(self):
        markers_msg = MarkerArray()

        for uav in self.uavs:
            markers = uav.get_ground_truth_path_markers()
            markers_msg.markers += markers

        self.ground_truth_path_pub.publish(markers_msg)

    def publish_noisy_path(self):
        markers_msg = MarkerArray()

        for uav in self.uavs:
            markers = uav.get_noisy_path_markers()
            markers_msg.markers += markers

        self.noisy_path_pub.publish(markers_msg)

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