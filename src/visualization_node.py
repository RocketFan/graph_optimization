import rospy
import rostopic

from nav_msgs.msg import Path

class PathMarker:
    def __init__(self):
        self.path = None

    def set_path(self, path: Path):
        self.path = path

class UAVVisualization:
    def __init__(self, name):
        print('Created: ', name)
        self.name = name
        self.path_marker = PathMarker()

        self.path_sub = rospy.Subscriber(f'/{name}/path', Path, self.path_callback)

    def path_callback(self, path: Path):
        print(self.name)
        self.path_marker.set_path(path)

class VisualizationNode:
    def __init__(self):
        self.rate = rospy.Rate(30)
        self.uav_names = []
        self.uavs = []
        self.update_timer = rospy.Timer(rospy.Duration(1), lambda _: self.update())

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

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