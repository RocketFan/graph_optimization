import rospy
import rosgraph
import rostopic

class UWBSimNode:
    def __init__(self):
        rospy.init_node("uwb_sim_node")
        self.rate = rospy.Rate(30)

        self.get_uav_list()
        # self.sub_ground_truth = rospy.Subscriber()

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def get_uav_list(self):
        master = rosgraph.Master('/rostopic')
        _, sub_topic_list = rostopic.get_topic_list(master=master)
        print(sub_topic_list)

if __name__ == "__main__":
    try:
        node = UWBSimNode()
        node.run()
    except rospy.ROSInterruptException:
        pass